/*
  (c)2018 Pawel A. Hernik
  YouTube video: https://youtu.be/6tiiceEf7gE
 */

// *** CONNECTIONS ***
// N5110 LCD from left:
// #1 RST      - Pin 9
// #2 CS/CE    - Pin 10
// #3 DC       - Pin 8
// #4 MOSI/DIN - Pin 11
// #5 SCK/CLK  - Pin 13
// #6 VCC      - 3.3V/5.0V
// #7 LIGHT    - Pin 6 (optionally via PWM)
// #8 GND

// DHT11 pinout from left:
// VCC DATA NC GND

// RTC DS1307 and AT24C32 EEPROM
// I2C A4/A5

#define DHT11_PIN 14
#define BACKLIGHT 6

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// define USESPI in LCD driver header for HW SPI version
#include "N5110_SPI.h"
#if USESPI==1
#include <SPI.h>
#endif
N5110_SPI lcd(9,10,8); // RST,CS,DC

#include "c64enh_font.h"
#include "times_dig_16x24_font.h"
#include "term9x14_font.h"
#include "tinydig3x7sq_font.h"
#include "small4x7_font.h"
#include "small5x7_font.h"
//#include "small5x7bold_font.h"
//#include "fonts_all.h"

#include <Wire.h>

struct LogData {
  int hour,minute,second;
  int year,month,day,dayOfWeek;
  int humidity;
  float temperature;
};

// values stored in RTC DS1307 RAM (56 bytes in total)
#define REG_ADDR   0  // 0,1
#define REG_NUM    2  // 2,3
#define REG_LOGINT 4
#define REG_LIGHT  5
#define REG_MINT   6   // 6,7,8,9,10
#define REG_MAXT   11  // 11,12,13,14,15
#define REG_MINH   16  // 16,17,18,19,20
#define REG_MAXH   21  // 21,22,23,24,25

LogData cur,tmp,set;
//LogData minD,maxD; // daily
//LogData minM,maxM; // monthly
//LogData minY,maxY; // yearly
//LogData minT,maxT; // total

LogData minTemp,maxTemp;
LogData minHum,maxHum;

long menuTime=0;
long logTime=0;
long lightTime=0;
int logInterval = 30;     // in minutes
int backlight = 10;       // in seconds
int recNumMax = 4096/5;   // 819
int recAddr=0, recNum=0;

#define REAL_SLEEP_8S 9100L

#define BACKLIGHT_MAX 11  // for always on
#define BACKLIGHT_MIN 1   // below is always off

#define USEHW 1
#include "rtc.h"
#include "i2cflash.h"

// -------------------------
#define encoderPinA    2
#define encoderPinB    4
#define encoderButton  3
volatile int encoderPos = 0;

void initEncoder()
{
  encoderPos=0;
  pinMode(encoderPinA,   INPUT_PULLUP); 
  pinMode(encoderPinB,   INPUT_PULLUP); 
  pinMode(encoderButton, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoderInt, CHANGE);  // encoder pin on interrupt 0 = pin 2
  attachInterrupt(digitalPinToInterrupt(encoderButton), buttonInt, CHANGE);  // encoder pin on interrupt 1 = pin 3
}

void buttonInt()
{
  menuTime=24*1000;
  if(backlight>=BACKLIGHT_MIN) {
    lightTime = backlight*10*1000L;
    //analogWrite(BACKLIGHT,0);
    //digitalWrite(BACKLIGHT,0);
    PORTD &= ~B01000000; // faster
  }
}

void readEncoderInt()
{
  //(digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? encoderPos++ : encoderPos--;
  uint8_t pd = PIND & B10100; // pins 2 and 4 direct reading
  ((pd == B10100) || (pd == B00000)) ? encoderPos++ : encoderPos--;
  menuTime=24*1000;
  if(backlight>=BACKLIGHT_MIN) {
    lightTime = backlight*10*1000L;
    //analogWrite(BACKLIGHT,0);
    //digitalWrite(BACKLIGHT,0);
    PORTD &= ~B01000000; // faster
  }
}

int readButton()
{
  static int lastState = HIGH;
  int v=0, state=digitalRead(encoderButton);
  if(state==LOW && lastState==HIGH) v=1;
  lastState = state;
  return v;
}

// -------------------------
long readVcc() 
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}
// -------------------------

#define DHT_OK         0
#define DHT_CHECKSUM  -1
#define DHT_TIMEOUT   -2
int temp1,temp10;

int readDHT11(int pin)
{
  uint8_t bits[5];
  uint8_t bit = 7;
  uint8_t idx = 0;

  for (int i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(18);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT_PULLUP);

  // ACKNOWLEDGE or TIMEOUT
  unsigned int loopCnt = 10000;
  while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

  loopCnt = 10000;
  while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++) {
    loopCnt = 10000;
    while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

    unsigned long t = micros();
    loopCnt = 10000;
    while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

    if(micros() - t > 40) bits[idx] |= (1 << bit);
    if(bit == 0) {
      bit = 7;    // restart at MSB
      idx++;      // next byte!
    }
    else bit--;
  }

  cur.humidity = bits[0];
  temp1  = bits[2];
  temp10 = bits[3];
  cur.temperature = abs(temp1+temp10/10.0);

  if(bits[4] != bits[0]+bits[1]+bits[2]+bits[3]) return DHT_CHECKSUM;
  return DHT_OK;
}

// -------------------------

enum wdt_time {
	SLEEP_15MS,
	SLEEP_30MS,	
	SLEEP_60MS,
	SLEEP_120MS,
	SLEEP_250MS,
	SLEEP_500MS,
	SLEEP_1S,
	SLEEP_2S,
	SLEEP_4S,
	SLEEP_8S,
	SLEEP_FOREVER
};

ISR(WDT_vect) { wdt_disable(); }

void powerDown(uint8_t time)
{
  ADCSRA &= ~(1 << ADEN);  // turn off ADC
  if(time != SLEEP_FOREVER) { // use watchdog timer
    wdt_enable(time);
    WDTCSR |= (1 << WDIE);	
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // most power saving
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  // ... sleeping here
  sleep_disable();
  ADCSRA |= (1 << ADEN); // turn on ADC
}

// --------------------------------------------------------------------------
byte scr[84*4];  // frame buffer
byte scrWd = 84;
byte scrHt = 4;

void clrBuf()
{
  for(int i=0;i<scrWd*scrHt;i++) scr[i]=0;
}

void drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  if((x < 0) || (x >= scrWd) || (y < 0) || (y >= scrHt*8)) return;
  switch (color) 
  {
    case 1: scr[x+(y/8)*scrWd] |=  (1 << (y&7)); break;
    case 0: scr[x+(y/8)*scrWd] &= ~(1 << (y&7)); break; 
    case 2: scr[x+(y/8)*scrWd] ^=  (1 << (y&7)); break; 
  }
}

void drawLineV(int x, int y0, int y1)
{
  if(y1>y0)
    for(int y=y0; y<=y1; y++) drawPixel(x,y,1);
  else
    for(int y=y1; y<=y0; y++) drawPixel(x,y,1);
}

// --------------------------------------------------------------------------

char buf[35],buf2[15];

int first=1;
char *menuTxt[] = {
  "Clock",        // 0
  "Clock LowPow", // 1
  "Temperature",  // 2
  "Humidity",     // 3
  "Both",         // 4
  "Review logs",  // 5
  "Dump serial",  // 6
  "Show MinMax",  // 7
  "Reset MinMax", // 8
  "Graph Temp.",  // 9
  "Graph Humid.", // 10
  "Clear logs",   // 11
  "Set clock",    // 12
  "Log interval", // 13
  "Backlight",    // 14
//  "EEPROM dump",  // 6
//  "I2CFlash dmp", // 7
//  "RTC dump",     // 8
};

int numMenus=0;
int menuLine;
int menuStart;
int numScrLines = 6;
int menuMode = 1; // -1 -> menu of options, 0..n -> option
int oldPos = 0;

void getCurAddr()
{
  recAddr = readRTCMem(REG_ADDR) + (readRTCMem(REG_ADDR+1)<<8);
  recNum  = readRTCMem(REG_NUM)  + (readRTCMem(REG_NUM+1)<<8);
}

void clearLogAddrNum()
{
  recAddr=recNum=0;
  writeRTCMem(REG_ADDR,0); writeRTCMem(REG_ADDR+1,0);
  writeRTCMem(REG_NUM,0); writeRTCMem(REG_NUM+1,0);
}

void setup() 
{
  first = 1;
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);  // faster
  lcd.init();
  lcd.clrScr();
  for(int i=0;i<14;i++) pinMode(i, OUTPUT); 
  //CLKPR = 0x80; // lower internal clock frequency to tmp power
  //CLKPR = 0x02; // 0-16MHz, 1-8MHz, 2-4MHz, 3-2MHz, ..
  initEncoder();
  numMenus = sizeof(menuTxt)/sizeof(char*);
  //analogWrite(BACKLIGHT,0); // 0=max
  digitalWrite(BACKLIGHT,0); // on
  // check if recAddr and recNum are valid, if not reset logger
  getCurAddr();
  if(recAddr>=recNumMax*5 || recAddr<0 || (recAddr%5)!=0 || recNum<0 || recNum>recNumMax) {
    clearLogAddrNum();
    writeRTCMem(REG_LOGINT,30);
    writeRTCMem(REG_LIGHT,2); // = 20s
  }
  logInterval = readRTCMem(REG_LOGINT);
  backlight = readRTCMem(REG_LIGHT);
  logTime=0;
  lightTime = backlight*10*1000L;
  readMinMax();
}

int drawBatt(int x, int y, int wd, int perc)
{
  int w = wd*perc/100;
  lcd.fillWin(x,y,1+w,1,B01111111);
  x+=w+1;
  w=wd-w;
  if(w>0) {
    lcd.fillWin(x,y,w,1,B01000001);
    x+=w;
  }
  lcd.fillWin(x++,y,1,1,B01111111);
  lcd.fillWin(x++,y,1,1,B00011100);
  lcd.fillWin(x++,y,1,1,B00011100);
  return x;
}

// ---------------------------------------
int x;
long v;
char *dowTxt[] = {"??","Mon","Tue","Wed","Thu","Fri","Sat","Sun"};
char *dowLongTxt[] = {"??","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday","Sunday"};

void showClock(int seconds) 
{  
  lcd.setFont(Term9x14);
  if(seconds)
    snprintf(buf,25," %d:%02d:%02d ",cur.hour,cur.minute,cur.second);
  else
    snprintf(buf,25," %d:%02d ",cur.hour,cur.minute);
  lcd.printStr(ALIGN_CENTER, 0, buf);
  lcd.setFont(c64enh);
  snprintf(buf,25,"% 02d.%02d.%d ",cur.day,cur.month,cur.year+2000);
  lcd.printStr(ALIGN_CENTER, 2, buf);
  snprintf(buf,25,"  %s  ", dowLongTxt[cur.dayOfWeek]);
  lcd.printStr(ALIGN_CENTER, 3, buf);

  snprintf(buf,25," %d.%d'C ",(int)temp1,temp10);
  lcd.printStr(1, 5, buf);
  snprintf(buf,25," %d%% ",cur.humidity);
  lcd.printStr(ALIGN_RIGHT, 5, buf);

  dtostrf(v/1000.0,1,2,buf);
  x=drawBatt(17,4,20,constrain(map(v,2900,4200,0,100),0,100));
  lcd.setFont(Small4x7PL);
  lcd.setDigitMinWd(4);
  x=lcd.printStr(x+4, 4, buf);
  lcd.printStr(x+2, 4, "V");
}

void showTempOrHum(int h) 
{  
  lcd.setFont(c64enh);
  //if(h)lcd.printStr(ALIGN_CENTER, 0, "Humidity"); else lcd.printStr(ALIGN_CENTER, 0, "Temperature");
  buf[0]=0;
  strcat(buf," <"); dtostrf(h?minHum.humidity:minTemp.temperature,1,h?0:1,buf2); strcat(buf,buf2);
  strcat(buf,h?"% >":"' >"); dtostrf(h?maxHum.humidity:maxTemp.temperature,1,h?0:1,buf2); strcat(buf,buf2); strcat(buf,h?"% ":"' ");
  lcd.printStr(ALIGN_CENTER, 5, buf);

  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x=h?22:5;
  if(h) snprintf(buf,10,"%d",cur.humidity); else snprintf(buf,10,"%d:%d",temp1,temp10);
  x=lcd.printStr(x, 1, buf);
  lcd.setFont(Term9x14);
  if(h) lcd.printStr(x+2, 1, "%"); else lcd.printStr(x+1, 1, "`C");
}

void showBothTempHum() 
{  
  snprintf(buf,10,"%d",(int)cur.temperature);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x=lcd.printStr(0, 0, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(x+0, 0, "`C");

  snprintf(buf,10,"%d",cur.humidity);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  lcd.printStr(39, 3, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(ALIGN_RIGHT, 3, "%");

  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);
  snprintf(buf,10,".%d",temp10);
  lcd.printStr(x+0, 2, buf);

  lcd.setFont(Small5x7PL);
  lcd.setDigitMinWd(5);

  buf[0]=0; strcat(buf,"<"); dtostrf(minTemp.temperature,1,1,buf2); strcat(buf,buf2); strcat(buf,"'");
  lcd.printStr(ALIGN_RIGHT, 0, buf);
  buf[0]=0; strcat(buf,">"); dtostrf(maxTemp.temperature,1,1,buf2); strcat(buf,buf2); strcat(buf,"'");
  lcd.printStr(ALIGN_RIGHT, 1, buf);

  buf[0]=0; strcat(buf,"<"); dtostrf(minHum.humidity,1,0,buf2); strcat(buf,buf2); strcat(buf,"%");
  lcd.printStr(ALIGN_LEFT, 4, buf);
  buf[0]=0; strcat(buf,">"); dtostrf(maxHum.humidity,1,0,buf2); strcat(buf,buf2); strcat(buf,"%");
  lcd.printStr(ALIGN_LEFT, 5, buf);
}


void showMinMax() 
{
  //calcMinMax();
  //lcd.setFont(Small5x7PL);
  lcd.setFont(c64enh);
  lcd.setDigitMinWd(5);
  lcd.printStr(ALIGN_CENTER, 0, "Min/Max");
  buf[0]=0; strcat(buf,"<"); dtostrf(minTemp.temperature,1,1,buf2); strcat(buf,buf2); //strcat(buf,"'");
  lcd.printStr(0, 2, buf);
  buf[0]=0; strcat(buf,">"); dtostrf(maxTemp.temperature,1,1,buf2); strcat(buf,buf2); //strcat(buf,"'");
  lcd.printStr(0, 3, buf);
  buf[0]=0; strcat(buf,"<"); dtostrf(minHum.humidity,1,0,buf2); strcat(buf,buf2); strcat(buf,"%");
  lcd.printStr(0, 4, buf);
  buf[0]=0; strcat(buf,">"); dtostrf(maxHum.humidity,1,0,buf2); strcat(buf,buf2); strcat(buf,"%");
  lcd.printStr(0, 5, buf);
  x=ALIGN_RIGHT;
  lcd.setFont(TinyDig3x7SqPL);
  lcd.setDigitMinWd(3);
  snprintf(buf,25,"%d.%02d.%02d %02d:%02d",minTemp.day,minTemp.month,minTemp.year,minTemp.hour,minTemp.minute);
  lcd.printStr(x, 2, buf);
  snprintf(buf,25,"%d.%02d.%02d %02d:%02d",maxTemp.day,maxTemp.month,maxTemp.year,maxTemp.hour,maxTemp.minute);
  lcd.printStr(x, 3, buf);
  snprintf(buf,25,"%d.%02d.%02d %02d:%02d",minHum.day,minHum.month,minHum.year,minHum.hour,minHum.minute);
  lcd.printStr(x, 4, buf);
  snprintf(buf,25,"%d.%02d.%02d %02d:%02d",maxHum.day,maxHum.month,maxHum.year,maxHum.hour,maxHum.minute);
  lcd.printStr(x, 5, buf);
}

/*
void dumpEEPROM()
{
  lcd.setFont(TinyDig3x7SqPL);
  lcd.setCharMinWd(3);
  lcd.setDigitMinWd(3);
  if(encoderPos>=(128-6)*2) encoderPos=(128-6)*2;
  int st = encoderPos/2;
  for(int j=0;j<numScrLines;j++) {
    int ii = st*8+j*8;
    ii&=0x3ff; // max 1kB
    snprintf(buf,8,"%03X",ii);
    lcd.printStr(0, j, buf);
    for(int i=0;i<8;i++) {
      int v = EEPROM.read(ii+i);
      snprintf(buf,8,"%02X",v);
      lcd.printStr(14+i*9, j, buf);
    }
  }
}

void dumpRTCMem()
{
  lcd.setFont(TinyDig3x7SqPL);
  lcd.setCharMinWd(3);
  lcd.setDigitMinWd(3);
  if(encoderPos>=1*2) encoderPos=1*2;
  int st = encoderPos/2;
  for(int j=0;j<numScrLines;j++) {
    int ii = st*8+j*8;
    snprintf(buf,8,"%03X",ii);
    lcd.printStr(0, j, buf);
    if(ii<56)
    for(int i=0;i<8;i++) {
      int v = readRTCMem(ii+i);
      snprintf(buf,8,"%02X",v);
      lcd.printStr(14+i*9, j, buf);
    }
    else lcd.fillWin(0, j, 84, 1, 0x00);
  }
}

void dumpI2CFlash()
{
  lcd.setFont(TinyDig3x7SqPL);
  lcd.setCharMinWd(3);
  lcd.setDigitMinWd(3);
  if(encoderPos>=85*2) encoderPos=85*2;
  int st = encoderPos/2;
  for(int j=0;j<numScrLines;j++) {
    int ii = st*48+j*8;
    snprintf(buf,8,"%03X",ii);
    lcd.printStr(0, j, buf);
    if(ii<4096)
    for(int i=0;i<8;i++) {
      int v = readByte(ii+i);
      snprintf(buf,8,"%02X",v);
      lcd.printStr(14+i*9, j, buf);
    }
    else lcd.fillWin(0, j, 84, 1, 0x00);
  }
}
*/

/*
 Compacting all data (time,date,temperature,humidity) to 39 bits bitfield:
 h 0..23 5b
 m 0..59 6b  =11b

 d 1..31 5b
 m 1..12 4b
 y 2018-2021 2b =11b

 t 0.0..60.0 =600 values=10b (could be limited to 9b)
 h 0..100  7b
=39b
        bits
        76543210
 byte0  hhhhhmmm
 byte1  mmmddddd
 byte2  mmmmyytt
 byte3  tttttttt
 byte4  hhhhhhh0
*/

byte recBuf[5];

void encodeRecord(struct LogData *data)
{
  int tm = data->temperature*10, y=data->year-18;
  recBuf[0] = (data->hour<<3) | (data->minute>>3);
  recBuf[1] = (data->minute<<5) | data->day;
  recBuf[2] = (data->month<<4) | (y<<2) | (tm>>8);
  recBuf[3] = (tm&0xff);
  recBuf[4] = (data->humidity<<1);
}

void storeRecord(struct LogData *data)
{
  getCurAddr();
  recNum++;
  if(recNum>=recNumMax) recNum=recNumMax;
  encodeRecord(data);
  for(int i=0;i<5;i++) { writeByte(recAddr+i,recBuf[i]); delay(20); }
  
  recAddr+=5;
  if(recAddr>=recNumMax*5) recAddr-=recNumMax*5;
  writeRTCMem(REG_ADDR,recAddr&0xff); writeRTCMem(REG_ADDR+1,recAddr>>8);
  writeRTCMem(REG_NUM,recNum&0xff); writeRTCMem(REG_NUM+1,recNum>>8);
}

void decodeRecord(struct LogData *data)
{
  data->hour     = recBuf[0]>>3;
  data->minute   = ((recBuf[0]&0x7)<<3) | (recBuf[1]>>5);
  data->day      = recBuf[1]&0x1f;
  data->month    = ((recBuf[2]&0xf0)>>4);
  data->year     = 18 + ((recBuf[2]>>2)&0x3);
  data->temperature = (((recBuf[2]&0x3)<<8) | recBuf[3])/10.0;
  data->humidity = (recBuf[4]>>1);
}

void readRecord(int addr, struct LogData *data)
{
  readBytes(addr,recBuf,5); // 2.5x faster
  decodeRecord(data);
}

void readMinMax()
{
  int i;
  for(i=0;i<5;i++) recBuf[i]=readRTCMem(REG_MINT+i);
  decodeRecord(&minTemp);
  for(i=0;i<5;i++) recBuf[i]=readRTCMem(REG_MAXT+i);
  decodeRecord(&maxTemp);
  for(i=0;i<5;i++) recBuf[i]=readRTCMem(REG_MINH+i);
  decodeRecord(&minHum);
  for(i=0;i<5;i++) recBuf[i]=readRTCMem(REG_MAXH+i);
  decodeRecord(&maxHum);
}

void storeMinMax(struct LogData *data, int memaddr)
{
  encodeRecord(data);
  for(int i=0;i<5;i++) writeRTCMem(memaddr+i,recBuf[i]);
}

int handleButton()
{
  lcd.setInvert(0);
  if(readButton()<=0) return 1;
  menuMode=-1;
  lcd.clrScr();
  return 0;
}

void clearLogs()
{
  if(encoderPos>=1*2) encoderPos=1*2;
  int st = encoderPos/2;
  getCurAddr();
  lcd.setFont(Small4x7PL);
  snprintf(buf,20,"recAddr = %d",recAddr);
  lcd.printStr(0, 0, buf);
  snprintf(buf,20,"recNum = %d / %d",recNum,recNumMax);
  lcd.printStr(0, 1, buf);
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 3, "Delete Logs?");
  lcd.setInvert(st?0:1); lcd.printStr(10, 5, " NO ");
  lcd.setInvert(st?1:0); lcd.printStr(43, 5, " YES ");
  if(handleButton()) return;
  encoderPos=oldPos; 
  if(st>0) { // yes
    clearLogAddrNum();
    lcd.printStr(ALIGN_CENTER, 2, "Deleting ..."); delay(500); lcd.clrScr();
    handleMenu();
  }
}

void resetMinMax()
{
  if(encoderPos>=2*2) encoderPos=2*2;
  int st = encoderPos/2;
  getCurAddr();
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 1, "Reset MinMax");
  lcd.setInvert(st==0?1:0); lcd.printStr(0, 4, " NO ");
  lcd.setInvert(st==1?1:0); lcd.printStr(23, 4, " YES ");
  lcd.setInvert(st==2?1:0); lcd.printStr(ALIGN_RIGHT, 4, " CALC");
  if(handleButton()) return;
  encoderPos=oldPos; 
  if(st==1) { // yes
    minTemp=maxTemp=minHum=maxHum=cur;
    storeAllMinMax();
  } else if(st==2) { // calc
    lcd.printStr(ALIGN_CENTER, 2, "Calculating...");
    calcMinMax();
    lcd.clrScr();
  }
  handleMenu();
}


int setMode = -1;
int encoderMin=0,encoderMax=8*2;

void setClock()
{
  if(encoderPos<encoderMin*2) encoderPos=encoderMin*2;
  if(encoderPos>encoderMax*2) encoderPos=encoderMax*2;
  int pos = encoderPos/2, st=pos;
  lcd.setFont(c64enh);
  lcd.setDigitMinWd(7);
  lcd.setCharMinWd(7);
  if(setMode>=0) {
    switch(setMode) {
      case 0: set.hour=st; break;
      case 1: set.minute=st; break;
      case 2: set.second=st; break;
      case 3: set.day=st; break;
      case 4: set.month=st; break;
      case 5: set.year=st; break;
      case 6: set.dayOfWeek=st; break;
    }
    st=setMode;
    lcd.setInvertMask(0x80);
  }
  
  x=10;
  snprintf(buf,10,"%02d",set.hour);
  lcd.setInvert(st==0?1:0); x=lcd.printStr(x, 0, buf);
  lcd.setInvert(0);  x=lcd.printStr(x, 0, ":");
  snprintf(buf,10,"%02d",set.minute);
  lcd.setInvert(st==1?1:0); x=lcd.printStr(x, 0, buf);
  lcd.setInvert(0); x=lcd.printStr(x, 0, ":");
  snprintf(buf,10,"%02d",set.second);
  lcd.setInvert(st==2?1:0); x=lcd.printStr(x, 0, buf);

  x=10;
  snprintf(buf,10,"%02d",set.day);
  lcd.setInvert(st==3?1:0); x=lcd.printStr(x, 2, buf);
  lcd.setInvert(0); x=lcd.printStr(x, 2, ".");
  snprintf(buf,10,"%02d",set.month);
  lcd.setInvert(st==4?1:0); x=lcd.printStr(x, 2, buf);
  lcd.setInvert(0); x=lcd.printStr(x, 2, ".");
  snprintf(buf,10,"%02d",set.year);
  lcd.setInvert(st==5?1:0); x=lcd.printStr(x, 2, buf);

  lcd.setInvert(st==6?1:0); snprintf(buf,10," %s ",dowTxt[set.dayOfWeek]);
  lcd.printStr(ALIGN_CENTER, 3, buf);
  lcd.setCharMinWd(2);
  lcd.setInvert(st==7?1:0); lcd.printStr(ALIGN_LEFT, 5, " CANCEL ");
  lcd.setInvert(st==8?1:0); lcd.printStr(ALIGN_RIGHT, 5, " SET ");
  lcd.setInvert(0);

  if(readButton()<=0) return;

  if(setMode>=0) {
    encoderPos=setMode*2;
    encoderMax=8; encoderMin=0;
    setMode=-1;
    lcd.setInvertMask(0xff);
    return;
  }
  if(pos<=6) {
    setMode=pos;
    encoderMin=0;
    switch(setMode) {
      case 0: encoderMax=23; encoderPos=set.hour; break;
      case 1: encoderMax=59; encoderPos=set.minute; break;
      case 2: encoderMax=59; encoderPos=set.second; break;
      case 3: encoderMax=31; encoderMin=1; encoderPos=set.day; break;
      case 4: encoderMax=12; encoderMin=1; encoderPos=set.month; break;
      case 5: encoderMax=21; encoderMin=18; encoderPos=set.year; break;
      case 6: encoderMax=7;  encoderMin=1; encoderPos=set.dayOfWeek; break;
    }
    encoderPos*=2;
    return;
  }
  
  menuMode=-1;
  encoderPos=oldPos; 
  lcd.clrScr();
  if(pos==8) { // SET
    cur = set;
    setRTCDateTime(&cur);
    lcd.printStr(ALIGN_CENTER, 2, "Setting ..."); delay(500); lcd.clrScr();
    handleMenu();
  }
}

void storeAllMinMax()
{
  storeMinMax(&minTemp,REG_MINT);
  storeMinMax(&maxTemp,REG_MAXT);
  storeMinMax(&minHum,REG_MINH);
  storeMinMax(&maxHum,REG_MAXH);
}

void calcMinMax()
{
  getCurAddr();
  int i,ii,offs = (recNum>=recNumMax) ? recAddr/5 : 0;
  for(i=0;i<recNum;i++) {
    ii = i+offs;
    if(ii>=recNumMax) ii-=recNumMax;
    readRecord(ii*5,&tmp);
    if(i==0) {
      minTemp=maxTemp=minHum=maxHum=tmp;
    } else {
      if(tmp.temperature<minTemp.temperature) minTemp=tmp;
      if(tmp.temperature>maxTemp.temperature) maxTemp=tmp;
      if(tmp.humidity<minHum.humidity) minHum=tmp;
      if(tmp.humidity>maxHum.humidity) maxHum=tmp;
    }
  }
  storeAllMinMax();
}

void dumpLogsSER()
{
  getCurAddr();
  int ii,t1,t10;
  int offs = (recNum>=recNumMax) ? recAddr/5 : 0;
  for(int i=0;i<recNum;i++) {
    ii = i+offs;
    if(ii>=recNumMax) ii-=recNumMax;
    readRecord(ii*5,&tmp);
    t1 = (int)tmp.temperature;
    t10 = tmp.temperature*10-t1*10;
    snprintf(buf,34,"%d,%02d-%02d-%d,%02d:%02d,%d.%d,%d%%",i,tmp.day,tmp.month,tmp.year,tmp.hour,tmp.minute,t1,t10,tmp.humidity);
    Serial.println(buf);
    long v = 84L*i/recNum;
    lcd.fillWin(0,2,v,2,0xff);
    if(v<84) lcd.fillWin(v,2,84-v,2,0);
  }
  Serial.flush();
  menuMode=-1;
  lcd.clrScr();
  encoderPos=oldPos;
  handleMenu();
}

void reviewLogs()
{
  getCurAddr();
  lcd.setFont(TinyDig3x7SqPL);
  lcd.setDigitMinWd(3);
  int numPages = (recNum+4)/5;
  if(encoderPos<-numPages*2) encoderPos=-numPages*2;
  if(encoderPos>numPages*2-1) encoderPos=numPages*2-1;
  int t1,t10,ii,st = encoderPos/2;
  if(st<0) st=numPages+st;
  int offs = (recNum>=recNumMax) ? recAddr/5 : 0;
  
  for(int j=0;j<5;j++) {
    ii = st*5+j+offs;
    if(ii>=recNumMax) ii-=recNumMax;
    readRecord(ii*5,&tmp);
    if(j==0) {
      snprintf(buf,25,"%03d %02d.%02d.%02d",recNum,tmp.day,tmp.month,tmp.year);
      lcd.printStr(0, 0, buf);
    }
    t1 = (int)tmp.temperature;
    t10 = tmp.temperature*10-t1*10;
    if(ii<4095/5 && ii<recNum) {
      snprintf(buf,25,"%03d %02d:%02d %2d.%d' %2d%%",ii,tmp.hour,tmp.minute,t1,t10,tmp.humidity);
      lcd.printStr(0, j+1, buf);
    } else lcd.fillWin(0, j+1, 84-3, 1, 0x00);
  }
  snprintf(buf,25,"  %d",st); lcd.printStr(ALIGN_RIGHT, 0, buf);
  // slider
  int y, n = (8*5-2-4-2)*st/(numPages-1);
  scrWd = 3;
  scrHt = numScrLines-1;
  clrBuf();
  for(y=0; y<5*8; y++) drawPixel(1,y,1);
  for(y=0; y<4; y++) { drawPixel(0,y+n+2,1); drawPixel(2,y+n+2,1); }
  lcd.drawBuf(scr,81,1,scrWd,scrHt);
}

int curT=0;
int graphStart=0;

void drawGraph(int m)
{
  getCurAddr();
//recNum=500;
  if(encoderPos<0) encoderPos=0;
  curT = encoderPos;
  if(curT>=recNum) { curT=recNum-1; encoderPos=curT; }
  if(curT>=graphStart+84) graphStart=curT-84+1;
  if(curT<graphStart) graphStart=curT;
  int x,y,t1,t10;
  LogData c;
  int maxy = 48-16-2;
  int maxx = 84;
  int mint2 = (int)minTemp.temperature;
  int maxt2 = (int)(maxTemp.temperature+0.99999);
  int minh2 = minHum.humidity;
  int maxh2 = maxHum.humidity;
  scrWd=84;
  scrHt=4;
  clrBuf();
  for(x=0; x<maxx; x++) {
    readRecord((x+graphStart)*5,&tmp);
    if(x+graphStart<recNum) {
      if(m)
        y = maxy-1-maxy*(tmp.humidity-minh2)/(maxh2-minh2);
      else
        y = maxy-1-maxy*(tmp.temperature-mint2)/(maxt2-mint2);
      drawLineV(x,y,maxy);
      //drawPixel(x,y,2);
    }
    if(x+graphStart==curT) {
      for(y=0;y<31;y++) drawPixel(x,y,2);
      c = tmp; 
    }
  }
  lcd.drawBuf(scr,0,1,scrWd,scrHt);

  t1 = (int)c.temperature;
  t10 = c.temperature*10-t1*10;
  lcd.setFont(TinyDig3x7SqPL);
  lcd.setDigitMinWd(3);
  snprintf(buf,33,"%03d/%03d  %02d.%d' %02d%%",curT,recNum,t1,t10,c.humidity);
  lcd.printStr(ALIGN_RIGHT,0,buf);
  if(m) snprintf(buf,30,"%d%% ",maxh2); else snprintf(buf,30,"%d' ",maxt2);
  lcd.printStr(ALIGN_LEFT,0,buf);
  if(m) snprintf(buf,30,"%d%% ",minh2); else snprintf(buf,30,"%d' ",mint2);
  lcd.printStr(ALIGN_LEFT,5,buf);
  snprintf(buf,33," %02d.%02d.%02d %02d:%02d",c.day,c.month,c.year,c.hour,c.minute);
  lcd.printStr(ALIGN_RIGHT,5,buf);
}

void showLowBatt()
{
  x=8;
  lcd.clrScr();
  lcd.setFont(Term9x14);
  lcd.printStr(ALIGN_CENTER, 0, "Low");
  lcd.printStr(ALIGN_CENTER, 2, "Battery");
  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);
  dtostrf(v/1000.0,1,3,buf);
  x=lcd.printStr(x, 5, "Vcc: ");
  x=lcd.printStr(x, 5, buf);
  lcd.printStr(x+2, 5, "V");
  detachInterrupt(digitalPinToInterrupt(encoderPinA));
  detachInterrupt(digitalPinToInterrupt(encoderButton));
  powerDown(SLEEP_8S);
  powerDown(SLEEP_8S);
  // disable LCD controller and power down forever to save battery
  lcd.sleep(true);
  powerDown(SLEEP_FOREVER);
}

void setInterval()
{
  if(encoderPos>120) encoderPos=120;
  if(encoderPos<1) encoderPos=1;
  snprintf(buf,20," %d minute%s ",encoderPos,encoderPos>1?"s":"");
  lcd.setFont(Small5x7PL);
  lcd.printStr(ALIGN_CENTER, 1, "Set log interval");
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  lcd.printStr(ALIGN_CENTER, 3, buf);
  int v=84*encoderPos/120;
  lcd.fillWin(0,4,v,1,0xfc);
  if(v<84) lcd.fillWin(v,4,84-v,1,0);
  if(handleButton()) return;
  logInterval = encoderPos;
  writeRTCMem(REG_LOGINT,logInterval);
  if(logTime>logInterval*60L*1000) logTime=logInterval*60L*1000;
  encoderPos = oldPos; 
}

void setBacklight()
{
  if(encoderPos>BACKLIGHT_MAX*2) encoderPos=BACKLIGHT_MAX*2;
  if(encoderPos<BACKLIGHT_MIN*2) encoderPos=BACKLIGHT_MIN*2-1;
  int st=encoderPos/2;
  if(st<BACKLIGHT_MIN) 
    snprintf(buf,20,"     OFF     "); 
  else if(st>=BACKLIGHT_MAX)
    snprintf(buf,20,"  ALWAYS ON  "); 
  else
    snprintf(buf,20," %d seconds ",st*10);
  lcd.setFont(Small5x7PL);
  lcd.printStr(ALIGN_CENTER, 1, "Set backlight");
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  lcd.printStr(ALIGN_CENTER, 3, buf);
  int v=84*(st-BACKLIGHT_MIN+1)/BACKLIGHT_MAX;
  lcd.fillWin(0,4,v,1,0xfc);
  if(v<84) lcd.fillWin(v,4,84-v,1,0);
  if(handleButton()) return;
  backlight = st;
  writeRTCMem(REG_LIGHT,backlight);
  encoderPos = oldPos; 
}

void setMenu(int m)
{
  menuMode=m;
  lcd.clrScr();
  oldPos=encoderPos;
  encoderPos=0;
}

void endMenu()
{
  if(readButton()<=0) return;
  menuMode=-1;
  lcd.clrScr();
  encoderPos=oldPos; 
}

void formatMenu(char *in, char *out, int num)
{
  int j=strlen(in);
  out[0]=' ';
  strncpy(out+1,in,j++);
  for(;j<num;j++) out[j]=' ';
  out[j]=0;
}

void drawMenuSlider()
{
  int y, n = (8*numScrLines-2-5-2)*menuLine/(numMenus-1);
  scrWd = 3;
  scrHt = numScrLines;
  clrBuf();
  for(y=0; y<numScrLines*8; y++) drawPixel(1,y,1);
  for(y=0; y<5; y++) { drawPixel(0,y+n+2,1); drawPixel(2,y+n+2,1); }
  lcd.drawBuf(scr,81,0,scrWd,scrHt);
}

void handleMenu()
{
  lcd.setFont(Small5x7PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  if(menuMode!=5) { if(encoderPos<0) encoderPos=0; } // reviewLogs
  if(menuMode==-1) {
    menuLine = encoderPos/2;
    if(menuLine>=numMenus) { menuLine=numMenus-1; encoderPos=menuLine*2; }
    if(menuLine>=menuStart+numScrLines) menuStart=menuLine-numScrLines+1;
    if(menuLine<menuStart) menuStart=menuLine;
    for(int i=0;i<numScrLines;i++) {
      if(i+menuStart<numMenus) {
        lcd.setInvert(i+menuStart==menuLine ? 1 : 0);
        formatMenu(menuTxt[i+menuStart], buf, 13);
        lcd.printStr(ALIGN_LEFT, i, buf);
      }
    }
    drawMenuSlider();
    if(readButton()) {
      setMenu(menuLine);
      if(menuLine==13) encoderPos=logInterval; // setInterval
      if(menuLine==14) encoderPos=backlight*2; // setBacklight
      if(menuLine==12) { set=cur; encoderMin=0; encoderMax=8; } // setClock
    }
  } else
  if(menuMode==0) { showClock(1);  endMenu(); } else
  if(menuMode==1) { showClock(0);  endMenu(); } else
  if(menuMode==2) { showTempOrHum(0); endMenu(); } else
  if(menuMode==3) { showTempOrHum(1); endMenu(); } else
  if(menuMode==4) { showBothTempHum(); endMenu(); } else
  if(menuMode==5) { reviewLogs(); endMenu(); } else
  if(menuMode==6) { dumpLogsSER(); } else
  if(menuMode==7) { showMinMax(); endMenu(); } else
  if(menuMode==8) { resetMinMax(); endMenu(); } else
  if(menuMode==9) { drawGraph(0); endMenu(); } else
  if(menuMode==10) { drawGraph(1); endMenu(); } else
  if(menuMode==11) { clearLogs(); endMenu();} else
  if(menuMode==12) { setClock(); endMenu(); } else
  if(menuMode==13) { setInterval(); endMenu(); } else
  if(menuMode==14) { setBacklight(); endMenu(); } else
  //if(menuMode==6) { dumpEEPROM(); endMenu(); } else
  //if(menuMode==7) { dumpI2CFlash(); endMenu(); } else
  //if(menuMode==8) { dumpRTCMem(); endMenu(); } else
  { endMenu(); }
}

unsigned long loopTime = 0;

void loop() 
{
  loopTime = millis();
  v=readVcc();
  if(v<2900) showLowBatt();

  getRTCDateTime(&cur);

  int ret = readDHT11(DHT11_PIN);   // only positive values - room temperatures

  if(ret==DHT_OK) {
    if(cur.temperature<minTemp.temperature) { minTemp=cur; storeMinMax(&minTemp,REG_MINT); }
    if(cur.temperature>maxTemp.temperature) { maxTemp=cur; storeMinMax(&maxTemp,REG_MAXT); }
    if(cur.humidity<minHum.humidity) { minHum=cur; storeMinMax(&minHum,REG_MINH); }
    if(cur.humidity>maxHum.humidity) { maxHum=cur; storeMinMax(&maxHum,REG_MAXH); }
    if(first) {
      //mint=maxt=cur.temperature;
      //minh=maxh=cur.humidity;
      //minD=maxD=cur;
      //minM=maxM=cur;
      //minY=maxY=cur;
      //minT=maxT=cur;
      first=0;
      lcd.clrScr();
    }
  }
  if(first && ret!=DHT_OK) {
    lcd.clrScr();
    lcd.setFont(Term9x14);
    lcd.printStr(ALIGN_CENTER, 1, "Sensor");
    lcd.printStr(ALIGN_CENTER, 3, "init");
    powerDown(SLEEP_2S);
    return;
  }

  handleMenu();

  loopTime=millis()-loopTime;
  powerDown(menuMode==0 ? SLEEP_1S : SLEEP_8S);
  loopTime+=(menuMode==0 ? REAL_SLEEP_8S/8 : REAL_SLEEP_8S);
/*
  // testing real SLEEP_8S delay
  getRTCDateTime(&cur);
  Serial.print("p");Serial.println(cur.second);  Serial.flush();
  powerDown(SLEEP_8S);
  powerDown(SLEEP_8S);
  powerDown(SLEEP_8S);
  powerDown(SLEEP_8S);
  powerDown(SLEEP_8S);
  getRTCDateTime(&cur);
  Serial.println(cur.second); Serial.flush();
*/
  logTime-=loopTime;
  if(logTime<=0) {
    logTime=logInterval*60L*1000;
    storeRecord(&cur);
  }
  lightTime-=loopTime;
  if((lightTime<=0 || backlight<BACKLIGHT_MIN) && backlight<BACKLIGHT_MAX) digitalWrite(BACKLIGHT,1); // off
  menuTime-=loopTime;
  if(menuTime<=0 && menuMode<0) setMenu(1); // show clock
}

