#define AT24Cxx_I2C_ADDR 0x50

bool isFlashPresent(void)
{
  Wire.beginTransmission(AT24Cxx_I2C_ADDR);
  if(Wire.endTransmission() == 0) return true;
  return false;
} 

int readBytes(int addr, uint8_t *buf, int cnt)
{
  int i=0, bytes;
  while(cnt>0) {
    Wire.beginTransmission(AT24Cxx_I2C_ADDR);
    Wire.write(addr>>8);   // Address MSB
    Wire.write(addr&0xff); // Address LSB
    Wire.endTransmission();

    bytes = min(cnt, 128);
    Wire.requestFrom(AT24Cxx_I2C_ADDR, bytes);

    while (Wire.available() && cnt>0) {
      buf[i] = Wire.read();
      i++; cnt--; addr++;
    }
  }
  return i;
}

int readByte(int addr)
{
  Wire.beginTransmission(AT24Cxx_I2C_ADDR);
  Wire.write(addr>>8);   // Address MSB
  Wire.write(addr&0xff); // Address LSB
  Wire.endTransmission();
  Wire.requestFrom(AT24Cxx_I2C_ADDR, 1);
  while(!Wire.available()) {};
  return Wire.read();
}

uint8_t writeByte(int addr, uint8_t val)
{
  Wire.beginTransmission(AT24Cxx_I2C_ADDR);
  Wire.write(addr>>8);   // Address MSB
  Wire.write(addr&0xff); // Address LSB
  Wire.write(val);
  return Wire.endTransmission();
}

