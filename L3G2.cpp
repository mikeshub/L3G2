#include <L3G2.h>
//#include <Wire.h>
#include <I2c.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define L3G24200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G24200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3G2D20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3G2D20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// Public Methods //////////////////////////////////////////////////////////////

bool L3G2::init(byte device, byte sa0)
{
  _device = device;
  switch (_device)
  {
    case L3G24200D_DEVICE:
      if (sa0 == L3G2_SA0_LOW)
      {
        address = L3G24200D_ADDRESS_SA0_LOW;
        return true;
      }
      else if (sa0 == L3G2_SA0_HIGH)
      {
        address = L3G24200D_ADDRESS_SA0_HIGH;
        return true;
      }
      else
        return autoDetectAddress();
      break;

    case L3G2D20_DEVICE:
      if (sa0 == L3G2_SA0_LOW)
      {
        address = L3G2D20_ADDRESS_SA0_LOW;
        return true;
      }
      else if (sa0 == L3G2_SA0_HIGH)
      {
        address = L3G2D20_ADDRESS_SA0_HIGH;
        return true;
      }
      else
        return autoDetectAddress();
      break;

    default:
      return autoDetectAddress();
  }
}

// Turns on the L3G2's gyro and places it in normal mode.
void L3G2::enableDefault(void)
{
  // 0x0F = 0b00001111
  // Normal power mode, all axes enabled
  writeReg(L3G2_CTRL_REG1, 0x0F);
}

// Writes a gyro register
void L3G2::writeReg(int reg, int value)
{
  /*Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();*/
  I2c.write((int)address,reg,value);
}

// Reads a gyro register
byte L3G2::readReg(int reg)
{
  byte value;

  /*Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();*/
  I2c.read((int)address,reg,1);
  value = I2c.receive();

  return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G2::read()
{
  /*Wire.beginTransmission(address);
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  Wire.write(L3G2_OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)6);

  while (Wire.available() < 6);

  uint8_t xla = Wire.read();
  uint8_t xha = Wire.read();
  uint8_t yla = Wire.read();
  uint8_t yha = Wire.read();
  uint8_t zla = Wire.read();
  uint8_t zha = Wire.read();*/
  I2c.read(address,L3G2_OUT_X_L | (1 << 7),6,inBuffer);

  /*g.x = xha << 8 | xla;
  g.y = yha << 8 | yla;
  g.z = zha << 8 | zla;*/

  g.x = inBuffer[1] << 8 | inBuffer[0];
  g.y = inBuffer[3] << 8 | inBuffer[2];
  g.z = inBuffer[5] << 8 | inBuffer[4];
}

void L3G2::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G2::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G2::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

bool L3G2::autoDetectAddress(void)
{
  // try each possible address and stop if reading WHO_AM_I returns the expected response
  address = L3G24200D_ADDRESS_SA0_LOW;
  if (readReg(L3G2_WHO_AM_I) == 0xD3) return true;
  address = L3G24200D_ADDRESS_SA0_HIGH;
  if (readReg(L3G2_WHO_AM_I) == 0xD3) return true;
  address = L3G2D20_ADDRESS_SA0_LOW;
  if (readReg(L3G2_WHO_AM_I) == 0xD4) return true;
  address = L3G2D20_ADDRESS_SA0_HIGH;
  if (readReg(L3G2_WHO_AM_I) == 0xD4) return true;

  return false;
}
