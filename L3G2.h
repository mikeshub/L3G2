#ifndef L3G2_h
#define L3G2_h

#include <Arduino.h> // for byte data type

// device types

#define L3G2_DEVICE_AUTO 0
#define L3G24200D_DEVICE 1
#define L3G2D20_DEVICE   2


// SA0 states

#define L3G2_SA0_LOW  0
#define L3G2_SA0_HIGH 1
#define L3G2_SA0_AUTO 2

// register addresses

#define L3G2_WHO_AM_I      0x0F

#define L3G2_CTRL_REG1     0x20
#define L3G2_CTRL_REG2     0x21
#define L3G2_CTRL_REG3     0x22
#define L3G2_CTRL_REG4     0x23
#define L3G2_CTRL_REG5     0x24
#define L3G2_REFERENCE     0x25
#define L3G2_OUT_TEMP      0x26
#define L3G2_STATUS_REG    0x27

#define L3G2_OUT_X_L       0x28
#define L3G2_OUT_X_H       0x29
#define L3G2_OUT_Y_L       0x2A
#define L3G2_OUT_Y_H       0x2B
#define L3G2_OUT_Z_L       0x2C
#define L3G2_OUT_Z_H       0x2D

#define L3G2_FIFO_CTRL_REG 0x2E
#define L3G2_FIFO_SRC_REG  0x2F

#define L3G2_INT1_CFG      0x30
#define L3G2_INT1_SRC      0x31
#define L3G2_INT1_THS_XH   0x32
#define L3G2_INT1_THS_XL   0x33
#define L3G2_INT1_THS_YH   0x34
#define L3G2_INT1_THS_YL   0x35
#define L3G2_INT1_THS_ZH   0x36
#define L3G2_INT1_THS_ZL   0x37
#define L3G2_INT1_DURATION 0x38

class L3G2
{
  public:
    typedef struct vector
    {
      float x, y, z;
    } vector;

    vector g; // gyro angular velocity readings

	uint8_t inBuffer[6];

    bool init(byte device = L3G2_DEVICE_AUTO, byte sa0 = L3G2_SA0_AUTO);

    void enableDefault(void);

    void writeReg(int reg, int value);
    byte readReg(int reg);
    void read(void);

    // vector functions
    static void vector_cross(const vector *a, const vector *b, vector *out);
    static float vector_dot(const vector *a,const vector *b);
    static void vector_normalize(vector *a);

  private:
      byte _device; // chip type (4200D or D20)
      byte address;

      bool autoDetectAddress(void);
};

#endif



