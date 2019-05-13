#include <i2c_t3.h>
#include <Arduino.h>
#include <BMP180MI.h>


#define I2C_ADDRESS 0x77


BMP180I2C bmp180(I2C_ADDRESS);

//--Acceleration parameter--//
#define ACCELERATION_X_DANGER 1800
#define ACCELERATION_Y_DANGER 1670
#define ACCELERATION_Z_DANGER 1990

boolean ACC_X_FLAG;
boolean ACC_Y_FLAG;
boolean ACC_Z_FLAG;

boolean ACC;

//--Gyroscope parameter--//
#define GYROSCOPE_X_DANGER 244
#define GYROSCOPE_Y_DANGER 244
#define GYROSCOPE_Z_DANGER 244

boolean GYRO_X_FLAG;
boolean GYRO_Y_FLAG;
boolean GYRO_Z_FLAG;

boolean GYRO;

//--------------------------------
uint16_t pressure;
boolean PRESS;

#define PRESSURE_DANGER 28800

#define LSM9DS1_AccelGyro 0x6B
#define LSM9DS1_Magnet    0x1E

#define LSM9DS1_COMMUNICATION_TIMEOUT 1000

// LSM9DS1 Accel/Gyro (XL/G) Reg isters
#define WHO_AM_I_XG       0x0F
#define CTRL_REG1_G       0x10
#define CTRL_REG2_G       0x11
#define CTRL_REG3_G       0x12
#define ORIENT_CFG_G      0x13
#define OUT_X_L_G         0x18
#define OUT_X_H_G         0x19
#define OUT_Y_L_G         0x1A
#define OUT_Y_H_G         0x1B
#define OUT_Z_L_G         0x1C
#define OUT_Z_H_G         0x1D
#define CTRL_REG4         0x1E
#define CTRL_REG5_XL      0x1F
#define CTRL_REG6_XL      0x20
#define CTRL_REG7_XL      0x21
#define OUT_X_L_XL        0x28
#define OUT_X_H_XL        0x29
#define OUT_Y_L_XL        0x2A
#define OUT_Y_H_XL        0x2B
#define OUT_Z_L_XL        0x2C
#define OUT_Z_H_XL        0x2D

// LSM9DS1 Magneto Registers
#define WHO_AM_I_M        0x0F
#define CTRL_REG1_M       0x20
#define CTRL_REG2_M       0x21
#define CTRL_REG3_M       0x22
#define CTRL_REG4_M       0x23
#define CTRL_REG5_M       0x24
#define OUT_X_L_M         0x28
#define OUT_X_H_M         0x29
#define OUT_Y_L_M         0x2A
#define OUT_Y_H_M         0x2B
#define OUT_Z_L_M         0x2C
#define OUT_Z_H_M         0x2D

// LSM9DS1 WHO_AM_I Responses
#define WHO_AM_I_AG_RSP   0x68
#define WHO_AM_I_M_RSP    0x3D

//-----------------------------------------------

enum lsm9ds1_axis {
  X_AXIS,
  Y_AXIS,
  Z_AXIS,
  ALL_AXIS
};

//-----------------------------------------------

struct gyroSettings
{
  // Gyroscope settings:
  uint8_t enabled;
  uint16_t scale;
  uint8_t sampleRate;
  
  // New gyro stuff:
  uint8_t bandwidth;
  uint8_t lowPowerEnable;
  uint8_t HPFEnable;
  uint8_t HPFCutoff;
  uint8_t flipX;
  uint8_t flipY;
  uint8_t flipZ;
  uint8_t orientation;
  uint8_t enableX;
  uint8_t enableY;
  uint8_t enableZ;
  uint8_t latchInterrupt;
};

struct accelSettings
{
  // Accelerometer settings:
  uint8_t enabled;
  uint8_t scale;
  uint8_t sampleRate;
  
  // New accel stuff:
  uint8_t enableX;
  uint8_t enableY;
  uint8_t enableZ;
  int8_t  bandwidth;
  uint8_t highResEnable;
  uint8_t highResBandwidth;
};

struct magSettings
{
  // Magnetometer settings:
  uint8_t enabled;
  uint8_t scale;
  uint8_t sampleRate;
  
  // New mag stuff:
  uint8_t tempCompensationEnable;
  uint8_t XYPerformance;
  uint8_t ZPerformance;
  uint8_t lowPowerEnable;
  uint8_t operatingMode;
};

struct temperatureSettings
{
  // Temperature settings
  uint8_t enabled;
};

struct IMUSettings
{
  gyroSettings gyro;
  accelSettings accel;
  magSettings mag;
  
  temperatureSettings temp;
};

//-----------------------------------------------

IMUSettings settings;

uint8_t _i2cAddress_Magnet;
uint8_t _i2cAddress_AccelGyro;

float gRes;
float aRes;
float mRes;

float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};

int16_t gx, gy, gz; 
int16_t ax, ay, az; 
int16_t mx, my, mz;
int16_t temperature;

float gBias[3], aBias[3], mBias[3];

int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];
//-----------------------------------------------


uint16_t begin()
{
  _i2cAddress_AccelGyro = LSM9DS1_AccelGyro;
  _i2cAddress_Magnet = LSM9DS1_Magnet;
  
  constrainScales();

  calcgRes(); // Calculate DPS
  calcmRes(); // Calculate Gs
  calcaRes(); // Calculate g
  
  // Now, initialize our hardware interface.
  Wire.begin();  // Initialize I2C
    
  // To verify communication, we can read from the WHO_AM_I register of
  // each device. Store those in a variable so we can return them.
  uint8_t mTest = I2CreadByte(_i2cAddress_Magnet,WHO_AM_I_M);    // Read the gyro WHO_AM_I
  uint8_t xgTest = I2CreadByte(_i2cAddress_AccelGyro,WHO_AM_I_XG); // Read the accel/mag WHO_AM_I
  uint16_t whoAmICombined = (xgTest << 8) | mTest;
  
  if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP)) return 0;
  
  // Gyro initialization
  initGyro();
  
  // Accelerometer initialization
  initAccel();
  
  // Magnetometer initialization
  initMag();
  
  // Once everything is initialized, return the WHO_AM_I registers we read:
  return whoAmICombined;
}

void constrainScales()
{
  if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) && 
    (settings.gyro.scale != 2000))
  {
    settings.gyro.scale = 245;
  }
    
  if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
    (settings.accel.scale != 8) && (settings.accel.scale != 16))
  {
    settings.accel.scale = 2;
  }
    
  if ((settings.mag.scale != 4) && (settings.mag.scale != 8) &&
    (settings.mag.scale != 12) && (settings.mag.scale != 16))
  {
    settings.mag.scale = 4;
  }
}

void init()
{
  settings.gyro.enabled = true;
  settings.gyro.enableX = true;
  settings.gyro.enableY = true;
  settings.gyro.enableZ = true;
  
  // gyro scale can be 245, 500, or 2000
  settings.gyro.scale = 245;
  // gyro sample rate: value between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  settings.gyro.sampleRate = 6;
  
  // gyro cutoff frequency: value between 0-3
  // Actual value of cutoff frequency depends
  // on sample rate.
  settings.gyro.bandwidth = 0;
  settings.gyro.lowPowerEnable = false;
  settings.gyro.HPFEnable = false;
  
  // Gyro HPF cutoff frequency: value between 0-9
  // Actual value depends on sample rate. Only applies
  // if gyroHPFEnable is true.
  settings.gyro.HPFCutoff = 0;
  settings.gyro.flipX = false;
  settings.gyro.flipY = false;
  settings.gyro.flipZ = false;
  settings.gyro.orientation = 0;
  settings.gyro.latchInterrupt = true;

  //---------------------------------------------------------------

  settings.accel.enabled = true;
  settings.accel.enableX = true;
  settings.accel.enableY = true;
  settings.accel.enableZ = true;
  
  // accel scale can be 2, 4, 8, or 16
  settings.accel.scale = 2;
  
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  settings.accel.sampleRate = 6;
  
  // Accel cutoff freqeuncy can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  settings.accel.bandwidth = -1;
  settings.accel.highResEnable = false;
  
  // accelHighResBandwidth can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  settings.accel.highResBandwidth = 0;
//---------------------------------------------------------------------------
  settings.mag.enabled = true;
  
  // mag scale can be 4, 8, 12, or 16
  settings.mag.scale = 4;
  
  // mag data rate can be 0-7
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  settings.mag.sampleRate = 7;
  settings.mag.tempCompensationEnable = false;
  
  // magPerformance can be any value between 0-3
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  settings.mag.XYPerformance = 3;
  settings.mag.ZPerformance = 3;
  settings.mag.lowPowerEnable = false;
  
  // magOperatingMode can be 0-2
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  settings.mag.operatingMode = 0;

  settings.temp.enabled = true;
  for (int i=0; i<3; i++)
  {
    gBias[i] = 0;
    aBias[i] = 0;
    mBias[i] = 0;
    gBiasRaw[i] = 0;
    aBiasRaw[i] = 0;
    mBiasRaw[i] = 0;
  }
  
}

//-----------------------
// LSM9DS1 Accelerator
//-----------------------

void initAccel()
{
  uint8_t tempRegValue = 0;
  
  //  CTRL_REG5_XL (0x1F) (Default value: 0x38)
  //  [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
  //  DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
  //    00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
  //  Zen_XL - Z-axis output enabled
  //  Yen_XL - Y-axis output enabled
  //  Xen_XL - X-axis output enabled
  if (settings.accel.enableZ) tempRegValue |= (1<<5);
  if (settings.accel.enableY) tempRegValue |= (1<<4);
  if (settings.accel.enableX) tempRegValue |= (1<<3);
  
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG5_XL, tempRegValue);
  
  // CTRL_REG6_XL (0x20) (Default value: 0x00)
  // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
  // ODR_XL[2:0] - Output data rate & power mode selection
  // FS_XL[1:0] - Full-scale selection
  // BW_SCAL_ODR - Bandwidth selection
  // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
  tempRegValue = 0;
  
  // To disable the accel, set the sampleRate bits to 0.
  if (settings.accel.enabled)
  {
    tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
  }
  switch (settings.accel.scale)
  {
    case 4:
      tempRegValue |= (0x2 << 3);
      break;
    case 8:
      tempRegValue |= (0x3 << 3);
      break;
    case 16:
      tempRegValue |= (0x1 << 3);
      break;
    // Otherwise it'll be set to 2g (0x0 << 3)
  }
  
  if (settings.accel.bandwidth >= 0)
  {
    tempRegValue |= (1<<2); // Set BW_SCAL_ODR
    tempRegValue |= (settings.accel.bandwidth & 0x03);
  }
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG6_XL, tempRegValue);
  
  // CTRL_REG7_XL (0x21) (Default value: 0x00)
  // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
  // HR - High resolution mode (0: disable, 1: enable)
  // DCF[1:0] - Digital filter cutoff frequency
  // FDS - Filtered data selection
  // HPIS1 - HPF enabled for interrupt function
  tempRegValue = 0;
  if (settings.accel.highResEnable)
  {
    tempRegValue |= (1<<7); // Set HR bit
    tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
  }
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG7_XL, tempRegValue);
}

void calcaRes()
{
  aRes = ((float) settings.accel.scale) / 32768.0;
}

void readAccel()
{
  uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp 
  
  I2CreadBytes(_i2cAddress_AccelGyro,OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
  
  ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
  ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
  az = (temp[5] << 8) | temp[4]; // Store z-axis values into az

}

float calcAccel(int16_t accel)
{
  // Return the accel raw reading times our pre-calculated g's / (ADC tick):
  return aRes * accel;
}

//-----------------------
// LSM9DS1 Gyroscope
//-----------------------

void initGyro()
{
  uint8_t tempRegValue = 0;
  
  if (settings.gyro.enabled)
  {
    tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
  }
  
  switch (settings.gyro.scale)
  {
    case 500:
      tempRegValue |= (0x1 << 3);
      break;
    case 2000:
      tempRegValue |= (0x3 << 3);
      break;
    // Otherwise we'll set it to 245 dps (0x0 << 4)
  }
  
  tempRegValue |= (settings.gyro.bandwidth & 0x3);
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG1_G, tempRegValue);
  
  // CTRL_REG2_G (Default value: 0x00)
  // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
  // INT_SEL[1:0] - INT selection configuration
  // OUT_SEL[1:0] - Out selection configuration
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG2_G, 0x00); 
  
  // CTRL_REG3_G (Default value: 0x00)
  // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
  // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
  // HP_EN - HPF enable (0:disabled, 1: enabled)
  // HPCF_G[3:0] - HPF cutoff frequency
  tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
  
  if (settings.gyro.HPFEnable)
  {
    tempRegValue |= (1<<6) | (settings.gyro.HPFCutoff & 0x0F);
  }
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG3_G, tempRegValue);
  
  // CTRL_REG4 (Default value: 0x38)
  // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
  // Zen_G - Z-axis output enable (0:disable, 1:enable)
  // Yen_G - Y-axis output enable (0:disable, 1:enable)
  // Xen_G - X-axis output enable (0:disable, 1:enable)
  // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
  // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
  tempRegValue = 0;
  
  if (settings.gyro.enableZ) tempRegValue |= (1<<5);
  if (settings.gyro.enableY) tempRegValue |= (1<<4);
  if (settings.gyro.enableX) tempRegValue |= (1<<3);
  if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
  I2CwriteByte(_i2cAddress_AccelGyro,CTRL_REG4, tempRegValue);
  
  // ORIENT_CFG_G (Default value: 0x00)
  // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
  // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
  // Orient [2:0] - Directional user orientation selection
  tempRegValue = 0;
  
  if (settings.gyro.flipX) tempRegValue |= (1<<5);
  if (settings.gyro.flipY) tempRegValue |= (1<<4);
  if (settings.gyro.flipZ) tempRegValue |= (1<<3);
  I2CwriteByte(_i2cAddress_AccelGyro,ORIENT_CFG_G, tempRegValue);
}

void calcgRes()
{
  gRes = ((float) settings.gyro.scale) / 32768.0;
}

void readGyro()
{
  uint8_t temp[6]; // We'll read six bytes from the gyro into temp
  
  I2CreadBytes(_i2cAddress_AccelGyro,OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
  
  gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
  gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
  gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
  
}

float calcGyro(int16_t gyro)
{
  // Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
  return gRes * gyro; 
}

//-----------------------
// LSM9DS1 Magnet
//-----------------------

void initMag()
{
  uint8_t tempRegValue = 0;
  
  // CTRL_REG1_M (Default value: 0x10)
  // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
  // TEMP_COMP - Temperature compensation
  // OM[1:0] - X & Y axes op mode selection
  //  00:low-power, 01:medium performance
  //  10: high performance, 11:ultra-high performance
  // DO[2:0] - Output data rate selection
  // ST - Self-test enable
  if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
  tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
  tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
  I2CwriteByte(_i2cAddress_Magnet,CTRL_REG1_M, tempRegValue);
  
  // CTRL_REG2_M (Default value 0x00)
  // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
  // FS[1:0] - Full-scale configuration
  // REBOOT - Reboot memory content (0:normal, 1:reboot)
  // SOFT_RST - Reset config and user registers (0:default, 1:reset)
  tempRegValue = 0;

  switch (settings.mag.scale)
  {
      case 8:
        tempRegValue |= (0x1 << 5);
        break;
      case 12:
        tempRegValue |= (0x2 << 5);
        break;
      case 16:
        tempRegValue |= (0x3 << 5);
        break;
      // Otherwise we'll default to 4 gauss (00)
  }
  I2CwriteByte(_i2cAddress_Magnet,CTRL_REG2_M, tempRegValue); // +/-4Gauss
  
  // CTRL_REG3_M (Default value: 0x03)
  // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
  // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
  // LP - Low-power mode cofiguration (1:enable)
  // SIM - SPI mode selection (0:write-only, 1:read/write enable)
  // MD[1:0] - Operating mode
  //  00:continuous conversion, 01:single-conversion,
  //  10,11: Power-down
  tempRegValue = 0;
  if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
  tempRegValue |= (settings.mag.operatingMode & 0x3);
  I2CwriteByte(_i2cAddress_Magnet,CTRL_REG3_M, tempRegValue); // Continuous conversion mode
  
  // CTRL_REG4_M (Default value: 0x00)
  // [0][0][0][0][OMZ1][OMZ0][BLE][0]
  // OMZ[1:0] - Z-axis operative mode selection
  //  00:low-power mode, 01:medium performance
  //  10:high performance, 10:ultra-high performance
  // BLE - Big/little endian data
  tempRegValue = 0;
  tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
  I2CwriteByte(_i2cAddress_Magnet,CTRL_REG4_M, tempRegValue);
  
  // CTRL_REG5_M (Default value: 0x00)
  // [0][BDU][0][0][0][0][0][0]
  // BDU - Block data update for magnetic data
  //  0:continuous, 1:not updated until MSB/LSB are read
  tempRegValue = 0;
  I2CwriteByte(_i2cAddress_Magnet,CTRL_REG5_M, tempRegValue);
}

void calcmRes()
{
  //mRes = ((float) settings.mag.scale) / 32768.0;
  switch (settings.mag.scale)
  {
      case 4:
        mRes = magSensitivity[0];
        break;
      case 8:
        mRes = magSensitivity[1];
        break;
      case 12:
        mRes = magSensitivity[2];
        break;
      case 16:
        mRes = magSensitivity[3];
        break;
  }
}

void readMag()
{
  uint8_t temp[6]; 

  // Read 6 bytes, beginning at OUT_X_L_M
  I2CreadBytes(_i2cAddress_Magnet,OUT_X_L_M, temp, 6); 
  
  mx = (temp[1] << 8) | temp[0];
  my = (temp[3] << 8) | temp[2];
  mz = (temp[5] << 8) | temp[4];
}

float calcMag(int16_t mag)
{
  // Return the mag raw reading times our pre-calculated Gs / (ADC tick):
  return mRes * mag;
}

//-----------------------
// LSM9DS1 i2C communication
//-----------------------

uint8_t I2CreadByte(uint8_t i2cAddress, uint8_t registerAddress)
{
  int timeout = LSM9DS1_COMMUNICATION_TIMEOUT;
  uint8_t data; // `data` will store the register data  

  // Initialize the Tx buffer
  Wire.beginTransmission(i2cAddress);

  // Put slave register address in Tx buffer
  Wire.write(registerAddress);                

  //If false, endTransmission() sends a restart message after transmission. The bus will not be released, 
  //which prevents another master device from transmitting between messages. This allows one master device 
  //to send multiple transmissions while in control. The default value is true.
  Wire.endTransmission(false);            

  // Read one byte from slave register address 
  Wire.requestFrom(i2cAddress, (uint8_t) 1);  
  
  while ((Wire.available() < 1) && (timeout-- > 0))
    delay(1);
  
  if (timeout <= 0) return -1;

  // Fill Rx buffer with result
  data = Wire.read();                      

  // Return data read from slave register
  return data;                             
}

void I2CwriteByte(uint8_t i2cAddress, uint8_t registerAddress, uint8_t data)
{
  // Initialize the Tx buffer
  Wire.beginTransmission(i2cAddress);

  // Put slave register address in Tx buffer
  Wire.write(registerAddress);          

  // Put data in Tx buffer
  Wire.write(data);                 

  // Send the Tx buffer
  Wire.endTransmission();           
}

uint8_t I2CreadBytes(uint8_t i2cAddress, uint8_t registerAddress, uint8_t * dest, uint8_t count)
{  
  int timeout = LSM9DS1_COMMUNICATION_TIMEOUT;

  // Initialize the Tx buffer
  Wire.beginTransmission(i2cAddress);
     
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  // Put slave register address in Tx buffer
  Wire.write(registerAddress | 0x80);     

  //If false, endTransmission() sends a restart message after transmission. The bus will not be released, 
  //which prevents another master device from transmitting between messages. This allows one master device 
  //to send multiple transmissions while in control. The default value is true.
  Wire.endTransmission(false);             

  // Read bytes from slave register address
  Wire.requestFrom(i2cAddress, count);   
  
  while ((Wire.available() < count) && (timeout-- > 0))
    delay(1);
    
  if (timeout <= 0) return -1;
  
  for (int i=0; i<count;)
  {
    if (Wire.available())
    {
      dest[i++] = Wire.read();
    }
  }
  return count;
}

//-----------------------------------------------
int acc_x, acc_y, acc_z;
int gyro_x, gyro_y, gyro_z;
void setup() 
{
  Serial.begin(115200);
  delay(100);
  Wire1.begin(); 
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
  init();
  if (!begin())
  {
    Serial.println("Failed to communicate with LSM9DS1....");
    while (1)
      ;
  }
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
}

bool accel_values(void)
{
  readAccel();
  
  acc_x = abs(1000*calcAccel(ax));
  acc_y = abs(1000*(0.0400-abs(calcAccel(ay))));
  acc_z = abs(1000*(abs(calcAccel(az))));
  (acc_x > ACCELERATION_X_DANGER)?ACC_X_FLAG=HIGH:ACC_X_FLAG=LOW;
  (acc_y > ACCELERATION_Y_DANGER)?ACC_Y_FLAG=HIGH:ACC_Y_FLAG=LOW;
  (acc_z > ACCELERATION_Z_DANGER)?ACC_Z_FLAG=HIGH:ACC_Z_FLAG=LOW;
  return (ACC_X_FLAG & ACC_Y_FLAG & ACC_Z_FLAG)?HIGH:LOW;
}

bool gyro_values(void)
{
  readGyro();
  gyro_x = abs(calcGyro(gx));
  gyro_y = abs(calcGyro(gy));
  gyro_z = abs(calcGyro(gz));   
  (gyro_x >= GYROSCOPE_X_DANGER)?GYRO_X_FLAG=HIGH:GYRO_X_FLAG=LOW;
  (gyro_y >= GYROSCOPE_Y_DANGER)?GYRO_Y_FLAG=HIGH:GYRO_Y_FLAG=LOW;
  (gyro_z >= GYROSCOPE_Z_DANGER)?GYRO_Z_FLAG=HIGH:GYRO_Z_FLAG=LOW;
  return (GYRO_X_FLAG & GYRO_Y_FLAG & GYRO_Z_FLAG)?HIGH:LOW;
}

 bool pressure_values(void)
{
  int target;
  target = bmp180.measurePressure();
  while (!bmp180.hasValue());
  pressure = bmp180.getPressure();
  return (pressure >= PRESSURE_DANGER)?HIGH:LOW;
}
uint16_t interval = 10000;
unsigned long previousMillis;

void loop()
{
  ACC = accel_values();
  GYRO = gyro_values();
  PRESS = pressure_values();
  Serial.println(pressure);
  previousMillis = millis();
   if(PRESS)
  {
    while((millis()- previousMillis) <= interval)
    {
      (ACC & GYRO)? Serial.println(0xfe):Serial.println(0x55);
    }
  }
   
}
