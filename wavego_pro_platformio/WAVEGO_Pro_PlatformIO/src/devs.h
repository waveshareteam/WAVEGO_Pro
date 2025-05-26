#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

float ACC_X;
float ACC_Y;
float ACC_Z;

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

void InitICM20948(){
  myIMU.init();
  delay(200);
  myIMU.autoOffsets();

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);
}

void accXYZUpdate(){
  xyzFloat gValue;
  myIMU.readSensor();
  // xyzFloat accRaw = myIMU.getAccRawValues();
  // xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  // xyzFloat gVal = myIMU.getGValues();
  myIMU.getGValues(&gValue);

  ACC_X = gValue.x;
  ACC_Y = gValue.y;
  ACC_Z = gValue.z;
}


#include <INA219_WE.h>
#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
bool ina219_overflow = false;

void InitINA219(){
  ina219.init();
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}