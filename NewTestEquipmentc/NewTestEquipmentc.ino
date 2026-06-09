#include <Wire.h>
#include <ICM_20948.h>

#define I2C_SDA  46
#define I2C_SCL  45
#define TCA_ADDR 0x70

ICM_20948_I2C imu1, imu2, imu3;
bool imu1ok = false, imu2ok = false, imu3ok = false;
bool tcaOk  = false;

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool checkTCA() {
  Wire.beginTransmission(TCA_ADDR);
  return Wire.endTransmission() == 0;
}

// tries 0x68 first, then 0x69
bool initIMU(ICM_20948_I2C &imu, uint8_t channel, String name) {
  tcaSelect(channel);
  delay(50);

  // try 0x68 first (AD0 LOW)
  imu.begin(Wire, 0);
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(name); Serial.println(" found at 0x68 ✅");
    return true;
  }

  // try 0x69 (AD0 HIGH)
  imu.begin(Wire, 1);
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(name); Serial.println(" found at 0x69 ✅");
    return true;
  }

  Serial.print(name); Serial.println(" NOT FOUND ❌");
  return false;
}

void printIMUData(ICM_20948_I2C &imu, uint8_t channel, String name) {
  tcaSelect(channel);
  if (!imu.dataReady()) {
    Serial.print(name); Serial.println(": not ready");
    return;
  }
  imu.getAGMT();
  Serial.print(name);
  Serial.print(" | Accel(g) X:"); Serial.print(imu.accX(), 2);
  Serial.print(" Y:"); Serial.print(imu.accY(), 2);
  Serial.print(" Z:"); Serial.print(imu.accZ(), 2);
  Serial.print(" | Gyro(dps) X:"); Serial.print(imu.gyrX(), 2);
  Serial.print(" Y:"); Serial.print(imu.gyrY(), 2);
  Serial.print(" Z:"); Serial.print(imu.gyrZ(), 2);
  Serial.print(" | Mag(uT) X:"); Serial.print(imu.magX(), 2);
  Serial.print(" Y:"); Serial.print(imu.magY(), 2);
  Serial.print(" Z:"); Serial.println(imu.magZ(), 2);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("=== TCA + IMU Test ===");
  Serial.println("Plug/unplug freely — retries every 2 seconds");
}

void loop() {
  Serial.println("\n--- Scanning ---");

  // check TCA
  tcaOk = checkTCA();
  if (tcaOk) {
    Serial.println("TCA9548A at 0x70 ✅");
  } else {
    Serial.println("TCA9548A NOT FOUND ❌ — check wiring");
    // no point checking IMUs without TCA
    delay(2000);
    return;
  }

  // try to init any IMU that isn't working yet
  if (!imu1ok) imu1ok = initIMU(imu1, 1, "IMU1 (upper, ch1)");
  if (!imu2ok) imu2ok = initIMU(imu2, 2, "IMU2 (mid,   ch2)");
  if (!imu3ok) imu3ok = initIMU(imu3, 7, "IMU3 (lower, ch7)");

  // read whichever IMUs are online
  Serial.println("--- Readings ---");
  if (imu1ok) printIMUData(imu1, 1, "IMU1 (upper)");
  if (imu2ok) printIMUData(imu2, 2, "IMU2 (mid)  ");
  if (imu3ok) printIMUData(imu3, 7, "IMU3 (lower)");

  // summary
  Serial.println("--- Status ---");
  Serial.print("IMU1: "); Serial.println(imu1ok ? "online ✅" : "offline ❌");
  Serial.print("IMU2: "); Serial.println(imu2ok ? "online ✅" : "offline ❌");
  Serial.print("IMU3: "); Serial.println(imu3ok ? "online ✅" : "offline ❌");
  Serial.println("======================================");

  delay(2000);
}