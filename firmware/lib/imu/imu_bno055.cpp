#include "imu_bno055.h"

IMU_BNO055::IMU_BNO055() : bno(Adafruit_BNO055(55, 0x29)) {}

bool IMU_BNO055::init() {
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  }
  bno.setExtCrystalUse(true);
  return true;
}

bool IMU_BNO055::getQuaternion(float &qx, float &qy, float &qz, float &qw) {
  imu::Quaternion quat = bno.getQuat();
  qx = (float)quat.x();
  qy = (float)quat.y();
  qz = (float)quat.z();
  qw = (float)quat.w();
  return true;
}

bool IMU_BNO055::getAngVelocity(float &wx, float &wy, float &wz) {
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  wx = angVelocityData.gyro.x;
  wy = angVelocityData.gyro.y;
  wz = angVelocityData.gyro.z;  
  return true;
}

bool IMU_BNO055::getLinearAcc(float &ax, float &ay, float &az) {
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ax = linearAccelData.acceleration.x;
  ay = linearAccelData.acceleration.y;
  az = linearAccelData.acceleration.z;  
  return true;
}

bool IMU_BNO055::getMagnetometer(float &mx, float &my, float &mz) {
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mx = magnetometerData.magnetic.x;
  my = magnetometerData.magnetic.y;
  mz = magnetometerData.magnetic.z;  
  return true;
}

void IMU_BNO055::getIMUData(sensor_msgs__msg__Imu &imu_msg) {
  float qx, qy, qz, qw;
  getQuaternion(qx, qy, qz, qw);
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;
  float ax, ay, az;
  getLinearAcc(ax, ay, az);
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  float wx, wy, wz;
  getAngVelocity(wx, wy, wz);
  imu_msg.angular_velocity.x = wx;
  imu_msg.angular_velocity.y = wy;
  imu_msg.angular_velocity.z = wz;
}