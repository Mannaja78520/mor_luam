#ifndef IMU_BNO055_H
#define IMU_BNO055_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <sensor_msgs/msg/imu.h>

// Define the IMU class
class IMU_BNO055 {
public:
  IMU_BNO055();
  bool init();
  bool getQuaternion(float &qx, float &qy, float &qz, float &qw);
  bool getAngVelocity(float &wx, float &wy, float &wz);
  bool getLinearAcc(float &ax, float &ay, float &az);
  bool getMagnetometer(float &mx, float &my, float &mz);
  void getIMUData(sensor_msgs__msg__Imu &imu_msg);

private:
  Adafruit_BNO055 bno;
  sensors_event_t angVelocityData , linearAccelData, magnetometerData;
};

#endif // IMU_H
