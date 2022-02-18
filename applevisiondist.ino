/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Wire.h>
#include "src/Adafruit_VL53L0X/Adafruit_VL53L0X.h"

ros::NodeHandle  nh;
std_msgs::UInt16 num_msg;
ros::Publisher dist("appl_dist", &num_msg);
Adafruit_VL53L0X lidar;

// for debugging, not used
void log_print_status(const char* func, int status) {
  if (status == 0)
    return;
  char buf[2048];
  snprintf(buf, sizeof(buf), "LIDAR: %s -> %d", func, status);
  nh.loginfo(buf);
}

void error(const char* str) {
  nh.logerror(str);
  auto start = millis();
  while (millis() - start < 5000)
    nh.spinOnce();
  HAL_NVIC_SystemReset();
}

void setup()
{
  // TODO: watchdog
  nh.initNode();
  while (!nh.connected())
    nh.spinOnce();
  
  if (!lidar.begin(VL53L0X_I2C_ADDR, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED))
    error("LIDAR: failed to connect to VL53L0X");

  if (VL53L0X_SetDeviceMode(lidar.pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != 0)
    error("LIDAR: failed to set the VL53L0X in continuous mode");

  if (VL53L0X_SetMeasurementTimingBudgetMicroSeconds(lidar.pMyDevice, 33333) != 0)
    error("LIDAR: failed to set the timing budget for the VL53L0X");

  nh.advertise(dist);
  nh.negotiateTopics();

  if (VL53L0X_StartMeasurement(lidar.pMyDevice) != 0)
    error("LIDAR: failed to start the first measurement");
}

void loop()
{
  auto start = millis();
  while (!lidar.isRangeComplete()) {
    if (millis() - start > 1000)
      error("LIDAR: Timed out waiting for the VL53L0X to finish measuring");
  }

  uint16_t dist_num = lidar.readRangeResult();
  if (dist_num == 0xffff)
    error("LIDAR: Failed to read range result from VL53L0X");

  num_msg.data = dist_num;
  dist.publish( &num_msg );
  nh.spinOnce();
}
