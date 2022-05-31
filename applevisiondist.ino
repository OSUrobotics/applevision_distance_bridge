/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <limits>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <Wire.h>
#include "src/Adafruit_VL53L0X/Adafruit_VL53L0X.h"

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
uint32_t cur_seq = 0;
ros::Publisher dist("applevision/apple_dist", &range_msg);
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
  range_msg.header.frame_id = "palm_dist";
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.field_of_view = 25.0f/180.0f*M_PI;
  range_msg.min_range = 0.0f;
  range_msg.max_range = 2.0f;

  // TODO: watchdog
  nh.initNode();
  while (!nh.connected())
    nh.spinOnce();
  
  if (!lidar.begin(VL53L0X_I2C_ADDR, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED))
    error("LIDAR: failed to connect to VL53L0X");

  if (VL53L0X_SetDeviceMode(lidar.pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != 0)
    error("LIDAR: failed to set the VL53L0X in continuous mode");

  if (VL53L0X_SetMeasurementTimingBudgetMicroSeconds(lidar.pMyDevice, 100000) != 0)
    error("LIDAR: failed to set the timing budget for the VL53L0X");

  nh.advertise(dist);
  nh.negotiateTopics();
  nh.now();

  if (VL53L0X_StartMeasurement(lidar.pMyDevice) != 0)
    error("LIDAR: failed to start the first measurement");
}

void loop()
{
  auto start = millis();
  while (!lidar.isRangeComplete()) {
    if (millis() - start > 1000) {
      nh.logwarn("LIDAR: Timed out waiting for the VL53L0X to finish measuring");
      return;
    }
  }
  auto stamp = nh.now();

  VL53L0X_RangingMeasurementData_t measure; // keep our own private copy
  auto status = VL53L0X_GetRangingMeasurementData(lidar.pMyDevice, &measure);
  auto range_status = measure.RangeStatus;
  if (status == VL53L0X_ERROR_NONE)
    status = VL53L0X_ClearInterruptMask(lidar.pMyDevice, 0);
  if ((status != VL53L0X_ERROR_NONE) || (range_status != 0 && range_status != 2 && range_status != 4)) {
    char buf[256];
    snprintf(buf, sizeof(buf), "LIDAR: range read failed with status %d and range status %d", status, range_status);
    nh.logwarn(buf);
    return;
  }

  if (range_status == 2)
    nh.logwarn("LIDAR: sensor reported low signal quality");
  if (range_status == 4)
    nh.logwarn("LIDAR: sensor reported phase aliasing (more than one object?)");

  range_msg.header.stamp = stamp;
  range_msg.header.seq = cur_seq;
  range_msg.range = (float)measure.RangeMilliMeter / 1000.0f;
  dist.publish( &range_msg );
  nh.spinOnce();

  if (cur_seq == std::numeric_limits<uint32_t>::max())
    cur_seq = 0;
  else
    cur_seq++;
}
