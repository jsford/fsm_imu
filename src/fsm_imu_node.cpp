#include <freespace/freespace.h>
#include <freespace/freespace_printers.h>
#include <freespace/freespace_util.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <stdint.h>
#include <string>

// The names corresponding to the sensor indices
const char *const SENSOR_NAMES[] = {
  "Accelerometer",
  "Gyroscope",
  "Magnetometer",
  "Ambient Light Sensor",
  "Pressure Sensor",
  "Proximity Sensor",
  "Sensor Fusion"
};

int sendGetSensorPeriodMessage(FreespaceDeviceId device, int sensor) {
  struct freespace_message message;

  // Send the sensor period message with the user parameters
  memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

  message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
  message.sensorPeriodRequest.get = 1; // We are getting, not setting
  message.sensorPeriodRequest.sensor = sensor; // Sensor index - see HCOMM doc for more info

  return freespace_sendMessage(device, &message);
}

int sendSetSensorPeriodMessage(FreespaceDeviceId device, int sensor, uint32_t period_us) {
  struct freespace_message message;

  // Send the sensor period message with the user parameters
  memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

  message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
  message.sensorPeriodRequest.get = 0; // We are getting, not setting
  message.sensorPeriodRequest.sensor = sensor; // Sensor index - see HCOMM doc for more info
  message.sensorPeriodRequest.period = period_us;
  message.sensorPeriodRequest.commit = 1;

  return freespace_sendMessage(device, &message);
}

int waitForPeriodResponse(FreespaceDeviceId device, int *sensorValue, int *periodValue) {
  int rc = 0;
  struct freespace_message message;

  // Keep looping if we get FREESPACE_SUCCESS but no SensorPeriodResponse
  while (rc == FREESPACE_SUCCESS) {
    rc = freespace_readMessage(device, &message, 200);
    if (rc != FREESPACE_SUCCESS) {
      return rc;
    }
    // Check if the sensor has given us a Sensor Period response
    if (message.messageType == FREESPACE_MESSAGE_SENSORPERIODRESPONSE) {
      if (sensorValue != NULL)
        *sensorValue = message.sensorPeriodResponse.sensor;
      if (periodValue != NULL)
        *periodValue = message.sensorPeriodResponse.period;
      return FREESPACE_SUCCESS;
    }
  }
  return 0;
}

int setSensorPeriod(FreespaceDeviceId device, uint32_t imu_period) {
  int rc;
  int index;
  int sensor;
  int period;

  // Update sensor information
  for (index = 0; index < 7; index++) {
    // Request the sensor period information
    rc = sendSetSensorPeriodMessage(device, index, imu_period);
    if (rc != FREESPACE_SUCCESS) {
      return rc;
    }

    // Wait for a response
    rc = waitForPeriodResponse(device, &sensor, &period);
  }
  return rc;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fsm_imu");
  ros::NodeHandle nh("~");

  int imu_period;
  std::string imu_frame_id, topic;
  nh.param<int>("period_us", imu_period, 8000);
  nh.param<std::string>("frame_id", imu_frame_id, "/imu");
  nh.param<std::string>("topic", topic, "/imu/data");

  ros::Publisher pubImu = nh.advertise<sensor_msgs::Imu>(topic, 100);

  // Initialize the freespace library
  int rc = freespace_init();
  if (rc != FREESPACE_SUCCESS) {
    printf("Initialization error. rc=%d\n", rc);
    return 1;
  }

  // Get the ID of the first device in the list of available devices
  int numIds;
  FreespaceDeviceId device;
  rc = freespace_getDeviceList(&device, 1, &numIds);
  if (rc != FREESPACE_SUCCESS || numIds == 0) {
    printf("Didn't find any devices.\n");
    return 1;
  }

  // Prepare to communicate with the device found above
  rc = freespace_openDevice(device);
  if (rc != FREESPACE_SUCCESS) {
    printf("Error opening device: %d\n", rc);

    // If the error is caused by permissions, print helpful udev information.
    if (rc == FREESPACE_ERROR_ACCESS) {
      struct FreespaceDeviceInfo info;
      // Retrieve the information for the device
      rc = freespace_getDeviceInfo(device, &info);
      if (rc != FREESPACE_SUCCESS) {
        return rc;
      }
      printf("\n\nInsufficient USB permissions.\n");
      printf("Try adding the following udev rule to "
             "/etc/udev/rules.d/99-fsm9.rules:\n");
      printf("SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"%x\", "
             "ATTRS{idProduct}==\"%x\", MODE=:\"0666\"\n",
             info.vendor, info.product);
      printf("Then, run the following command to reload udev rules:\n");
      printf("sudo udevadm trigger\n\n\n");
    }

    return 1;
  }

  // Make sure any old messages are cleared out of the system
  rc = freespace_flush(device);
  if (rc != FREESPACE_SUCCESS) {
    printf("Error flushing device: %d\n", rc);
    return 1;
  }

  rc = setSensorPeriod(device, imu_period);
  if (rc != FREESPACE_SUCCESS) {
    printf("Error setting device period.\n");
    return 1;
  }

  // Configure the device for motion outputs
  struct freespace_message message;
  memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

  message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
  message.dataModeControlV2Request.packetSelect = 8; // MotionEngine Outout
  message.dataModeControlV2Request.mode = 4;         // Set full motion on
  message.dataModeControlV2Request.formatSelect = 0; // MEOut format 0
  message.dataModeControlV2Request.ff0 = 1;          // Pointer fields
  message.dataModeControlV2Request.ff1 = 1;          // Pointer fields
  message.dataModeControlV2Request.ff2 = 1;          // Pointer fields
  message.dataModeControlV2Request.ff3 = 1;          // Angular velocity fields
  message.dataModeControlV2Request.ff4 = 1;          // Angular velocity fields
  message.dataModeControlV2Request.ff5 = 1;          // Angular velocity fields
  message.dataModeControlV2Request.ff6 = 1;          // Angular velocity fields
  message.dataModeControlV2Request.ff7 = 1;          // ActClass/PowerMgmt

  rc = freespace_sendMessage(device, &message);
  if (rc != FREESPACE_SUCCESS) {
    printf("Could not send message: %d.\n", rc);
  }

  unsigned int imu_seq = 0;

  // Loop to read messages
  while (ros::ok()) {
    rc = freespace_readMessage(device, &message, 100);
    if (rc == FREESPACE_ERROR_TIMEOUT || rc == FREESPACE_ERROR_INTERRUPTED) {
      // Both timeout and interrupted are ok.
      // Timeout happens if there aren't any events for a second.
      // Interrupted happens if you type CTRL-C or if you
      // type CTRL-Z and background the app on Linux.
      continue;
    }

    if (rc != FREESPACE_SUCCESS) {
      printf("Error reading: %d. Quitting...\n", rc);
      break;
    }

    if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
      struct MultiAxisSensor accel, accelNoGrav, actClass, angPos, angVel,
          compass, inclination, mag;

      bool have_accel = (0 == freespace_util_getAcceleration(&message.motionEngineOutput, &accel));
      bool have_accelNoGrav = (0 == freespace_util_getAcceleration(&message.motionEngineOutput, &accelNoGrav));
      bool have_ActClass = (0 == freespace_util_getActClass(&message.motionEngineOutput, &actClass));
      bool have_angVel = (0 == freespace_util_getAngularVelocity(&message.motionEngineOutput, &angVel));
      bool have_angPos = (0 == freespace_util_getAngPos(&message.motionEngineOutput, &angPos));
      bool have_compass = (0 == freespace_util_getCompassHeading(&message.motionEngineOutput, &compass));
      bool have_inclination = (0 == freespace_util_getInclination(&message.motionEngineOutput, &inclination));
      bool have_mag = (0 == freespace_util_getMagnetometer(&message.motionEngineOutput, &mag));

      if (have_accel && have_angVel && have_angPos) {
        sensor_msgs::Imu msg;
        msg.header.seq = imu_seq++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = imu_frame_id;

        // Extract orientation quaternion.
        msg.orientation.x = angPos.x;
        msg.orientation.y = angPos.y;
        msg.orientation.z = angPos.z;
        msg.orientation.w = angPos.w;
        std::fill(std::begin(msg.orientation_covariance),
                  std::end(msg.orientation_covariance), 0.0);

        // Extract angular velocity.
        msg.angular_velocity.x = angVel.x;
        msg.angular_velocity.y = angVel.y;
        msg.angular_velocity.z = angVel.z;
        std::fill(std::begin(msg.angular_velocity_covariance),
                  std::end(msg.angular_velocity_covariance), 0.0);

        // Extract linear acceleration.
        msg.linear_acceleration.x = accel.x;
        msg.linear_acceleration.y = accel.y;
        msg.linear_acceleration.z = accel.z;
        std::fill(std::begin(msg.linear_acceleration_covariance),
                  std::end(msg.linear_acceleration_covariance), 0.0);

        pubImu.publish(msg);
      }
    }
  }

  // Close communications with the device
  printf("Cleaning up...\n");
  freespace_closeDevice(device);

  // Cleanup the library
  freespace_exit();

  return 0;
}
