#include <freespace/freespace.h>
#include <freespace/freespace_printers.h>
#include <freespace/freespace_util.h>
#include <ros/ros.h>

// The names corresponding to the sensor indices
const char* const SENSOR_NAMES[] = {
    "Accelerometer",
    "Gyroscope",
    "Magnetometer",
    "Ambient Light Sensor",
    "Pressure Sensor",
    "Proximity Sensor",
    "Sensor Fusion"
};

/**
 * sendGetSensorPeriodMessage
 * Sends a message to read the sample rate
 * of a sensor on a Freespace device.
 * device - The Freespace Device ID of the device
 * sensor - The sensor index - see the HCOMM document
 * for more information
 * return - The return code of the message
 */
int sendGetSensorPeriodMessage(FreespaceDeviceId device, int sensor) {
	struct freespace_message message;

	// Send the sensor period message with the user parameters
	memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

	message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
	message.sensorPeriodRequest.get = 1;  // We are getting, not setting
	message.sensorPeriodRequest.sensor = sensor; // Sensor index - see HCOMM doc for more info

	return freespace_sendMessage(device, &message);
}

int waitForPeriodResponse(FreespaceDeviceId device, int* sensorValue, int* periodValue) {
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


int printSensorInfo(FreespaceDeviceId device) {
	int rc;
	int index;
	int sensor;
	int period;

	// Update sensor information
	printf("Sensors:\n");
	for (index = 0;index < 7;index++) {
		// Request the sensor period information
		rc = sendGetSensorPeriodMessage(device, index);
		if (rc != FREESPACE_SUCCESS) {
			return rc;
		}

		// Wait for a response
		rc = waitForPeriodResponse(device, &sensor, &period);

		if (rc == FREESPACE_ERROR_TIMEOUT) { // Indicates timeout
			printf("     %d. %s TIMED OUT.\n", index, SENSOR_NAMES[index]);
		} else if (rc == FREESPACE_SUCCESS) {
			printf("     %d. %s", sensor, SENSOR_NAMES[index]);
			if (period != 0)
				printf(" @ %d us.\n", period);
			else
				printf(" disabled.\n");
		} else {
			return rc;
		}
	}
	return FREESPACE_SUCCESS;
	printf("\n");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "fsm_imu");

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
    printf("Found a device. Trying to open it...\n");
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
            printf("Try adding the following udev rule to /etc/udev/rules.d/99-fsm9.rules:\n");
            printf("SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"%x\", ATTRS{idProduct}==\"%x\", MODE=:\"0666\"\n", info.vendor, info.product);
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

    // Print out the sensor info
	rc = printSensorInfo(device);
	if (rc != FREESPACE_SUCCESS) {
		printf("Error getting sensor info: %d\n", rc);
		return 1;
	}

     // Configure the device for motion outputs
    struct freespace_message message;
    printf("Sending message to enable motion data.\n");
    memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8;  // MotionEngine Outout
    message.dataModeControlV2Request.mode = 0;          // Set full motion
    message.dataModeControlV2Request.formatSelect = 0;  // MEOut format 0
    message.dataModeControlV2Request.ff0 = 1;           // Pointer fields
    message.dataModeControlV2Request.ff3 = 1;           // Angular velocity fields

    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("Could not send message: %d.\n", rc);
    }

    // A loop to read messages
    printf("Listening for messages.\n");
    while (ros::ok()) {
        rc = freespace_readMessage(device, &message, 100);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
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
            struct MultiAxisSensor angVel;
            rc = freespace_util_getAngularVelocity(&message.motionEngineOutput, &angVel);
            if (rc == 0) {
                printf ("X: % 6.2f, Y: % 6.2f, Z: % 6.2f\n", angVel.x, angVel.y, angVel.z);
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
