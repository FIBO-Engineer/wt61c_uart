#include "wt61c_uart.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

wt61c_uart::Wt61cUart::Wt61cUart(ros::NodeHandle &nh)
{
	// Retrieve parameters without specifying the node name and without using getParamCached
	nh.param<std::string>("dev_path", dev_path_, "/dev/ttyUSB0");
	nh.param<int>("baud", baud_, 115200);

	imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 1);
	write_srv_ = nh.advertiseService("/imu/cmd", &Wt61cUart::write, this);
}

wt61c_uart::Wt61cUart::~Wt61cUart()
{
	shutdown();
}

bool wt61c_uart::Wt61cUart::initialize()
{
	try
	{
		serial_.setPort(dev_path_);									   // Set the device path for the serial port
		serial_.setBaudrate(baud_);									   // Set the baud rate for the serial port
		serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // Set a simple timeout of 500ms for the serial port
		serial_.setTimeout(timeout);								   // Apply the timeout settings to the serial port
		serial_.open();												   // Attempt to open the serial port

		if (serial_.isOpen()) // Check if the serial port was successfully opened
		{
			ROS_INFO_STREAM("[WT61C IMU] Serial port initialized successfully.");
			serial_.flushInput(); // Clear the input buffer to ensure clean communication
			return true;		  // Return true to indicate success
		}
		else
		{
			ROS_ERROR_STREAM("[WT61C IMU] Serial port failed to open.");
			return false; // Return false if the port did not open successfully
		}
	}
	catch (const serial::IOException &e)
	{
		ROS_ERROR_STREAM("[WT61C IMU] IOException on opening serial port: " << e.what());
		return false; // Return false upon catching an exception, indicating failure to initialize
	}
	catch (const std::exception &e) // Catch other std exceptions
	{
		ROS_ERROR_STREAM("[WT61C IMU] Exception on opening serial port: " << e.what());
		return false; // Return false, indicating failure
	}
	catch (...) // Catch all other exceptions
	{
		ROS_ERROR_STREAM("[WT61C IMU] Unknown exception on opening serial port.");
		return false; // Return false, indicating failure
	}
}

void wt61c_uart::Wt61cUart::shutdown()
{
	std::cout << "[WT61C IMU] Shutting down." << std::endl;
	if (serial_.isOpen())
	{
		serial_.close();
		std::cout << "[WT61C IMU] Serial port closed." << std::endl;
	}
}

// read data function
void wt61c_uart::Wt61cUart::retrieveData()
{
	constexpr int packet_size = 33;
	while (serial_.waitReadable())
	{
		serial_.read(uart_data_, serial_.available());
		if (uart_data_.size() >= packet_size)
		{
			for (int i = 0; i < uart_data_.size(); i++)
			{
				if (uart_data_.size() - i >= packet_size && uart_data_[i] == 0x55 && uart_data_[i + 1] == 0x51)
				{
					header_index_ = i;
					return;
				}
			}
			throw std::runtime_error("Unable to find header index.");
		}
	}
	throw std::runtime_error("Serial port not readable (timeout or interruption).");
}

bool wt61c_uart::Wt61cUart::verifyChecksum() const
{

	int validSections = 0;
	for (int sectionStart : {0, 11, 22})
	{
		// std::stringstream ss;
		uint8_t sum = 0x00; // Reset sum for each section
		for (int i = sectionStart; i < sectionStart + 10; i++)
		{
			sum += uart_data_[header_index_ + i];
			// ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << int(uart_data_[header_index_ + i]) << " ";
		}
		// std::cout << ss.str() << std::endl;
		if (uart_data_[header_index_ + sectionStart + 10] == sum)
			validSections++;
		else
			ROS_INFO("[WT61C IMU] Invalid Checksum of %d (Received %d != Calculated %d)", sectionStart, uart_data_[header_index_ + sectionStart + 10], sum);
	}
	return validSections == 3;
}

void wt61c_uart::Wt61cUart::decodeAndPublish()
{
	// https://witmotion-sensor.com/products/witmotion-wt61c-high-accuracy-accelerometer-sensor-6-axis-acceleration-16g-gyro-angle-xy-0-05-accuracy-with-kalman-filtering-mpu6050-ahrs-imu-unaffected-by-magnetic-field-for-arduino
	constexpr double linear_acceleration_covariance = (0.01 * 9.80665) * (0.01 * 9.80665);
	constexpr double angular_velocity_covariance_dynamic = (0.1 * M_PI / 180.0) * (0.1 * M_PI / 180.0);
	constexpr double angular_velocity_covariance_static = (0.05 * M_PI / 180.0) * (0.05 * M_PI / 180.0);
	constexpr double sampling_period = 0.01;
	constexpr double d_orientation_covariance_static = sampling_period * sampling_period * angular_velocity_covariance_static;
	constexpr double d_orientation_covariance_dynamic = sampling_period * sampling_period * angular_velocity_covariance_dynamic;
	static double orientation_covariance = 0.0;

	sensor_msgs::Imu imu_msg; // IMU message for publishing
	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = "imu"; // Frame ID for the IMU data

	// Convert and assign linear acceleration values
	imu_msg.linear_acceleration.x = static_cast<int16_t>((uart_data_[header_index_ + 3] << 8) + uart_data_[header_index_ + 2]) / 32768.0 * 16.0 * 9.8;
	imu_msg.linear_acceleration.y = static_cast<int16_t>((uart_data_[header_index_ + 5] << 8) + uart_data_[header_index_ + 4]) / 32768.0 * 16.0 * 9.8;
	imu_msg.linear_acceleration.z = static_cast<int16_t>((uart_data_[header_index_ + 7] << 8) + uart_data_[header_index_ + 6]) / 32768.0 * 16.0 * 9.8;
	imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
	imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
	imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

	// Convert and assign angular velocity values
	imu_msg.angular_velocity.x = static_cast<int16_t>((uart_data_[header_index_ + 14] << 8) + uart_data_[header_index_ + 13]) / 32768.0 * 2000.0 * M_PI / 180.0;
	imu_msg.angular_velocity.y = static_cast<int16_t>((uart_data_[header_index_ + 16] << 8) + uart_data_[header_index_ + 15]) / 32768.0 * 2000.0 * M_PI / 180.0;
	imu_msg.angular_velocity.z = static_cast<int16_t>((uart_data_[header_index_ + 18] << 8) + uart_data_[header_index_ + 17]) / 32768.0 * 2000.0 * M_PI / 180.0;

	bool isDynamic = imu_msg.angular_velocity.x + imu_msg.angular_velocity.y + imu_msg.angular_velocity.z != 0.0;

	if (isDynamic)
	{
		imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance_dynamic;
		imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance_dynamic;
		imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance_dynamic;
		orientation_covariance += d_orientation_covariance_dynamic;
	}
	else
	{
		imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance_static;
		imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance_static;
		orientation_covariance += d_orientation_covariance_static;
	}

	// Convert and assign orientation (roll, pitch, yaw) using a quaternion
	tf2::Quaternion quaternion;

	double roll = static_cast<int16_t>((uart_data_[header_index_ + 25] << 8) + uart_data_[header_index_ + 24]) / 32768.0 * M_PI;
	double pitch = static_cast<int16_t>((uart_data_[header_index_ + 27] << 8) + uart_data_[header_index_ + 26]) / 32768.0 * M_PI;
	double yaw = static_cast<int16_t>((uart_data_[header_index_ + 29] << 8) + uart_data_[header_index_ + 28]) / 32768.0 * M_PI;

	quaternion.setRPY(roll, pitch, yaw);
	imu_msg.orientation = tf2::toMsg(quaternion);
	imu_msg.orientation_covariance[0] += orientation_covariance;
	imu_msg.orientation_covariance[4] += orientation_covariance;
	imu_msg.orientation_covariance[8] += orientation_covariance;

	imu_pub_.publish(imu_msg); // Publish the IMU message
}

void wt61c_uart::Wt61cUart::clearBuffer()
{
	uart_data_ = std::vector<uint8_t>();
}

bool wt61c_uart::Wt61cUart::write(wt61c_uart::Write::Request &request, wt61c_uart::Write::Response &response)
{
	Command cmd = static_cast<Command>(request.data);
	switch (cmd)
	{
	case Command::resetYawAngle:
		response.success = true;
		response.message = "Reset yaw angle command sent.";
		break;
	case Command::toggleSleepMode:
		response.success = true;
		response.message = "Toggle sleep mode sent.";
		break;
	case Command::serialMode:
		response.success = true;
		response.message = "Serial mode sent.";
		break;
	case Command::i2CMode:
		response.success = false;
		response.message = "I2C mode not allowed! Please change with software or change the code.";
		break;
	case Command::serial115200:
		response.success = true;
		response.message = "Serial baudrate 115200 sent.";
		break;
	case Command::serial9600:
		response.success = true;
		response.message = "Serial baudrate 9600 sent.";
		break;

	case Command::horizontalInstallation:
		response.success = true;
		response.message = "Horizontal installation sent.";
		break;

	case Command::verticalInstallation:
		response.success = true;
		response.message = "Vertical installation sent.";
		break;

	case Command::calibrateAcceleration:
		response.success = true;
		response.message = "Acceleration calibration sent.";
		break;

	default:
		response.success = false;
		response.message = "Unrecognized command.";
		break;
	}
	if (response.success)
	{
		std::vector<uint8_t> output_data{0xFF, 0xAA, request.data};
		serial_.write(output_data);
	}
	return true;
}