#include "wt61c_uart.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

WTU::Wt61cUart::Wt61cUart(ros::NodeHandle &nh)
{
	// Retrieve parameters without specifying the node name and without using getParamCached
	nh.param<std::string>("dev_path", dev_path_, "/dev/ttyUSB0");
	nh.param<int>("baud", baud_, 115200);

	pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 1);
}

WTU::Wt61cUart::~Wt61cUart()
{
	shutdown();
}

bool WTU::Wt61cUart::initialize()
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

void WTU::Wt61cUart::shutdown()
{
	std::cout << "[WT61C IMU] Shutting down." << std::endl;
	if (serial_.isOpen())
	{
		serial_.close();
		std::cout << "[WT61C IMU] Serial port closed." << std::endl;
	}
}

// read data function
void WTU::Wt61cUart::retrieveData()
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

bool WTU::Wt61cUart::verifyChecksum() const
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
			ROS_INFO("Invalid Checksum of %d (Received %d != Calculated %d)", sectionStart, uart_data_[header_index_ + sectionStart + 10], sum);
	}
	return validSections == 3;
}

void WTU::Wt61cUart::clearBuffer()
{
	uart_data_ = std::vector<uint8_t>();
}

void WTU::Wt61cUart::decodeAndPublish()
{
	sensor_msgs::Imu imu_msg; // IMU message for publishing
	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = "imu"; // Frame ID for the IMU data

	// Convert and assign linear acceleration values
	imu_msg.linear_acceleration.x = static_cast<int16_t>((uart_data_[header_index_ + 3] << 8) + uart_data_[header_index_ + 2]) / 32768.0 * 16.0 * 9.8;
	imu_msg.linear_acceleration.y = static_cast<int16_t>((uart_data_[header_index_ + 5] << 8) + uart_data_[header_index_ + 4]) / 32768.0 * 16.0 * 9.8;
	imu_msg.linear_acceleration.z = static_cast<int16_t>((uart_data_[header_index_ + 7] << 8) + uart_data_[header_index_ + 6]) / 32768.0 * 16.0 * 9.8;

	// Convert and assign angular velocity values
	imu_msg.angular_velocity.x = static_cast<int16_t>((uart_data_[header_index_ + 14] << 8) + uart_data_[header_index_ + 13]) / 32768.0 * 2000.0 * M_PI / 180.0;
	imu_msg.angular_velocity.y = static_cast<int16_t>((uart_data_[header_index_ + 16] << 8) + uart_data_[header_index_ + 15]) / 32768.0 * 2000.0 * M_PI / 180.0;
	imu_msg.angular_velocity.z = static_cast<int16_t>((uart_data_[header_index_ + 18] << 8) + uart_data_[header_index_ + 17]) / 32768.0 * 2000.0 * M_PI / 180.0;

	// Convert and assign orientation (roll, pitch, yaw) using a quaternion
	tf2::Quaternion quaternion;

	double roll = static_cast<int16_t>((uart_data_[header_index_ + 25] << 8) + uart_data_[header_index_ + 24]) / 32768.0 * M_PI;
	double pitch = static_cast<int16_t>((uart_data_[header_index_ + 27] << 8) + uart_data_[header_index_ + 26]) / 32768.0 * M_PI;
	double yaw = static_cast<int16_t>((uart_data_[header_index_ + 29] << 8) + uart_data_[header_index_ + 28]) / 32768.0 * M_PI;

	quaternion.setRPY(roll, pitch, yaw);
	imu_msg.orientation = tf2::toMsg(quaternion);

	pub_.publish(imu_msg); // Publish the IMU message
}
