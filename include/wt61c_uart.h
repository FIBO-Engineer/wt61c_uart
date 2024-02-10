#ifndef WT61C_UART_H
#define WT61C_UART_H

#include <vector>
#include <math.h>
#include <time.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "wt61c_uart/Write.h"

namespace wt61c_uart
{
	enum class Command : uint8_t
	{
		resetYawAngle = 0x52,
		toggleSleepMode = 0x60,
		serialMode = 0x61,
		i2CMode = 0x62, // Very Dangerous
		serial115200 = 0x63,
		serial9600 = 0x64,
		horizontalInstallation = 0x65,
		verticalInstallation = 0x66,
		calibrateAcceleration = 0x67,
	};

	class Wt61cUart
	{
	public:
		Wt61cUart(ros::NodeHandle &);
		~Wt61cUart();
		bool initialize();
		void shutdown();

		/* Incoming */
		void retrieveData();
		bool verifyChecksum() const;
		void clearBuffer();
		void decodeAndPublish();

		/* Outcoming */
		bool write(wt61c_uart::Write::Request &request, wt61c_uart::Write::Response &response);

	private:
		std::string dev_path_;
		int baud_;
		std::string frame_id_;

		/* Serial Instance*/
		serial::Serial serial_;
		std::vector<uint8_t> uart_data_;
		unsigned int header_index_;

		ros::Publisher imu_pub_;
		ros::ServiceServer write_srv_;
	};
}
#endif