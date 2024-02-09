#ifndef WT61C_UART_H
#define WT61C_UART_H

#include <vector>
#include <math.h>
#include <time.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"

namespace WTU
{
	class Wt61cUart
	{
	public:
		Wt61cUart(ros::NodeHandle &);
		~Wt61cUart();
		bool initialize();
		void shutdown();

		void retrieveData();
		bool isHeaderIndexValid() const;
		bool isSizeValid() const;
		bool isPacketValid() const;
		bool verifyChecksum() const;

		void clearBuffer();
		void decodeAndPublish();

	private:
		std::string dev_path_;
		int baud_;

		/* Serial Instance*/
		serial::Serial serial_;
		std::vector<uint8_t> uart_data_;
		unsigned int header_index_;

		ros::Publisher pub_;
	};
}
#endif