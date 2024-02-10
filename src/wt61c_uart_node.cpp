#include "ros/ros.h"
#include "wt61c_uart.h"
#include "serial/serial.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wt61c_uart_node");
	ros::NodeHandle nh("~");

	wt61c_uart::Wt61cUart wt61c_uart(nh); // initilize the uart parameter

	while (!wt61c_uart.initialize())
	{
		ROS_ERROR("[WT61C IMU] Unable to initialize, retrying in 1 second.");
		wt61c_uart.shutdown();
		ros::Duration(1.0).sleep();
	}

	ros::Rate loop_rate(200);

	while (ros::ok() && !ros::isShuttingDown())
	{
		try
		{
			wt61c_uart.retrieveData();
			if (wt61c_uart.verifyChecksum())
				wt61c_uart.decodeAndPublish();
			else
				ROS_WARN("[WT61C IMU] Wrong Checksum");
			wt61c_uart.clearBuffer();
		}
		catch (const std::runtime_error &e)
		{
			ROS_ERROR("[WT61C IMU] std::runtime error: %s, restart in 1 second.", e.what());
			ros::Duration(1.0).sleep();
			wt61c_uart.initialize();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}