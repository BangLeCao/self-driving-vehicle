#pragma once

#include <libserial/SerialPort.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "json.hpp"

#include <cstdlib>
#include <cstdint> // type variables: int64_t, uint8_t...
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono> // for timestamp since epoch
#include <iomanip> // for setprecision, setfill

using namespace LibSerial;
using namespace std;
using namespace std::chrono;
namespace nh = nlohmann;

constexpr const char* const USB_SERIAL_PORT = "/dev/ttyUSB1" ;
constexpr const char* const IMU_SERIAL_PORT = "/dev/ttyUSB2" ;

#define DEBUG 0
#define INFO(thread, message) "["#thread": "<<hmsCurrent()<<"] "<<message

typedef struct my_serials {
    SerialPort      *usb_port;
    SerialPort      *imu_port;
	std::string     imuReadData;
    std::string     usbReadData;
	ros::Publisher  pub;
	ros::Subscriber sub;
    std::ofstream   log_ofs; // for logging file
    pthread_mutex_t usb_lockWrite;
    pthread_mutex_t usb_lockRead;
    pthread_t imu_thread_id;
	pthread_t usb_thread_id;
} my_serials_t;


namespace coordinate_ns {
	typedef struct mapData {
		size_t idx;
		double x;
		double y;
	} mapData_t;

	void from_json(const nh::json& j, coordinate_ns::mapData_t& val);
	void to_json(nh::json& j, const coordinate_ns::mapData_t& val);
}

void config_serial(LibSerial::SerialPort *, LibSerial::BaudRate, LibSerial::CharacterSize, 
					LibSerial::FlowControl, LibSerial::Parity, LibSerial::StopBits);

std::vector<std::string> split(const std::string& s, char seperator);

std::string checkSumToString(uint8_t);
std::string formatMessage(std::string);
std::string msTimeToString(int64_t);
std::string hmsCurrent();
