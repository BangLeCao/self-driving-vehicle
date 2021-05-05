#pragma once

#include <libserial/SerialPort.h>
#include "json.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace LibSerial;
using namespace std;
namespace nh = nlohmann;

constexpr const char* const USB_SERIAL_PORT = "/dev/ttyUSB2" ;
constexpr const char* const IMU_SERIAL_PORT = "/dev/ttyUSB1" ;

typedef struct my_serials {
    SerialPort      *usb_port;
    SerialPort      *imu_port;
	std::string     imuReadData;
    std::string     usbDataString;
    pthread_mutex_t usb_lockWrite;
    pthread_mutex_t usb_lockRead;
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

