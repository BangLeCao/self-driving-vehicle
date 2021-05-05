#include "../include/serial_driver.h"

void config_serial(SerialPort *serial, BaudRate baud, CharacterSize size, FlowControl flow, Parity parity, StopBits bits)
{
    serial->SetBaudRate(baud);
    serial->SetCharacterSize(size);
    serial->SetFlowControl(flow);
    serial->SetParity(parity);
    serial->SetStopBits(bits);
}

std::vector<std::string> split(const std::string& s, char seperator)
{
    std::vector<std::string> output;
    
    std::string::size_type prev_pos = 0, pos = 0;
    while((pos = s.find(seperator, pos)) != std::string::npos)
    {
        std::string substring( s.substr(prev_pos, pos-prev_pos) );
        output.push_back(substring);
        prev_pos = ++pos;
    }
    output.push_back(s.substr(prev_pos, pos-prev_pos)); // Last word
    
    return output;
}

void coordinate_ns::from_json(const nh::json& j, coordinate_ns::mapData_t& val)
{
	val.x = j.at("x").get<double>();
	val.y = j.at("y").get<double>();
	val.idx = j.at("idx").get<size_t>();
}

void coordinate_ns::to_json(nh::json& j, const coordinate_ns::mapData_t& val)
{
	j["x"] = val.x;
	j["y"] = val.y;
	j["idx"] = val.idx;
}

