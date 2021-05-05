#include "serial_helper.h"

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

std::string checkSumToString(uint8_t cksum)
{
    char buf[3];

    buf[0] = (cksum & 0xF0) >> 4; // high byte
    buf[0] += (buf[0] < 10) ? 48 : 55;
    buf[1] = cksum & 0x0F; // low byte
    buf[1] += (buf[1] < 10) ? 48 : 55;
    buf[2] = 0; // NULL-terminated

    std::string res(buf);
    return res;
}

std::string formatMessage(std::string msg)
{
    stringstream ss;
    uint8_t checkSum = 0;
    for (std::size_t i = 0; i < msg.length(); ++i)
    {
        checkSum ^= msg.at(i);
    }
    ss << "$" << msg << checkSumToString(checkSum) << "\r\n";
    return ss.str();
}

std::string msTimeToString(int64_t ms_from_epoch)
{
    std::stringstream ss; 

    int64_t s_epoch = ms_from_epoch / 1000;
    int32_t min_epoch = s_epoch / 60;
    int32_t hour_epoch = min_epoch / 60;

    int32_t ms = ms_from_epoch % 1000;
    int32_t s = s_epoch % 60;
    int32_t min = min_epoch % 60;
    int32_t hour = (hour_epoch % 24 + 7) % 24;

    ss << hour << ":" << setw(2) << setfill('0') << min << ":" 
        << setw(2) << setfill('0') << s << "." << setw(3) << setfill('0') << ms;

    return ss.str();
}

std::string hmsCurrent()
{
    return msTimeToString(duration_cast<milliseconds>
        (system_clock::now().time_since_epoch()).count());
}
