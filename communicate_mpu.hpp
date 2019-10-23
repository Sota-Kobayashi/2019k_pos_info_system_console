#pragma once

#include <utility>
#include <cstring>
#include "serial.hpp"
#include "position.hpp"

class CommunicateMPU
{
public:
	CommunicateMPU() : last_pos(0.0f, 0.0f){}
	~CommunicateMPU() {}

	void receive();
	inline position<float> lastPos() { return last_pos; }
	const bool zone_is_red() { return zone_is_red_; };
	void transmit_pos(const std::pair<int16_t, int16_t>& transmit_x_y);

	inline int open(const std::string& port_name, const Serial::SerialInitStruct& init_struct) { return mpu_serial_.open(port_name, init_struct); }
private:
	Serial mpu_serial_;

	position<float> last_pos;
	bool zone_is_red_ = false;

	static constexpr int RECEIVE_MESSAGE_SIZE = 11;
	static constexpr int TRANSMIT_MESSAGE_SIZE = 6;

	static constexpr uint8_t START_BYTE = 0x80;
};