#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "communicate_mpu.hpp"
#include "lrf.hpp"

class MachineInfo
{
public:
	MachineInfo() {}
	~MachineInfo() {}

	inline void mpu_message_receive() { communicate_mpu_.receive(); }
	void readLRF();
	inline int startMesurement() { return lrf_.startMesurement(); }

	inline int lrfPortOpen(const std::string& lrf_port_name, const long baudrate) { return lrf_.portOpen(lrf_port_name, baudrate); }
	inline int serialPortOpen(const std::string& port_name, const Serial::SerialInitStruct& init_struct) { return communicate_mpu_.open(port_name, init_struct); }

	bool zone_is_red() { return communicate_mpu_.zone_is_red(); }

	inline int16_t x_odometry() { return communicate_mpu_.lastPos().posX(); }
	inline int16_t y_odometry() { return communicate_mpu_.lastPos().posY(); }
	inline float theta_odometry() { return communicate_mpu_.lastPos().posTheta(); }

	const int16_t& x_lrf = x_lrf_last_;
	const int16_t& y_lrf = y_lrf_last_;

private:
	static constexpr int LRF_DATA_ARR_SIZE = 1080;
	LRF lrf_;
	CommunicateMPU communicate_mpu_;

	int16_t x_lrf_last_ = 0;
	int16_t y_lrf_last_ = 0;

	std::pair<long, long> x_y_detectDistance(const std::array<long, LRF_DATA_ARR_SIZE>& lrf_distance);
	void calibratePosition(const float x_detected_val, const float y_detected_val);
};