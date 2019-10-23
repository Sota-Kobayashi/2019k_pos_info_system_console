#pragma once

#include <string>
#include <array>
#include <urg_sensor.h>

class LRF
{
public:
	LRF() {}
	~LRF()
	{
		urg_close(&urg_);
	}

	inline int portOpen(const std::string& lrf_port_name, const long baudrate)
	{
		return urg_open(&urg_, urg_connection_type_t::URG_SERIAL, lrf_port_name.c_str(), baudrate);
	}

	template<size_t S>
	inline int getDistance(std::array<long, S>& data_arr)
	{
		long data[1440];
		int return_val = urg_get_distance(&urg_, data, nullptr);
		for (int i = 0; i < S; ++i)data_arr.at(i) = data[i];

		return return_val;
	}

	inline int startMesurement()
	{
		return urg_start_measurement(&urg_, urg_measurement_type_t::URG_DISTANCE, 0, 0);
	}

private:
	urg_t urg_ = {};
};