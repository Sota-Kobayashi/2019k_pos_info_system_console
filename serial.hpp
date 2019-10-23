#pragma once

#include <Windows.h>
#include <tchar.h>
#include <string>
#include <array>
#include <vector>

class Serial
{
public:
	struct SerialInitStruct
	{
		DWORD baudrate = 9600;
		DWORD byte_size = 8;
		DWORD parity = NOPARITY;
		BYTE stop_bits = ONESTOPBIT;
		DWORD cts_flow = false;
		DWORD rts_flow = false;
	};

	Serial() {};
	virtual~Serial() { CloseHandle(com_port_); }
	
	int open(const std::string& port_name, const SerialInitStruct& init_struct);
	inline void transmit(const uint8_t transmit_data) const { WriteFile(com_port_, &transmit_data, 1, nullptr, nullptr); }
	template<size_t S>
	inline void transmit(const std::array<uint8_t, S>& transmit_data_arr) const { for (auto i : transmit_data_arr) transmit(i); }
	inline void transmit(const std::vector<uint8_t>& transmit_data_vec) const { for (auto i : transmit_data_vec) transmit(i); }

	int available();
	int receive(uint8_t& set_receive_byte);

private:
	HANDLE com_port_ = nullptr;
	std::string com_port_name_ = {};
};