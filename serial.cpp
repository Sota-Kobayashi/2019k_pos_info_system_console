#include "serial.hpp"

int Serial::open(const std::string& port_name, const SerialInitStruct& init_struct)
{
	com_port_name_ = port_name;

	WCHAR com_port_name_wchar[10] = {};
	_stprintf_s(com_port_name_wchar, _T("%hs"), com_port_name_.c_str());
	com_port_ = CreateFile(com_port_name_wchar, (GENERIC_READ | GENERIC_WRITE), 0, nullptr, OPEN_EXISTING, 0, nullptr);

	if (com_port_ == INVALID_HANDLE_VALUE)
	{
		return -1;
	}

	DCB dcb;

	GetCommState(com_port_, &dcb);

	dcb.BaudRate	= init_struct.baudrate;
	dcb.ByteSize	= init_struct.byte_size;
	dcb.Parity = init_struct.parity;
	dcb.StopBits = init_struct.stop_bits;
	dcb.fOutxCtsFlow = init_struct.cts_flow;
	dcb.fRtsControl = init_struct.rts_flow;

	if (!SetCommState(com_port_, &dcb))
	{
		return -1;
	}

	return 0;
}

int Serial::available()
{
	DWORD errors;
	COMSTAT com_stat;

	ClearCommError(com_port_, &errors, &com_stat);

	return com_stat.cbInQue;
}

int Serial::receive(uint8_t& set_receive_byte)
{
	uint8_t receive_data;
	DWORD num_of_read;

	if (available() < 1 || !ReadFile(com_port_, &receive_data, 1, &num_of_read, nullptr))
	{
		return -1;
	}

	set_receive_byte = receive_data;
	return 0;
}