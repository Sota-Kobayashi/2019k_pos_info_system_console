#include "communicate_mpu.hpp"
#include <iostream>
void CommunicateMPU::receive()
{
	static int receive_count = 0;
	static std::array<uint8_t, RECEIVE_MESSAGE_SIZE> received_message = {};

	uint8_t receive_byte = 0;
	uint8_t check_sum = 0;
	uint32_t receive_theta_data = 0;
	float receive_theta_float = 0.0f;

	while (mpu_serial_.available() > 0)
	{
		if (mpu_serial_.receive(receive_byte) == -1) return;
		received_message.at(receive_count) = receive_byte;
		if (received_message.at(0) == START_BYTE)
		{
			++receive_count;
			if (receive_count == RECEIVE_MESSAGE_SIZE)
			{
				check_sum = 0;
				for (uint8_t i = 1; i < RECEIVE_MESSAGE_SIZE - 1; ++i) check_sum += received_message.at(i);
				if (received_message.at(RECEIVE_MESSAGE_SIZE - 1) == check_sum)
				{
					last_pos.posX(static_cast<float>(static_cast<int16_t>(static_cast<int16_t>(received_message.at(1)) << 8 | static_cast<int16_t>(received_message.at(2)))));
					last_pos.posY(static_cast<float>(static_cast<int16_t>(static_cast<int16_t>(received_message.at(3)) << 8 | static_cast<int16_t>(received_message.at(4)))));
				
					receive_theta_data =
						static_cast<uint32_t>(received_message.at(5)) << 24 |
						static_cast<uint32_t>(received_message.at(6)) << 16 |
						static_cast<uint32_t>(received_message.at(7)) << 8 |
						static_cast<uint32_t>(received_message.at(8));

					std::memcpy(&receive_theta_float, &receive_theta_data, 4);
					last_pos.posTheta(receive_theta_float);

					zone_is_red_ = received_message.at(9) == 0 ? true : false;// red zone -> 0x00, blue zone -> not 0x00
				}
				receive_count = 0;
			}
		}
		else
		{
			receive_byte = 0;
		}
	}
}

void CommunicateMPU::transmit_pos(const std::pair<int16_t, int16_t>& transmit_x_y)
{
	std::array<uint8_t, TRANSMIT_MESSAGE_SIZE> transmit_message = {};

	transmit_message.at(0) = START_BYTE;
	transmit_message.at(1) = static_cast<uint8_t>(transmit_x_y.first >> 8);
	transmit_message.at(2) = static_cast<uint8_t>(transmit_x_y.first);
	transmit_message.at(3) = static_cast<uint8_t>(transmit_x_y.second >> 8);
	transmit_message.at(4) = static_cast<uint8_t>(transmit_x_y.second);

	uint8_t check_sum = 0;
	for (uint8_t i = 1; i < TRANSMIT_MESSAGE_SIZE - 1; ++i)check_sum += transmit_message.at(i);
	transmit_message.at(5) = check_sum;

	//mpu_serial_.transmit(transmit_message);
}