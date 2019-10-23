#include "machine_info.hpp"
#include <iostream>

void MachineInfo::readLRF()
{
	std::array<long, LRF_DATA_ARR_SIZE> lrf_distance = {};
	lrf_.getDistance(lrf_distance);

	const std::pair<long, long> x_y_detect_val = x_y_detectDistance(lrf_distance);
	calibratePosition(static_cast<float>(x_y_detect_val.first), static_cast<float>(x_y_detect_val.second));
}

std::pair<long, long> MachineInfo::x_y_detectDistance(const std::array<long, LRF_DATA_ARR_SIZE>& lrf_distance)
{
	constexpr std::pair<int, int> SCANNABLE_AREA = { 180, 900 };

	constexpr int THETA_ZERO_DATA_POS = 540;
	constexpr int THETA_90DEG_DATA_POS = 180;

	constexpr float STEP_PER_DEG = 0.25;

	const int x_lrf_data_pos = static_cast<int>((communicate_mpu_.lastPos().posTheta() - (communicate_mpu_.zone_is_red() ? M_PI : 0.0f)) * 180.0f / (M_PI * STEP_PER_DEG)) + THETA_ZERO_DATA_POS;
	const int y_lrf_data_pos = static_cast<int>(communicate_mpu_.lastPos().posTheta() * 180.0f / (M_PI * STEP_PER_DEG)) + THETA_90DEG_DATA_POS;

	const long x_val = (x_lrf_data_pos < lrf_distance.size() && x_lrf_data_pos >= SCANNABLE_AREA.first && x_lrf_data_pos <= SCANNABLE_AREA.second) ? lrf_distance.at(x_lrf_data_pos) : -1;
	const long y_val = (y_lrf_data_pos < lrf_distance.size() && y_lrf_data_pos >= SCANNABLE_AREA.first && y_lrf_data_pos <= SCANNABLE_AREA.second) ? lrf_distance.at(y_lrf_data_pos) : -1;

	x_lrf_last_ = static_cast<int16_t>(x_val);
	y_lrf_last_ = static_cast<int16_t>(y_val);

	return std::make_pair(x_val, y_val);
}

void MachineInfo::calibratePosition(const float x_detected_val, const float y_detected_val)
{
	constexpr std::pair<float, float> FIELED_SIZE = { 5600.0f, 10000.0f };

	constexpr std::pair<float, float> LRF_POS = { 329.8f, 0.0f };

	constexpr std::pair<float, float> rod_base_1_x = { 1200.0f, 1950.0f };
	constexpr std::pair<float, float> rod_base_2_x = { 4350.0f, 5100.0f };

	constexpr std::pair<float, float> rod_base_h1000_y = { 4100.0f, 4900.0f };
	constexpr std::pair<float, float> rod_base_h1500_y = { 6100.0f, 6900.0f };
	constexpr std::pair<float, float> rod_base_h2000_y = { 8100.0f, 8900.0f };

	constexpr std::pair<float, float> UPDATE_THETA_RENGE_X_RED = { (0.4f * static_cast<float>(M_PI)), (1.6f * static_cast<float>(M_PI)) };
	constexpr std::pair<float, float> UPDATE_THETA_RENGE_X_BLUE = { (-0.4f * static_cast<float>(M_PI)), (0.6f * static_cast<float>(M_PI)) };
	constexpr std::pair<float, float> UPDATE_THETA_RENGE_Y = { (-0.1f * static_cast<float>(M_PI)), (1.1f * static_cast<float>(M_PI)) };

	static auto now_lrf_pos_local = [&]()
	{
		const float pos_lrf_x = LRF_POS.first * std::cos(communicate_mpu_.lastPos().posTheta()) - LRF_POS.second * std::sin(communicate_mpu_.lastPos().posTheta());
		const float pos_lrf_y = LRF_POS.first * std::sin(communicate_mpu_.lastPos().posTheta()) + LRF_POS.second * std::cos(communicate_mpu_.lastPos().posTheta());

		return std::make_pair(pos_lrf_x, pos_lrf_y);
	};

	static auto check_update_enable_y = [&]()
	{
		const float now_lrf_pos_x = std::abs(communicate_mpu_.lastPos().posX() + now_lrf_pos_local().first);
		const bool theta_is_ok = communicate_mpu_.lastPos().posTheta() > UPDATE_THETA_RENGE_Y.first && communicate_mpu_.lastPos().posTheta() < UPDATE_THETA_RENGE_Y.second;
		return	(now_lrf_pos_x < rod_base_1_x.first || now_lrf_pos_x > rod_base_1_x.second) && (now_lrf_pos_x < rod_base_2_x.first || now_lrf_pos_x > rod_base_2_x.second);
	};

	static auto check_update_enable_x = [&]()
	{
		const float now_lrf_pos_y = std::abs(communicate_mpu_.lastPos().posY() + now_lrf_pos_local().second);
		const bool theta_is_ok = communicate_mpu_.zone_is_red() ?
			communicate_mpu_.lastPos().posTheta() > UPDATE_THETA_RENGE_X_RED.first && communicate_mpu_.lastPos().posTheta() < UPDATE_THETA_RENGE_X_RED.second:
			communicate_mpu_.lastPos().posTheta() > UPDATE_THETA_RENGE_X_BLUE.first && communicate_mpu_.lastPos().posTheta() < UPDATE_THETA_RENGE_X_BLUE.second;
		return	(now_lrf_pos_y < rod_base_h1000_y.first || now_lrf_pos_y > rod_base_h1000_y.second) &&
			(now_lrf_pos_y < rod_base_h1500_y.first || now_lrf_pos_y > rod_base_h1500_y.second) &&
			(now_lrf_pos_y < rod_base_h2000_y.first || now_lrf_pos_y > rod_base_h2000_y.second) && theta_is_ok;
	};

	std::pair<int16_t, int16_t> lrf_update_pos = { 0, 0 };

	//update x position
	if (check_update_enable_x() && x_detected_val > 0.0f)
	{
		lrf_update_pos.first = static_cast<int16_t>(-1.0f * now_lrf_pos_local().first + (communicate_mpu_.zone_is_red() ? x_detected_val : (-1.0f * x_detected_val)));
	}

	//update y position
	if (check_update_enable_y() && y_detected_val > 0.0f)
	{
		lrf_update_pos.second = static_cast<int16_t>(FIELED_SIZE.second - y_detected_val - now_lrf_pos_local().second);
	}

	communicate_mpu_.transmit_pos(lrf_update_pos);
}