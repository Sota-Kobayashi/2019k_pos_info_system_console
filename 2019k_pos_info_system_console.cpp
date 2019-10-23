// 2019k_pos_info_system_console.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>

#include "machine_info.hpp"

int main()
{
	std::string urg_port_name;
	std::string mpu_port_name;
	MachineInfo machine;

	Serial::SerialInitStruct mpu_port_init;
	mpu_port_init.baudrate = 38400;
	
	while (true)
	{
		urg_port_name.clear();
		std::cout << "LRF PORT : ";
		std::cin >> urg_port_name;
		if (machine.lrfPortOpen(urg_port_name, 57600) == 0) break;;
	}
	
	while (true)
	{
		mpu_port_name.clear();
		std::cout << "MPU PORT : ";
		std::cin >> mpu_port_name;
		if (machine.serialPortOpen(mpu_port_name, mpu_port_init) == 0) break;
	}

	machine.startMesurement();
	std::cout << "start..." << std::endl;
	
	while (true)
	{
		//system("cls");
		machine.mpu_message_receive();
		machine.readLRF();

		if (machine.zone_is_red())
		{
			std::cout << "RED: ";
		}
		else
		{
			std::cout << "BLUE: ";
		}
		std::cout << machine.x_odometry() << ", " << machine.y_odometry() << ", " << machine.theta_odometry() << "; " << machine.x_lrf << ", " << machine.y_lrf << std::endl;

	}
}

// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//    1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
