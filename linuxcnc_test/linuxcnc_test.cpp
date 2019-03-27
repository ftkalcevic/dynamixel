// linuxcnc_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "Dynamixel.h"
#include "RX48.h"
#include "linuxcnc_test.h"

int main()
{
	//Dynamixel dmx("\\\\.\\COM21", 3000000);
	Dynamixel dmx("\\\\.\\COM21",3000000);

	//RX48 id2(2);
	//RX48 id3(3);
	//dmx.addDevice(&id2);
	//dmx.addDevice(&id3);

	//dmx.open();
	//dmx.enableTorque(false);
	//dmx.setWheelMode();
	//dmx.readPositions();
	//id2.velocity = 50;
	//id3.velocity = 1024 | 10;
	//dmx.setVelocities();
	//dmx.enableTorque(true);
	//for (int i = 0; i < 10000; i++)
	//{
	//	dmx.readPositions();
	//	std::cout << "2:" << id2.position << ", 3:" << id3.position << "\n";
	//	//std::cout << "2:" << id2.last_position << ", 3:" << id3.last_position << "\n";
	//	Sleep(100);
	//}
	//dmx.close();

 //   std::cout << "Hello World!\n"; 


	RX48 id4(4);
	dmx.addDevice(&id4);

	dmx.open();
	dmx.ping(id4.id);
	dmx.enableTorque(false);
	dmx.enableTorque(false);
	dmx.setWheelMode();
	dmx.readPositions();
	dmx.setVelocities();

	dmx.enableTorque(true);
	for (int i = 0; i < 10000; i++)
	{
		dmx.readPositions();
		//std::cout << "2:" << id2.position << ", 3:" << id3.position << "\n";
		//std::cout << "2:" << id2.last_position << ", 3:" << id3.last_position << "\n";
		std::cout << "4:" << id4.position << "\n";
		Sleep(100);
	}
	dmx.close();

	std::cout << "Hello World!\n";
}



