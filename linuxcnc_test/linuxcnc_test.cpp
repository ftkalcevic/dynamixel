// linuxcnc_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "Dynamixel.h"
#include "RX48.h"
#include "linuxcnc_test.h"

int main()
{
	Dynamixel dmx("\\\\.\\COM21",3000000);

	RX48 id2(2);
	RX48 id3(3);
	dmx.addDevice(&id2);
	dmx.addDevice(&id3);

	dmx.open();
	dmx.enableTorque(false);
	dmx.setWheelMode();
	dmx.readPositions();
	id2.velocity = 50;
	id3.velocity = 1024 | 10;
	dmx.setVelocities();
	dmx.enableTorque(true);
	for (int i = 0; i < 10000; i++)
	{
		dmx.readPositions();
		std::cout << "2:" << id2.position << ", 3:" << id3.position << "\n";
		//std::cout << "2:" << id2.last_position << ", 3:" << id3.last_position << "\n";
		Sleep(100);
	}
	dmx.close();

    std::cout << "Hello World!\n"; 
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
