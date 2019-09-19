// linuxcnc_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "Dynamixel.h"
#include "RX28.h"
#include "linuxcnc_test.h"

int main()
{
#if defined(_WIN32) || defined(_WIN64)
	Dynamixel dmx("\\\\.\\COM21",3000000);
#else
        Dynamixel dmx("/dev/ttyUSB0",3000000);
#endif

	//RX28 id2(2);
	//RX28 id3(3);
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


	RX28 id4(4);
	dmx.addDevice(&id4);

    if ( !dmx.open() )
	{
        return -1;
    }
	dmx.ping(id4.id);
    dmx.enableTorque();
    dmx.setWheelMode();
    dmx.readPositions();
    dmx.setVelocities();
    //id2.enable = true;
	//id3.enable = true;
	id4.enable = true;
    dmx.enableTorque();
    for (int i = 0; i < 10000; i++)
	{
		dmx.readPositions();
		//std::cout << "2:" << id2.position << ", 3:" << id3.position << "\n";
        //std::cout << "2:" << id2.last_position << ", 3:" << id3.last_position << "\n";
		std::cout << "4:" << id4.position << "\n";
        //dmx.delayms(100);
        dmx.setVelocities();
    }
 //   id2.enable = false;
	//id3.enable = false;
	id4.enable = false;
    dmx.enableTorque();
	dmx.close();

    std::cerr << "Run done!\n";
    std::cerr << "Checksum Errors: " << dmx.checksumErrors << "\n";
    std::cerr << "Timeout Errors: " << dmx.timeoutErrors << "\n";
    std::cerr << "Data Errors: " << dmx.dataErrors << "\n";
}



