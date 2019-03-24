#include "pch.h"
#include "Device.h"


Device::Device(uint8_t id) : id(id)
{
	first = true;
    enable = 0;
}

Device::~Device()
{
}

void Device::UpdatePosition(uint16_t pos)
{
	if (first)
	{
		position = 0;
		first = false;
	}
	else
	{
		// 12bit pos value.		
		//	1	2		+1
		//  2   1		-1
		//  4095 0		+1
		//  0 4095		-1

		int16_t delta = pos - last_position;

		if (delta & 0x0800)
			delta = delta | 0xF000;
		else
			delta = delta & 0x0FFF;

		position += delta;
	}

	last_position = pos;
}
