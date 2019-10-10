#include "pch.h"
#include "Device.h"


Device::Device(uint8_t id) : id(id)
{
	first = true;
    enable = 0;
    velocityChanged = false;
    torqueLimitChanged = false;
    positionChanged = false;
}

Device::~Device()
{
}

void Device::UpdatePosition(uint16_t pos)
{
	if (first)
	{
		position_fb = 0;
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

		position_fb += delta;
	}

	last_position = pos;
}

void Device::UpdateVelocity(uint16_t speed)
{
    velocity_fb = speed;
}

void Device::UpdateTorque(uint16_t torque)
{
    torque_fb = torque;
}

void Device::SetVelocity( uint16_t v )
{
    velocity_cmd = v;
    velocityChanged = true;
}

void Device::SetTorqueLimit( uint16_t t )
{
    torque_limit_cmd = t;
    torqueLimitChanged = true;
}

void Device::SetPosition( uint16_t p )
{
    position_cmd = p;
    positionChanged = true;
}


