#pragma once
class Device
{
public:
	Device(uint8_t id);
	virtual ~Device();

	uint8_t id;
	uint16_t velocity;
	uint16_t last_position;
	int32_t position;
	bool first;

	void UpdatePosition(uint16_t position);
};

