#pragma once
class Device
{
public:
	Device(uint8_t id);
	virtual ~Device();
    
    void SetVelocity( uint16_t v );
    void SetTorqueLimit( uint16_t t );
    void SetPosition( uint16_t p );
	void UpdatePosition(uint16_t position);
	void UpdateVelocity(uint16_t speed);
	void UpdateTorque(uint16_t torque);

	uint8_t id;
	uint16_t last_position;
	int32_t position_fb;
    uint16_t velocity_fb;
    uint16_t torque_fb;
	bool first;
    bool enable;

    uint16_t velocity_cmd;
    uint16_t torque_limit_cmd;
    uint16_t position_cmd;

    bool velocityChanged;
    bool torqueLimitChanged;
    bool positionChanged;

};

