#pragma once

template <int SCALE>
class PID
{
	int16_t Kp, Ki, Kd;
	int32_t i_error;
	int16_t last_error;
	
public:
	PID()
	{
		Kp = Ki = Kd = 0;
		clear();
}
	
	PID(int16_t Kp, int16_t Ki, int16_t Kd)
		: Kp(Kp), Ki(Ki), Kd(Kd)
	{
		clear();
	}
	
	void clear()
	{
		last_error = 0;
		i_error = 0;
	}
	
	int16_t Update( int16_t error )
	{
		int32_t d_error = error - last_error;
		i_error += error;
		last_error = error;
		
		int32_t output = (Kp*error + Kd*(d_error) + Ki*i_error) >> SCALE; 

		return (int16_t)output;
	}	
};