// Implementation of Dynamixel protocol 1.0
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "serial.h"

struct EEpromRegisters
{
	uint16_t ModelNumber;			// 0
	uint8_t FirmwareVersion;		// 2
	uint8_t ID;						// 3
	uint8_t BaudRate;				// 4
	uint8_t ReturnDelayTime;		// 5
	uint16_t CWAngleLimit;			// =6
	uint16_t CCWAngleLimit;			// =8
	uint8_t pad1;
	uint8_t TemperatureLimit;		// 11
	uint8_t MinVoltageLimit;		// 12
	uint8_t MaxVoltageLimit;		// 13
	uint16_t MaxTorque;				// 14
	uint8_t StatusReturnLevel;		// 16
	uint8_t AlarmLED;				// 17
	uint8_t Shutdown;				// 18
	uint8_t pad2;
	int16_t MultiTurnOffset;		// 20
	uint8_t ResolutionDivider;		// 22
	uint8_t EepromCRC;				// 23
};

struct ControlRegisters
{
	uint8_t TorqueEnable;		// 24	
	uint8_t LEDStatus;			// 25	
	uint8_t DGain;				// 26	
	uint8_t IGain;				// 27	
	uint8_t PGain;				// 28	
	uint8_t pad3;
	uint16_t GoalPosition;		// 30	
	uint16_t MovingSpeed;		// 32	
	uint16_t TorqueLimit;		// 34	
	uint16_t PresentPosition;	// 36	
	uint16_t PresentSpeed;		// 38	
	uint16_t PresentLoad;		// 40	
	uint8_t PresentVoltage;		// 42	
	uint8_t PresentTemperature;	// 43	
	uint8_t Registered;			// 44	
	uint8_t pad4;
	uint8_t Moving;				// 46	
	uint8_t Lock;				// 47	
	uint16_t Punch;				// 48	
	uint16_t RealtimeTick;		// 50	
	uint8_t pad5[21];
	uint8_t GoalAcceleration;	// 73	
};

struct Registers
{
	union 
	{
		struct 
		{
			EEpromRegisters e;
			ControlRegisters c;
		};
		uint8_t r[0];
	};
};

class Protocol1
{
	static const int8_t REPLY_NONE = -1;
	static const int8_t REPLY_NOW = 0;

	static const uint8_t BYTE_TIMEOUT = 100;	// 100 ms
	static const uint8_t REGISTERS = 74;		// Number of registers

	static const uint8_t HEADER_BYTE1 = 0xFF;
	static const uint8_t HEADER_BYTE2 = 0xFF;
	static const uint8_t BROADCAST_ID = 0xFE;

	static const uint8_t INS_PING			= 0x01;
	static const uint8_t INS_READ			= 0x02;
	static const uint8_t INS_WRITE			= 0x03;
	static const uint8_t INS_REG_WRITE		= 0x04;
	static const uint8_t INS_ACTION			= 0x05;
	static const uint8_t INS_FACTORY_RESET	= 0x06;
	static const uint8_t INS_REBOOT			= 0x08;
	static const uint8_t INS_SYNC_WRITE		= 0x83;
	static const uint8_t INS_BULK_READ		= 0x92;
	

	static const uint8_t ERROR_INSTRUCTION	= 0b01000000;
	static const uint8_t ERROR_OVERLOAD		= 0b00100000;
	static const uint8_t ERROR_CHECKSUM		= 0b00010000;
	static const uint8_t ERROR_RANGE		= 0b00001000;
	static const uint8_t ERROR_OVERHEAT	    = 0b00000100;
	static const uint8_t ERROR_ANGLE_LIMIT	= 0b00000010;
	static const uint8_t ERROR_INPUT_VOLTAGE= 0b00000001;
	
	SerialBase &serial;
	
	enum State
	{
		ReadHeader1,
		ReadHeader2,
		ReadID,
		ReadLength,
		ReadInstruction,
		ReadData
	} state;
	uint8_t deviceId;
	uint8_t readId;
	uint8_t readLen;
	uint8_t readIns;
	uint8_t bytesRead;
	uint8_t chksum;
	uint8_t params[256];	// input params.
	uint32_t last_byte_tick;
	uint8_t skipMessages;
	
	uint8_t replyBuffer[255 + 4];	// output buffer
	uint16_t replyLength;
	uint8_t errorFlags;
	
public:
	Registers registers;
	Registers controlregisters;
	Registers control;
	
	Protocol1(SerialBase &serial, uint8_t deviceID)
		: serial(serial)
		, deviceId(deviceID)
	{
		last_byte_tick = 0;
		skipMessages = 0;
		errorFlags = 0;
		memset(&control, 0, sizeof(control));
		SetFactoryDefaults();
	}

	void Start()
	{
		state = State::ReadHeader1;
		serial.Start();
	}

	bool Process()
	{
		uint32_t tick = HAL_GetTick();	// ms tick
		
		while (serial.ReadDataAvailable())
		{
			if ( tick - last_byte_tick > BYTE_TIMEOUT )
				state = State::ReadHeader1;
			
			last_byte_tick = tick;
			uint8_t c = serial.ReadByte();
			switch (state)
			{
				case State::ReadHeader1:
					if (c == HEADER_BYTE1)
						state = State::ReadHeader2;
					break;
				case State::ReadHeader2:
					if (c == HEADER_BYTE2)
						state = State::ReadID;
					else
						state = State::ReadHeader1;
					break;
				case State::ReadID:
					readId = c;
					chksum = c;
					state = State::ReadLength;
					break;
				case State::ReadLength:
					readLen = c;	// Bytes to read after instruction (params+chksum)
					chksum += c;
					bytesRead = 0;
					state = State::ReadInstruction;
					break;
				case State::ReadInstruction:
					readIns = c;
					chksum += c;
					state = State::ReadData;
					break;
				case State::ReadData:
					if (bytesRead == readLen-2)
					{
						// done.  c is chksum
						chksum = ~chksum;
						
						// process packet.  
						if(skipMessages)
						{
							skipMessages--;
							if (skipMessages == 0)
							{
								serial.SendPacket(replyBuffer, replyLength);
								state = State::ReadHeader1;
							}
						}
						else if(readId == deviceId || readId == BROADCAST_ID)
						{
							// For us.
							int8_t reply = ProcessMessage(c != chksum);
							if (reply == REPLY_NONE)
							{
								state = State::ReadHeader1;
							}
							else if (reply == REPLY_NOW)
							{
								serial.SendPacket(replyBuffer, replyLength);
								state = State::ReadHeader1;
							}
							else 
							{
								skipMessages = reply;
							}
						}
						//ProcessPacket();
						// if readId = deviceid, process, delay, reply reply
						// if readid = broadcast, and ins = bulk read, reply after others in the list in the correct order.
						// if readid = broadcast, process
						state = State::ReadHeader1;
					}
					else
					{
						params[bytesRead++] = c;
						chksum += c;
					}
					break;
			}
			
		}
	}

	// reset message specific errors
	void clearMessageErrors()
	{
		errorFlags &= ~(ERROR_INSTRUCTION | ERROR_CHECKSUM | ERROR_RANGE | ERROR_ANGLE_LIMIT );
	}
	
	uint8_t getErrorStatus()
	{ 
		return errorFlags;
	}
	
	void setErrorStatus(uint8_t flag)
	{
		errorFlags |= flag;
	}
	
	uint8_t MakeChecksum(const uint8_t *buffer, uint8_t len)
	{
		uint8_t chksum = 0;
		for (int i = 0; i < len; i++)
			chksum += buffer[i];
		return ~chksum;
	}
	
	void MakeStatusReply(const uint8_t *buffer, uint8_t len )
	{
		replyBuffer[0] = HEADER_BYTE1;
		replyBuffer[1] = HEADER_BYTE1;
		replyBuffer[2] = deviceId;
		replyBuffer[3] = len+2;
		replyBuffer[4] = getErrorStatus();
		int idx = 5;
		uint8_t chksum = replyBuffer[2] + replyBuffer[3] + replyBuffer[4];
		for (uint8_t i = 0; i < len; i++, idx++)
		{
			uint8_t c = buffer[i];
			replyBuffer[idx] = c;
			chksum += c;
		}
		replyBuffer[idx] = ~chksum;
		replyLength = idx + 1;
	}
	
	int8_t ProcessMessage(bool chksumError)
	{
		clearMessageErrors();
		bool reply = (readId == deviceId);
	
		if (chksumError)
		{
			setErrorStatus(ERROR_CHECKSUM);
			if (reply)			
			{
				MakeStatusReply(NULL, 0);
				return REPLY_NOW;
			}
			return REPLY_NONE;
		}
		
		switch (readIns)
		{
			case INS_PING:
				if (reply)			
				{
					MakeStatusReply(NULL, 0);
					return REPLY_NOW;
				}
				return REPLY_NONE;

			case INS_BULK_READ:
			{
				// Bulk read.  Are we in the list?
				bool found = false;
				uint8_t len;
				uint8_t addr;
				uint8_t repliesToSkip;
				for (int i = 0; i < readLen; i++)
					if (params[1 + 3*i + 1] == deviceId)
					{
						len = params[1 + 3*i + 0];
						addr = params[1 + 3*i + 2];
						// TODO error check len and addr
						repliesToSkip = i;
						found = true;
						break;
					}
				if (!found)
					return REPLY_NONE;
				MakeStatusReply( registers.r + addr, len);
				return repliesToSkip;
			}
				
			case INS_READ:
			{
				if (reply)
				{
					uint8_t addr = params[0];
					uint8_t len = params[1];
					// TODO error check len and addr
					MakeStatusReply( registers.r + addr, len);
					return REPLY_NOW;
				}
				else
					return REPLY_NONE;
			}
				
			case INS_WRITE:
			{
				uint8_t addr = params[0];
				uint8_t len = readLen-2;
				// TODO error check len and addr
				// TODO Don't update eeprom when locked
				memcpy(registers.r + addr, params + 1, len);
				// TODO something with updated registers.
				if (reply)
				{
					MakeStatusReply(NULL,0);
					return REPLY_NOW;
				}
				else
					return REPLY_NONE;
			}
							
			case INS_SYNC_WRITE:
			{
				uint8_t addr = params[0];
				uint8_t len = params[1];
				// TODO error check len and addr
				
				// Was this for us?
				for(int i = 2 ; i < readLen ; i += len)
					if(params[i] == deviceId)
					{
						// TODO Don't update eeprom when locked
						memcpy(registers.r + addr, params + i + 1, len);
						if (reply)
						{
							MakeStatusReply(NULL,0);
							return REPLY_NOW;
						}
						break;
					}
				// TODO something with updated registers.
				return REPLY_NONE;
			}
				
			case INS_REG_WRITE:
			{
				uint8_t addr = params[0];
				uint8_t len = readLen-2;
				// TODO error check len and addr
				memcpy(controlregisters.r + addr, params + 1, len);
				memset(control.r + addr, 1, len);
				control.c.Registered = 1;
				
				// TODO something with updated registers.
				if (reply)
				{
					MakeStatusReply(NULL,0);
					return REPLY_NOW;
				}
				else
					return REPLY_NONE;
			}
				
			case INS_ACTION:
			{
				// TODO Don't update eeprom when locked
				if(!control.c.Registered)
				{
					setErrorStatus(ERROR_INSTRUCTION);
				}
				else
				{
					control.c.Registered = 0;
					for (int i = 0; i < sizeof(registers); i++)
						if (control.r[i])
						{
							registers.r[i] = controlregisters.r[i];
							control.r[i] = 0;
						}
				}

				// TODO something with updated registers.
				if (reply)
				{
					MakeStatusReply(NULL,0);
					return REPLY_NOW;
				}
				else
					return REPLY_NONE;
			}
				
			case INS_FACTORY_RESET:
				SetFactoryDefaults();
				
			case INS_REBOOT:
				// reply, wait, then reboot
				if(reply)
				{
					MakeStatusReply(NULL, 0);
					serial.SendPacket(replyBuffer, replyLength);
					while (serial.Transmitting())
						continue;
				}
				NVIC_SystemReset();
				break;
				
			default:
				if (reply)
				{
					setErrorStatus(ERROR_INSTRUCTION);
					MakeStatusReply(NULL, 0);
					return REPLY_NOW;
				}
		}
		return REPLY_NONE;
	}
	
	uint8_t CalcCRC(const uint8_t *buf, uint8_t len)
	{
		__HAL_CRC_DR_RESET(&hcrc);	// #define __HAL_CRC_DR_RESET(__HANDLE__)            (SET_BIT((__HANDLE__)->Instance->CR,CRC_CR_RESET))

		for(int i = 0; i < len; i++ )
		{
			hcrc.Instance->DR = buf[i];
		}
		
		return hcrc.Instance->DR & 0xFF;
	}
	
	void SetFactoryDefaults()
	{
		registers.e.ModelNumber = 0x8000+29;
		registers.e.FirmwareVersion = 7;
		registers.e.ID = 1;
		registers.e.BaudRate = 34;
		registers.e.ReturnDelayTime = 250;
		registers.e.CWAngleLimit = 0;
		registers.e.CCWAngleLimit = 0xFFF;
		registers.e.TemperatureLimit = 80;
		registers.e.MinVoltageLimit = 60;
		registers.e.MaxVoltageLimit = 160;
		registers.e.MaxTorque = 0x3ff;
		registers.e.StatusReturnLevel = 2;
		registers.e.AlarmLED = 36;
		registers.e.Shutdown = 36;
		registers.e.MultiTurnOffset = 0;
		registers.e.ResolutionDivider = 1;
		registers.e.EepromCRC = CalcCRC(registers.r, sizeof(registers.e)-1);
	}
	
	void ReadEEPROM()
	{
		
	}
};
