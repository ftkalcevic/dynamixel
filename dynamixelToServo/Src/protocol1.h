// Implementation of Dynamixel protocol 1.0
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "serial.h"

enum EEPROMAddress : uint8_t 
{
	ModelNumber=0,			// 2
	FirmwareVersion=2,		// 1
	ID=3,					// 1
	BaudRate=4,				// 1
	ReturnDelayTime=5,		// 1
	CWAngleLimit=6,			// 2
	CCWAngleLimit=8,		// 2
	TemperatureLimit=11,	// 1	
	MinVoltageLimit=12,		// 1	
	MaxVoltageLimit=13,		// 1	
	MaxTorque=14,			// 2	
	StatusReturnLevel=16,	// 1	
	AlarmLED=17,			// 1	
	Shutdown=18,			// 1	
	MultiTurnOffset=20,		// 2	
	ResolutionDivider=22,	// 1	
	EepromCRC=23,			// 1	
};

enum ControlAddress : uint8_t 
{
	TorqueEnable		=24, //	1	
	LEDStatus			=25, //	1	
	DGain				=26, //	1	
	IGain				=27, //	1	
	PGain				=28, //	1	
	GoalPosition		=30, //	2	
	MovingSpeed			=32, //	2	
	TorqueLimit			=34, //	2	
	PresentPosition		=36, //	2	
	PresentSpeed		=38, //	2	
	PresentLoad			=40, //	2	
	PresentVoltage		=42, //	1	
	PresentTemperature	=43, //	1	
	Registered			=44, //	1	
	Moving				=46, //	1	
	Lock				=47, //	1	
	Punch				=48, //	2	
	RealtimeTick		=50, //	2	
	GoalAcceleration	=73, //	1	
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
	uint8_t registers[REGISTERS];
	uint8_t controlregisters[REGISTERS];
	uint8_t control[REGISTERS];
	
	Protocol1(SerialBase &serial, uint8_t deviceID)
		: serial(serial)
		, deviceId(deviceID)
	{
		last_byte_tick = 0;
		skipMessages = 0;
		errorFlags = 0;
		memset(control, 0, sizeof(control));
		
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
				MakeStatusReply( registers + addr, len);
				return repliesToSkip;
			}
				
			case INS_READ:
			{
				if (reply)
				{
					uint8_t addr = params[0];
					uint8_t len = params[1];
					// TODO error check len and addr
					MakeStatusReply( registers + addr, len);
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
				memcpy(registers + addr, params + 1, len);
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
						memcpy(registers + addr, params + i + 1, len);
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
				memcpy(controlregisters + addr, params + 1, len);
				memset(control + addr, 1, len);
				
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
				int writes = 0;
				for (int i = 0; i < sizeof(registers); i++)
					if (control[i])
					{
						registers[i] = controlregisters[i];
						control[i] = 0;
						writes++;
					}
				if (!writes)
					setErrorStatus(ERROR_INSTRUCTION);

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
		registers[EEPROMAddress::ModelNumber] = 29;			// 2
		registers[EEPROMAddress::ModelNumber+1] = 0x80;		// 2
		registers[EEPROMAddress::FirmwareVersion] = 7;		// 1
		registers[EEPROMAddress::ID] = 1;					// 1
		registers[EEPROMAddress::BaudRate] = 34;			// 1
		registers[EEPROMAddress::ReturnDelayTime] = 250;	// 1
		registers[EEPROMAddress::CWAngleLimit] = 0;			// 2
		registers[EEPROMAddress::CWAngleLimit+1] = 0;		// 2
		registers[EEPROMAddress::CCWAngleLimit] = 0xFF;		// 2
		registers[EEPROMAddress::CCWAngleLimit+1] = 0x0F;	// 2
		registers[EEPROMAddress::TemperatureLimit] = 80;	// 1	
		registers[EEPROMAddress::MinVoltageLimit] = 60;		// 1	
		registers[EEPROMAddress::MaxVoltageLimit] = 160;	// 1	
		registers[EEPROMAddress::MaxTorque] = 0xff;			// 2	
		registers[EEPROMAddress::MaxTorque+1] = 0x03;		// 2	
		registers[EEPROMAddress::StatusReturnLevel] = 2;	// 1	
		registers[EEPROMAddress::AlarmLED] = 36;			// 1	
		registers[EEPROMAddress::Shutdown] = 36;			// 1	
		registers[EEPROMAddress::MultiTurnOffset] = 0;		// 2	
		registers[EEPROMAddress::MultiTurnOffset+1] = 0;	// 2	
		registers[EEPROMAddress::ResolutionDivider] = 1;	// 1		
		registers[EEPROMAddress::EepromCRC] = CalcCRC(registers, EEPROMAddress::ResolutionDivider);	// 1		
	}
	
	void ReadEEPROM()
	{
		
	}
};
