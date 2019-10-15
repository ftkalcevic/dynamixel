// Implementation of Dynamixel protocol 1.0
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "serial.h"
#include "eeprom_spi_25lc020.h"

// addresses of registers
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004; 
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC; 

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
	static const uint8_t EEPROM_REG_MAX = 24;	// Last eeprom register

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
		ReadData,
		SendReply
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
	uint16_t messageReceivedTimestamp;
	
	uint8_t replyBuffer[255 + 4];	// output buffer
	uint16_t replyLength;
	uint8_t errorFlags;
	EepromSPI25lc020 eeprom;


public:
	Registers registers;			// main set
	Registers controlregisters;		// holding set for reg_write
	Registers control;				// dirty flags for reg_write
	EEpromRegisters eepromWriteBuffer;
	bool eeprom_dirty;
	
	Protocol1(SerialBase &serial)
		: serial(serial)
	{
		last_byte_tick = 0;
		skipMessages = 0;
		errorFlags = 0;
		memset(&control, 0, sizeof(control));
		deviceId = 0;
		eeprom_dirty = false;
	}

	void Start()
	{
		ReadEEPROM();
		SetControlDefaults();
		state = State::ReadHeader1;
		StartSerial();
	}
	
	void StartSerial()
	{
		serial.Start(baudRate());
	}

	bool Process()
	{
		uint32_t tick = HAL_GetTick();	// ms tick
		
		while (true)
		{
			if ( tick - last_byte_tick > BYTE_TIMEOUT )
				state = State::ReadHeader1;
			
			// special handling of send reply, which relies on time events, not data
			if (state == State::SendReply && GetusTick() - messageReceivedTimestamp >= 2 * registers.e.ReturnDelayTime)
			{
				//printf("R");
				serial.SendPacket(replyBuffer, replyLength);
				state = State::ReadHeader1;
			}
			
			if (eeprom_dirty)
				WriteEEPROMAsync();
			eeprom.Async();
			
			if (serial.ReadDataAvailable())
			{
				//extern UART_HandleTypeDef huart1;
				last_byte_tick = tick;
				uint8_t c = serial.ReadByte();
				//printf("%02X ", c);
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
						if (c < 2)
						{
							state = State::ReadHeader1;
						}
						else
						{
							readLen = c; 	// Bytes to read after instruction (params+chksum)
							chksum += c;
							bytesRead = 0;
							state = State::ReadInstruction;
						}
					
						break;
					case State::ReadInstruction:
						readIns = c;
						chksum += c;
						state = State::ReadData;
						break;
					case State::ReadData:
						if (bytesRead == readLen - 2 )
						{
							messageReceivedTimestamp = GetusTick();
							// done.  c is chksum
							chksum = ~chksum;
						
							// process packet.  
							if(skipMessages)
							{
								skipMessages--;
								if (skipMessages == 0)
								{
									state = State::SendReply;
								}
								else
								{
									state = State::ReadHeader1;
								}
							}
							else if(readId == deviceId || readId == BROADCAST_ID)
							{
								// For us.
								//printf("M");
								int8_t reply = ProcessMessage(c != chksum);
								if (reply == REPLY_NONE)
								{
									state = State::ReadHeader1;
								}
								else if (reply == REPLY_NOW)
								{
									state = State::SendReply;
								}
								else 
								{
									skipMessages = reply;
									state = State::ReadHeader1;
								}
							}
							else
							{
								state = State::ReadHeader1;
							}
						}
						else
						{
							params[bytesRead++] = c;
							chksum += c;
						}
						break;
					case State::SendReply:
						state = State::ReadHeader1;
						break;
				}
			}
			else
			{
				break;
			}
		}
		return false;
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
						//printf("B");
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
				uint8_t len = readLen-3;
				// TODO error check len and addr
				CheckEepromLock(addr, len);
				memcpy(registers.r + addr, params + 1, len);
				// TODO something with updated registers.
				
				VerifyRegisters();

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
				for(int i = 2 ; i < readLen-2 ; i += len+1)
					if(params[i] == deviceId)
					{
						CheckEepromLock(addr, len);
						memcpy(registers.r + addr, params + i + 1, len);
						VerifyRegisters();
						//printf("S0%02X\n", addr);

						if (reply)
//						{
//							MakeStatusReply(NULL,0);
//							return REPLY_NOW;
//						}
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
				registers.c.Registered = 1;
				VerifyRegisters();
				
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
				if(!registers.c.Registered)
				{
					setErrorStatus(ERROR_INSTRUCTION);
				}
				else
				{
					registers.c.Registered = 0;
					for (int i = 0; i < sizeof(registers); i++)
						if (control.r[i])
						{
							if (i < EEPROM_REG_MAX)
							{
								if (!registers.c.TorqueEnable)	// can only update if torque not enabled
								{
									registers.r[i] = controlregisters.r[i];
									eeprom_dirty = true;
								}
							}
							else
							{
								registers.r[i] = controlregisters.r[i];
							}
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
					while (GetusTick() - messageReceivedTimestamp >= 2 * registers.e.ReturnDelayTime)
						continue;
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
	
	void VerifyRegisters()
	{
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
		registers.e.ModelNumber = 29;//0x8000+29;
		registers.e.FirmwareVersion = 36;
		registers.e.ID = 1;
		registers.e.BaudRate = 34;
		registers.e.ReturnDelayTime = 0;
		registers.e.CWAngleLimit = 400;
		registers.e.CCWAngleLimit = 260;
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

//#ifdef DEBUG
//		// override hacks
//		registers.e.ModelNumber = 29; // fake mx-28 so Dynamixel Wizard works
//		registers.e.BaudRate = 252; // 3M
//#endif
	}
	
	void SetControlDefaults()
	{
		registers.c.TorqueEnable = 0;		
		registers.c.LEDStatus = 0;			
		registers.c.DGain = 0;				
		registers.c.IGain = 0;				
		registers.c.PGain = 32;				
		registers.c.GoalPosition = 0;		
		registers.c.MovingSpeed = 0;		
		registers.c.TorqueLimit = registers.e.MaxTorque;		
		registers.c.PresentPosition = 0;	
		registers.c.PresentSpeed = 0;		
		registers.c.PresentLoad = 0;		
		registers.c.PresentVoltage = 0;		
		registers.c.PresentTemperature = 0;	
		registers.c.Registered = 0;			
		registers.c.Moving = 0;				
		registers.c.Lock = 0;				
		registers.c.Punch = 0;				
		registers.c.RealtimeTick = 0;		
		registers.c.GoalAcceleration = 0;	
		
		
		registers.c.TorqueLimit = 50;		
	}
	
	void WriteEEPROM()
	{
		registers.e.EepromCRC = CalcCRC((const uint8_t *)&(registers.e), sizeof(registers.e) - 1);
		eeprom.Write( 0, (uint8_t *)&(registers.e), sizeof(registers.e));

////*DEMCR = *DEMCR | 0x01000000;		// enable the use DWT
////*DWT_CYCCNT = 0;					// Reset cycle counter
////*DWT_CONTROL = *DWT_CONTROL | 1 ;	// enable cycle counter
//
//
//		registers.e.EepromCRC = CalcCRC((const uint8_t *)&(registers.e), sizeof(registers.e) - 1);
//		
//		uint32_t addr = (uint32_t)emulatedEeprom;
//		HAL_FLASH_Unlock();
//		
//		FLASH_PageErase(addr);
//        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
//		for (uint16_t i = 0; i < sizeof(registers.e); i += 2)
//			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,addr + i, *(uint16_t *)(registers.r + i));
//		HAL_FLASH_Lock();
//		eeprom_dirty = false;
//		
////// number of cycles stored in count variable
////int count = *DWT_CYCCNT;
////		printf("WriteEEPROM=%d\n", count);		
//		
	}
	
	
	void WriteEEPROMAsync()
	{
		if (eeprom_dirty && !eeprom.busy())
		{
			registers.e.EepromCRC = CalcCRC((const uint8_t *)&(registers.e), sizeof(registers.e) - 1);
			memcpy(&eepromWriteBuffer, &(registers.e), sizeof(registers.e));
			if (eeprom.WriteAsync(0, (uint8_t *)&(eepromWriteBuffer), sizeof(eepromWriteBuffer)))
			{
				eeprom_dirty = false;
			}
		}
	}

	void ReadEEPROM()
	{
////#ifdef DEBUG
//		// override hacks
//		SetFactoryDefaults();
//		registers.e.ModelNumber = 29; // fake mx-28 so Dynamixel Wizard works
//		registers.e.BaudRate = 252; // 3M
//		registers.e.ID = 4;
//		deviceId = registers.e.ID;
//		return;
////#endif		
		eeprom.Read( 0, (uint8_t *)&(registers.e), sizeof(registers.e));
		uint8_t crc = CalcCRC((const uint8_t *)&(registers.e), sizeof(registers.e) - 1);
		if (crc != registers.e.EepromCRC)
		{
			SetFactoryDefaults();
			WriteEEPROM();
		}
		deviceId = registers.e.ID;
	}
	
	uint32_t baudRate()
	{
		// some of the baud rates
		switch (registers.e.BaudRate) 
		{
			case 0:		return 2000000;
			case 1:		return 1000000;
			case 3:		return 500000;
			case 4:		return 400000;
			case 7:		return 250000;
			case 9:		return 200000;
			case 15:	return 125000;
			case 16:	return 115200;
			case 34:	return 57600;
			case 103:	return 19200;
			case 207:	return 9600;
			case 250:	return 2250000;
			case 251:	return 2500000;
			case 252:	return 3000000;
			default:	return 57600;
		}
	}	
	
	// Don't update eeprom when locked
	void CheckEepromLock(uint8_t &addr, uint8_t &len)
	{
		if(addr < EEPROM_REG_MAX)
		{
			if (registers.c.TorqueEnable)
			{
				// Can't update
				int newLen = len;
				newLen -= EEPROM_REG_MAX - addr;
				addr += EEPROM_REG_MAX - addr;
				if (newLen <= 0)
					len = 0;
				else
					len = (uint8_t)newLen;
			}
			else
			{
				eeprom_dirty = true;
			}
		}
	}

};


// TODO
// Don't use HAL for serial TX.  There's too big an overhead setting up and handling irrelevant scenarios.
// error handling
// 