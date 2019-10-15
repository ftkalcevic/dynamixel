#pragma  once

extern SPI_HandleTypeDef hspi1;

class EepromSPI25lc640
{
	enum EEPROMCmds
	{
		READ = 0b00000011,
		WRITE= 0b00000010,
		WREN = 0b00000110,
		WRDI = 0b00000100,
		RDSR = 0b00000101,
		WRSR = 0b00000001,
	};
	
	const uint32_t EEPROM_WIP = 0b00000001;

	enum State
	{
		Idle,
		EnableWrite,
		WaitForTransfer,
		WriteData,
		WaitForWriteComplete
	};
	State state, next_state;
	
	uint16_t eeAddress;
	uint8_t *eeData;
	uint16_t eeLen;
	uint8_t transmitBuffer[3 + 32];		// largest packet WR + 32 bytes of data
	uint8_t receiveBuffer[2];			
	
public:
	EepromSPI25lc640()
	{
		state = State::Idle;
	}

	bool busy() const
	{
		return state != State::Idle;
	}
	
	bool WriteAsync(uint16_t address, uint8_t *data, uint16_t len)
	{
		if (state != State::Idle)
			return false;
		
		eeAddress = address;
		eeData = data;
		eeLen = len;
		state = State::EnableWrite;
		return true;
	}


	void Async()
	{
		switch (state)
		{
			case State::Idle:
				break;
				
			case State::EnableWrite:
				// Enable write
				transmitBuffer[0] = EEPROMCmds::WREN;
				chipSelect(true);
				HAL_SPI_Transmit_DMA(&hspi1, transmitBuffer, 1);
				state = State::WaitForTransfer;
				next_state = State::WriteData;
				break;

			case State::WriteData:
			{
				chipSelect(false);
				transmitBuffer[0] = EEPROMCmds::WRITE;
				transmitBuffer[1] = (uint8_t)(eeAddress & 0xFF);
				uint8_t len = eeLen < 16 ? eeLen : 16;
				memcpy(transmitBuffer + 2, eeData, len);
				chipSelect(true);
				HAL_SPI_Transmit_DMA(&hspi1, transmitBuffer, 2 + len);
				eeData += len;
				eeAddress += len;
				eeLen -= len;
				state = State::WaitForTransfer;
				next_state = State::WaitForWriteComplete;
				receiveBuffer[1] = EEPROM_WIP;
				break;
			}
				
			case State::WaitForWriteComplete:
				chipSelect(false);
				if (receiveBuffer[1] & EEPROM_WIP)	// still busy
				{
					transmitBuffer[0] = EEPROMCmds::RDSR; 	// Read status register
					transmitBuffer[1] = 0;
					chipSelect(true);
					HAL_SPI_TransmitReceive_DMA(&hspi1, transmitBuffer, receiveBuffer, 2);
					state = State::WaitForTransfer;
					next_state = State::WaitForWriteComplete;
				}
				else
				{
					if (eeLen == 0)
						state = State::Idle;
					else
						state = State::EnableWrite;
				}
				break;
				
			case State::WaitForTransfer:
				if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
					state = next_state;
				break;
		}
	}

	void chipSelect(bool cs)
	{
		HAL_GPIO_WritePin(GPIO_SPI1_CS_GPIO_Port, GPIO_SPI1_CS_Pin, cs ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
	
	bool Write(uint16_t address, uint8_t *data, uint16_t len)
	{
		if (state != State::Idle)
			return false;
		
		for (int i = 0; i < len; i += 16)
		{
			// 32 bytes at a time (here we are assuming address starts on at 0)
			uint16_t datalen = 16;
			if (datalen + i > len)
				datalen  = len - i;

			// Enable write
			chipSelect(true);
			transmitBuffer[0] = EEPROMCmds::WREN;
			HAL_SPI_Transmit(&hspi1, transmitBuffer, 1, 1000);
			chipSelect(false);			
			
			// Write address and data
			transmitBuffer[0] = EEPROMCmds::WRITE;
			transmitBuffer[1] = (uint8_t)(i & 0xFF);
			memcpy(transmitBuffer + 2, data, datalen);
			chipSelect(true);			
			HAL_SPI_Transmit(&hspi1, transmitBuffer, datalen+2, 1000);
			chipSelect(false);			
			data += datalen;
		
			// Wait for write to complete
			bool busy = true;
			while (busy)
			{
				transmitBuffer[0] = EEPROMCmds::RDSR; 	// Read status register
				transmitBuffer[1] = 0;
				chipSelect(true);			
				HAL_SPI_TransmitReceive(&hspi1, transmitBuffer, receiveBuffer, 2, 1000);
				chipSelect(false);			
				busy = (receiveBuffer[1] & EEPROM_WIP) ? true : false;
			}
		}
	}

	void Read(uint16_t address, uint8_t *buffer, uint16_t len)
	{
		// read
		uint8_t buf[2] = { EEPROMCmds::READ, (uint8_t)(address & 0xFF) };
		chipSelect(true);
		HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), 1000);
		HAL_SPI_Receive(&hspi1, buffer, len, 1000);
		chipSelect(false);
	}

};