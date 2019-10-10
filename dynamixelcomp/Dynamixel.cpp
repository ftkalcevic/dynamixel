#include "pch.h"
#include "Dynamixel.h"

#if defined(_WIN32) || defined(_WIN64)
#else

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#endif

static const uint8_t H1 = 0;
static const uint8_t H2 = 1;
static const uint8_t ID = 2;
static const uint8_t LEN = 3;
static const uint8_t INS = 4;
static const uint8_t SYNC_WRITE_ADDR = 5;
static const uint8_t SYNC_WRITE_LEN = 6;


static const uint8_t HEADER1 = 0xFF;
static const uint8_t HEADER2 = 0xFF;
static const uint8_t BROADCAST_ID = 0xFE;
static const uint8_t SYNC_WRITE = 0x83;
static const uint8_t BULK_READ = 0x92;

static const uint8_t CW_ANGLE_LIMIT = 6;
static const uint8_t LED_DEVICE = 25;
static const uint8_t GOAL_POSITION = 30;
static const uint8_t MOVING_SPEED = 32;
static const uint8_t TORQUE_LIMIT = 34;
static const uint8_t PRESENT_POSITION = 36;
static const uint8_t TORQUE_ENABLE = 24;
static const uint8_t GOAL_ACCELERATION = 73;

static const uint16_t MAX_PACKET_LEN = 257;

Dynamixel::Dynamixel(std::string portName, int baud) : baud(baud), portName(portName)
{
	hPort = INVALID_HANDLE_VALUE;
	tx_time_per_byte = 0;
	reply_delay = 0;
	checksumErrors = 0;
	timeoutErrors = 0;
    dataErrors = 0;
	packetBuilt = false;
	readPositionPacket = NULL;
	setVelocityPacket = NULL;
	setTorqueLimitPacket = NULL;
	setPositionPacket = NULL;
	setLEDPacket = NULL;

	initTimer();
}


Dynamixel::~Dynamixel()
{
	close();
}

void Dynamixel::addDevice(Device *device)
{
	devices.push_back(device);
	devicesById[device->id] = device;
	packetBuilt = false;
}

void Dynamixel::enableTorque(Device *device, bool enable )
{
	const int PacketLen = 5 + 2 + 2 + 1;
	uint8_t Packet[MAX_PACKET_LEN];
	Packet[H1] = HEADER1;
	Packet[H2] = HEADER2;
	Packet[ID] = BROADCAST_ID;
	Packet[LEN] = PacketLen - 4;
	Packet[INS] = SYNC_WRITE;
	Packet[SYNC_WRITE_ADDR] = TORQUE_ENABLE;
	Packet[SYNC_WRITE_LEN] = 1;

	uint8_t *data = Packet + SYNC_WRITE_LEN + 1;
    *(data++) = device->id;
    *(data++) = enable ? 1 : 0;
	Packet[PacketLen - 1] = CalcChecksum(Packet + ID, PacketLen - 3);

	// Send the packet.  There is no reply because we are sending to the broadcast address. 
	write(Packet, PacketLen);
}

void Dynamixel::goalAcceleration(uint8_t acc)
{
	const int PacketLen = 5 + 2 + 2 * (uint8_t)devices.size() + 1;
	uint8_t Packet[MAX_PACKET_LEN];
	Packet[H1] = HEADER1;
	Packet[H2] = HEADER2;
	Packet[ID] = BROADCAST_ID;
	Packet[LEN] = PacketLen - 4;
	Packet[INS] = SYNC_WRITE;
	Packet[SYNC_WRITE_ADDR] = GOAL_ACCELERATION;
	Packet[SYNC_WRITE_LEN] = 1;

	uint8_t *data = Packet + SYNC_WRITE_LEN + 1;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
		*(data++) = devices[i]->id;
        *(data++) = acc;
	}
	Packet[PacketLen - 1] = CalcChecksum(Packet + ID, PacketLen - 3);

	// Send the packet.  There is no reply because we are sending to the broadcast address. 
	write(Packet, PacketLen);

}

void Dynamixel::setRotationLimits(uint8_t id, uint16_t cw, uint16_t ccw)
{
	const int PacketLen = 5 + 2 + 5 + 1;
	uint8_t Packet[MAX_PACKET_LEN];
	Packet[H1] = HEADER1;
	Packet[H2] = HEADER2;
	Packet[ID] = BROADCAST_ID;
	Packet[LEN] = PacketLen - 4;
	Packet[INS] = SYNC_WRITE;
	Packet[SYNC_WRITE_ADDR] = CW_ANGLE_LIMIT;
	Packet[SYNC_WRITE_LEN] = 4;

	uint8_t *data = Packet + SYNC_WRITE_LEN + 1;
    *(data++) = id;
    *(data++) = cw & 0xFF;
    *(data++) = (cw >> 8) & 0xFF;
    *(data++) = ccw & 0xFF;
    *(data++) = (ccw >> 8) & 0xFF;
	Packet[PacketLen - 1] = CalcChecksum(Packet + ID, PacketLen - 3);

	// Send the packet.  There is no reply because we are sending to the broadcast address. 
	write(Packet, PacketLen);
}


void Dynamixel::setMode(Device *device, DeviceMode  mode)
{
    if ( mode == Joint )
    {
        setRotationLimits( device->id, device->cwLimit, device->ccwLimit );
    }
    else if ( mode == Wheel )
    {
        setRotationLimits( device->id, 0, 0 );
    }
    else if ( mode == MultiTurn )
    {}
}

void Dynamixel::Enable()
{
	for (unsigned int i = 0; i < devices.size(); i++)
        if ( devices[i]->enableChanged )
        {
            devices[i]->enableChanged = false;
            if ( devices[i]->enable )
            {
                setMode( devices[i], devices[i]->mode );
                delayus(10);
            }
            enableTorque(devices[i], devices[i]->enable );
        }
}


void Dynamixel::readPositions()
{
	if (!packetBuilt)
		buildPackets();

	// Send the read packet.
	write(readPositionPacket, readPositionPacketLen);

	// Wait for the replies from each device.
	BlockingReadPositions();
}

void Dynamixel::BlockingReadPositions()
{
    uint8_t buffer[4096];

	readPositionsState = WaitForHeader;
	devicesToRead = (uint8_t)devices.size();

	// Dumb loop
#define OneMS 0.001
	double timeoutDouble = (OneMS + readPositionPacketLen * tx_time_per_byte + 10 * tx_time_per_byte + OneMS);
	int64_t timePeriod = (int64_t)(timeoutDouble * timerFrequency);
    int64_t timeout = getTimer() + timePeriod;
    int loopcount=0;
	while (true)
	{
        int ba = bytesAvailable();
        if (ba > 0)
		{
            int bytesRead = read(buffer, ba);
			if (ProcessReadPositions(buffer, bytesRead))
				break;
            timeout = getTimer() + timePeriod;
		}
        int64_t t = getTimer();
        if (t > timeout)
        {
            timeoutErrors++;
            break;
        }

        delayus(250);	// sleep 1/4 ms
        loopcount++;
	}
}

bool Dynamixel::ProcessReadPositions(uint8_t *buffer, int len)
{
	for (int i = 0; i < len; i++)
	{
		uint8_t c = buffer[i];
		switch (readPositionsState)
		{
			case EReadPositionsState::WaitForHeader: 
				if (devicesToRead == 0)
					break;
				if (c == 0xFF)
					readPositionsState = EReadPositionsState::WaitForHeader2;
				break;
			case EReadPositionsState::WaitForHeader2:
				if (c == 0xFF)
					readPositionsState = EReadPositionsState::WaitForID;
				break;
			case EReadPositionsState::WaitForID:
				readId = c;
				readChksum = c;
				readPositionsState = EReadPositionsState::WaitForLen;
				break;
			case EReadPositionsState::WaitForLen:
				readLen = c-1;	// exclude chksum
				readChksum += c;
				readBytes = 0;
				readPositionsState = EReadPositionsState::WaitForData;
				break;
			case EReadPositionsState::WaitForData:
				if (readLen == readBytes)
				{
					readChksum = ~readChksum;

					if (readChksum != c)
					{
						checksumErrors++;
					}
					else
					{
						ProcessPosition(readId, readBuffer, readLen);
					}
					readPositionsState = EReadPositionsState::WaitForHeader;
					devicesToRead--;
					if (devicesToRead == 0)
						return true;
					break;
				}
					
				readBuffer[readBytes] = c;
				readChksum += c;
				readBytes++;

				break;
		}
	}
	return false;
}

void Dynamixel::ProcessPosition(uint8_t id, uint8_t *buffer, uint8_t len)
{
	// position, speed, torque comes back in a status packet

    Device *dev = devicesById[id];
    if ( !dev )
    {
        dataErrors++;    
        return;
    }
    if ( len >= 1 )
    {
        uint8_t err = buffer[0];
        //if ( err != 0 )
        //    dataErrors++;
    }
    if ( len >= 3 )
    {
	    uint16_t position = buffer[1] | (buffer[2] << 8);
        dev->UpdatePosition(position);
    }
    if ( len >= 5 )
    {
	    uint16_t speed = buffer[3] | (buffer[4] << 8);
        dev->UpdateVelocity(speed);
    }
    if ( len >= 7 )
    {
	    uint16_t torque = buffer[5] | (buffer[6] << 8);
        dev->UpdateTorque(torque);
    }
}

void Dynamixel::Update()
{
    setTorqueLimits();
    setVelocities();
    setPositions();
    setLEDs();
}

void Dynamixel::setTorqueLimits()
{
	if (!packetBuilt)
		buildPackets();

	uint8_t offset = SYNC_WRITE_LEN+1;
    uint8_t changes = 0;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
        if ( devices[i]->torqueLimitChanged )
        {
            setTorqueLimitPacket[offset + 0] = devices[i]->id;
            setTorqueLimitPacket[offset + 1] = devices[i]->torque_limit_cmd & 0xFF;	// LSB
            setTorqueLimitPacket[offset + 2] = (devices[i]->torque_limit_cmd >> 8) & 0xFF;	// MSB
            devices[i]->torqueLimitChanged = false;
            offset += 3;
            changes++;
        }
	}
    if ( changes )
    {
        printf("torque limit changes %d\n", changes );
        setTorqueLimitPacketLen = 5 + 2 + 3 * changes + 1;
        setTorqueLimitPacket[LEN] = setTorqueLimitPacketLen - 4;
	    setTorqueLimitPacket[setTorqueLimitPacketLen-1] = CalcChecksum(setTorqueLimitPacket + ID, setTorqueLimitPacketLen - 3);

	    // Send the packet.  There is no reply because we are sending to the broadcast address. 
	    write(setTorqueLimitPacket, setTorqueLimitPacketLen);

	    // Need to make sure we don't send another packet until this one is sent
    }
}

void Dynamixel::setPositions()
{
	if (!packetBuilt)
		buildPackets();

	uint8_t offset = SYNC_WRITE_LEN+1;
    uint8_t changes = 0;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
        if ( devices[i]->positionChanged )
        {
            setPositionPacket[offset + 0] = devices[i]->id;
            setPositionPacket[offset + 1] = devices[i]->position_cmd & 0xFF;	// LSB
            setPositionPacket[offset + 2] = (devices[i]->position_cmd >> 8) & 0xFF;	// MSB
            devices[i]->positionChanged = false;
            offset += 3;
            changes++;
        }
	}
    if ( changes )
    {
        printf("position changes %d\n", changes );
        setPositionPacketLen = 5 + 2 + 3 * changes + 1;
        setPositionPacket[LEN] = setPositionPacketLen - 4;
	    setPositionPacket[setPositionPacketLen-1] = CalcChecksum(setPositionPacket + ID, setPositionPacketLen - 3);

	    // Send the packet.  There is no reply because we are sending to the broadcast address. 
	    write(setPositionPacket, setPositionPacketLen);

	    // Need to make sure we don't send another packet until this one is sent
    }
}

void Dynamixel::setVelocities()
{
	if (!packetBuilt)
		buildPackets();

	uint8_t offset = SYNC_WRITE_LEN+1;
    uint8_t changes = 0;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
        if ( devices[i]->velocityChanged )
        {
            setVelocityPacket[offset + 0] = devices[i]->id;
            setVelocityPacket[offset + 1] = devices[i]->velocity_cmd & 0xFF;	// LSB
            setVelocityPacket[offset + 2] = (devices[i]->velocity_cmd >> 8) & 0xFF;	// MSB
            devices[i]->velocityChanged = false;
            offset += 3;
            changes++;
        }
	}
    if ( changes )
    {
        //printf("velocities changes %d\n", changes );
        setVelocityPacketLen = 5 + 2 + 3 * changes + 1;
        setVelocityPacket[LEN] = setVelocityPacketLen - 4;
	    setVelocityPacket[setVelocityPacketLen-1] = CalcChecksum(setVelocityPacket + ID, setVelocityPacketLen - 3);

	    // Send the velocity packet.  There is no reply because we are sending to the broadcast address. 
	    write(setVelocityPacket, setVelocityPacketLen);

	    // Need to make sure we don't send another packet until this one is sent
    }
}

void Dynamixel::setLEDs()
{
	if (!packetBuilt)
		buildPackets();

	uint8_t offset = SYNC_WRITE_LEN+1;
    uint8_t changes = 0;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
        if ( devices[i]->ledChanged )
        {
            setLEDPacket[offset + 0] = devices[i]->id;
            setLEDPacket[offset + 1] = devices[i]->led ? 1 : 0;
            devices[i]->ledChanged = false;
            offset += 2;
            changes++;
        }
	}
    if ( changes )
    {
        setLEDPacketLen = 5 + 2 + 2 * changes + 1;
        setLEDPacket[LEN] = setLEDPacketLen - 4;
	    setLEDPacket[setLEDPacketLen-1] = CalcChecksum(setLEDPacket + ID, setLEDPacketLen - 3);

	    // Send the packet.  There is no reply because we are sending to the broadcast address. 
	    write(setLEDPacket, setLEDPacketLen);

	    // Need to make sure we don't send another packet until this one is sent
    }
}

void Dynamixel::buildPackets()
{
	uint8_t *data;

	if (readPositionPacket)
		delete readPositionPacket;

	readPositionPacketLen = 5 + 1 + 3 * (uint8_t)devices.size() + 1;
	readPositionPacket = new uint8_t[readPositionPacketLen];
	readPositionPacket[H1] = HEADER1;
	readPositionPacket[H2] = HEADER2;
	readPositionPacket[ID] = BROADCAST_ID;
	readPositionPacket[LEN] = readPositionPacketLen - 4;
	readPositionPacket[INS] = BULK_READ;

	data = readPositionPacket + INS + 1;
	*(data++) = 0;

	for (unsigned int i = 0; i < devices.size(); i++)
	{
		*(data++) = 6;	// Length of data to read - 2 bytes position, 2 bytes speed, 2 bytes load
		*(data++) = devices[i]->id;
		*(data++) = PRESENT_POSITION;
		//*(data++) = (i == 0 ? 30 : 36);
	}
	*data = CalcChecksum(readPositionPacket + ID, readPositionPacketLen - 3);

	if (setVelocityPacket)
		delete setVelocityPacket;

	setVelocityPacketLen = 5 + 2 + 3 * (uint8_t)devices.size() + 1;
	setVelocityPacket = new uint8_t[setVelocityPacketLen];
	setVelocityPacket[H1] = HEADER1;
	setVelocityPacket[H2] = HEADER2;
	setVelocityPacket[ID] = BROADCAST_ID;
	setVelocityPacket[LEN] = setVelocityPacketLen - 4;
	setVelocityPacket[INS] = SYNC_WRITE;
	setVelocityPacket[SYNC_WRITE_ADDR] = MOVING_SPEED;
	setVelocityPacket[SYNC_WRITE_LEN] = 2;

	data = setVelocityPacket + SYNC_WRITE_LEN + 1;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
		*(data++) = devices[i]->id;
		*(data++) = 0;	// LSB
		*(data++) = 0;	// MSB
	}

	if (setTorqueLimitPacket)
		delete setTorqueLimitPacket;

	setTorqueLimitPacketLen = 5 + 2 + 3 * (uint8_t)devices.size() + 1;
	setTorqueLimitPacket = new uint8_t[setTorqueLimitPacketLen];
	setTorqueLimitPacket[H1] = HEADER1;
	setTorqueLimitPacket[H2] = HEADER2;
	setTorqueLimitPacket[ID] = BROADCAST_ID;
	setTorqueLimitPacket[LEN] = setTorqueLimitPacketLen - 4;
	setTorqueLimitPacket[INS] = SYNC_WRITE;
	setTorqueLimitPacket[SYNC_WRITE_ADDR] = TORQUE_LIMIT;
	setTorqueLimitPacket[SYNC_WRITE_LEN] = 2;

	data = setTorqueLimitPacket + SYNC_WRITE_LEN + 1;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
		*(data++) = devices[i]->id;
		*(data++) = 0;	// LSB
		*(data++) = 0;	// MSB
	}

	if (setPositionPacket)
		delete setPositionPacket;

	setPositionPacketLen = 5 + 2 + 3 * (uint8_t)devices.size() + 1;
	setPositionPacket = new uint8_t[setPositionPacketLen];
	setPositionPacket[H1] = HEADER1;
	setPositionPacket[H2] = HEADER2;
	setPositionPacket[ID] = BROADCAST_ID;
	setPositionPacket[LEN] = setPositionPacketLen - 4;
	setPositionPacket[INS] = SYNC_WRITE;
	setPositionPacket[SYNC_WRITE_ADDR] = GOAL_POSITION;
	setPositionPacket[SYNC_WRITE_LEN] = 2;

	data = setPositionPacket + SYNC_WRITE_LEN + 1;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
		*(data++) = devices[i]->id;
		*(data++) = 0;	// LSB
		*(data++) = 0;	// MSB
	}

	if (setLEDPacket)
		delete setLEDPacket;

	setLEDPacketLen = 5 + 2 + 2 * (uint8_t)devices.size() + 1;
	setLEDPacket = new uint8_t[setLEDPacketLen];
	setLEDPacket[H1] = HEADER1;
	setLEDPacket[H2] = HEADER2;
	setLEDPacket[ID] = BROADCAST_ID;
	setLEDPacket[LEN] = setLEDPacketLen - 4;
	setLEDPacket[INS] = SYNC_WRITE;
	setLEDPacket[SYNC_WRITE_ADDR] = LED_DEVICE;
	setLEDPacket[SYNC_WRITE_LEN] = 1;

	data = setLEDPacket + SYNC_WRITE_LEN + 1;
	for (unsigned int i = 0; i < devices.size(); i++)
	{
		*(data++) = devices[i]->id;
		*(data++) = 0;
	}

	packetBuilt = true;
}

uint8_t Dynamixel::CalcChecksum(uint8_t *buffer, uint8_t len)
{
	uint8_t chksum = 0;
	for (int i = 0; i < len; i++)
		chksum += *(buffer++);
	return ~chksum;
}

bool Dynamixel::CheckStatusChecksum(uint8_t *buffer, uint8_t len)
{
	uint8_t chk = CalcChecksum(buffer, len - 1);
	return (chk == buffer[len - 1]);
}



// From https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/src/dynamixel_sdk/port_handler_windows.cpp
#if defined(_WIN32) || defined(_WIN64)

bool Dynamixel::open()
{
	DCB dcb;
	COMMTIMEOUTS timeouts;
	DWORD dwError;

	close();

	hPort = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hPort == INVALID_HANDLE_VALUE)
	{
		printf("[PortHandlerWindows::SetupPort] Error opening serial port!\n");
		return false;
	}

	dcb.DCBlength = sizeof(DCB);
	if (GetCommState(hPort, &dcb) == FALSE)
		goto DXL_HAL_OPEN_ERROR;

	// Set baudrate
	dcb.BaudRate = (DWORD)baud;
	dcb.ByteSize = 8;                    // Data bit = 8bit
	dcb.Parity = NOPARITY;             // No parity
	dcb.StopBits = ONESTOPBIT;           // Stop bit = 1
	dcb.fParity = NOPARITY;             // No Parity check
	dcb.fBinary = 1;                    // Binary mode
	dcb.fNull = 0;                    // Get Null byte
	dcb.fAbortOnError = 0;
	dcb.fErrorChar = 0;
	// Not using XOn/XOff
	dcb.fOutX = 0;
	dcb.fInX = 0;
	// Not using H/W flow control
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fDsrSensitivity = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fOutxCtsFlow = 0;

	if (SetCommState(hPort, &dcb) == FALSE)
		goto DXL_HAL_OPEN_ERROR;

	if (SetCommMask(hPort, 0) == FALSE) // Not using Comm event
		goto DXL_HAL_OPEN_ERROR;
	if (SetupComm(hPort, 4096, 4096) == FALSE) // Buffer size (Rx,Tx)
		goto DXL_HAL_OPEN_ERROR;
	if (PurgeComm(hPort, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE) // Clear buffer
		goto DXL_HAL_OPEN_ERROR;
	if (ClearCommError(hPort, &dwError, NULL) == FALSE)
		goto DXL_HAL_OPEN_ERROR;

	if (GetCommTimeouts(hPort, &timeouts) == FALSE)
		goto DXL_HAL_OPEN_ERROR;
	// Timeout (Not using timeout)
	// Immediatly return
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 1; // must not be zero.
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	if (SetCommTimeouts(hPort, &timeouts) == FALSE)
		goto DXL_HAL_OPEN_ERROR;

	tx_time_per_byte = 10.0 / (double)baud;		// This doesn't take into account the "polling" action of USB
	return true;

DXL_HAL_OPEN_ERROR:
	close();
	return false;
}

void Dynamixel::close()
{
	if (hPort != INVALID_HANDLE_VALUE)
	{
		CloseHandle(hPort);
		hPort = INVALID_HANDLE_VALUE;
	}
}

int Dynamixel::bytesAvailable()
{
	DWORD bytesAvailable=2;

	COMSTAT stat;
	DWORD errors;
	if (!ClearCommError(hPort, &errors, &stat))
	{
		DWORD e = GetLastError();
		return -1;
	}

	return (int)stat.cbInQue;
}

int Dynamixel::write(uint8_t *buffer, uint8_t len)
{
	DWORD written = 0;

	if (WriteFile(hPort, buffer, len, &written, NULL) == FALSE)
		return -1;

	return (int)written;
}

int Dynamixel::read(uint8_t *buffer, uint8_t len)
{
	DWORD bytesRead = 0;

	if (ReadFile(hPort, buffer, len, &bytesRead, NULL) == FALSE)
		return -1;

	return (int)bytesRead;
}

void Dynamixel::initTimer()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	timerFrequency = freq.QuadPart;
}

int64_t Dynamixel::getTimer()
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return counter.QuadPart;
}

void Dynamixel::delayms(uint32_t ms)
{
    Sleep(ms);
}
#else

static int getCFlagBaud(int baudrate)
{
	switch(baudrate)
	{
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 500000:
			return B500000;
		case 576000:
			return B576000;
		case 921600:
			return B921600;
		case 1000000:
			return B1000000;
		case 1152000:
			return B1152000;
		case 1500000:
			return B1500000;
		case 2000000:
			return B2000000;
		case 2500000:
			return B2500000;
		case 3000000:
			return B3000000;
		case 3500000:
			return B3500000;
		case 4000000:
			return B4000000;
		default:
			return -1;
	}
}

static bool setCustomBaudrate(HANDLE hPort, int speed)
{
	// try to set a custom divisor
	struct serial_struct ss;
	if(ioctl(hPort, TIOCGSERIAL, &ss) != 0)
	{
		printf("[PortHandlerLinux::SetCustomBaudrate] TIOCGSERIAL failed!\n");
		return false;
	}

	ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
	ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
	int closest_speed = ss.baud_base / ss.custom_divisor;

	if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
	{
		printf("[PortHandlerLinux::SetCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
		return false;
	}

	if(ioctl(hPort, TIOCSSERIAL, &ss) < 0)
	{
		printf("[PortHandlerLinux::SetCustomBaudrate] TIOCSSERIAL failed!\n");
		return false;
	}

	return true;
}


bool Dynamixel::open()
{
	struct termios newtio;

	hPort = ::open(portName.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(hPort < 0)
	{
        printf("[PortHandlerLinux::SetupPort] Error opening serial port! Errno=%d\n", errno );
		return false;
	}

	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    int cflag_baud = getCFlagBaud(baud);
    bool customBaud = cflag_baud <= 0;
    if ( customBaud ) cflag_baud = B9600;

	newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag      = 0;
	newtio.c_lflag      = 0;
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN]   = 0;

	// clean the buffer and activate the settings for the port
	tcflush(hPort, TCIFLUSH);
	tcsetattr(hPort, TCSANOW, &newtio);

    if ( customBaud )
    {
        setCustomBaudrate( hPort, baud );
    }

	tx_time_per_byte = 10.0 / (double)baud;		// This doesn't take into account the "polling" action of USB
	return true;
}

void Dynamixel::close()
{
	if (hPort != INVALID_HANDLE_VALUE)
	{
		::close(hPort);
		hPort = INVALID_HANDLE_VALUE;
	}
}

int Dynamixel::bytesAvailable()
{
 	int bytes_available=0;
  	ioctl(hPort, FIONREAD, &bytes_available);
  	return bytes_available;
}

int Dynamixel::write(uint8_t *buffer, uint8_t len)
{
    //char buf[1024];
    //buf[0] = 0;
    //for ( int i = 0; i < len; i++ )
        //sprintf(buf+strlen(buf), "%02X ", buffer[i] );
    //printf( "%s\n",buf );
	return ::write(hPort, buffer, len);
}

int Dynamixel::read(uint8_t *buffer, uint8_t len)
{
	return ::read(hPort, buffer, len);
}

void Dynamixel::initTimer()
{
    // us
    timerFrequency = 1000000;
}

int64_t Dynamixel::getTimer()
{
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
    int64_t t = tv.tv_sec * 1000000 + tv.tv_nsec /1000;
    return t;
}

void Dynamixel::delayus(uint32_t us)
{
    usleep(us);
}

#endif
