#pragma once

#include "Device.h"

#define READ_BUFFER_SIZE	64

class Dynamixel
{
	std::vector<Device *> devices;
	std::map<uint8_t,Device *> devicesById;
	bool packetBuilt;
	uint8_t *readPositionPacket;
	uint8_t *setVelocityPacket;
	uint8_t readPositionPacketLen;
	uint8_t setVelocityPacketLen;
	int baud;
	std::string portName;
	HANDLE hPort;
	double tx_time_per_byte;
	double reply_delay;
	int64_t timerFrequency;
	uint8_t readId;
	uint8_t readLen;
	uint8_t readChksum;
	uint8_t readBytes;
	uint8_t readBuffer[READ_BUFFER_SIZE];
	uint8_t devicesToRead;

	int checksumErrors;
	int statusChecksumErrors;
	int timeoutErrors;

	void buildPackets();
	uint8_t CalcChecksum(uint8_t *buffer, uint8_t len);
	bool CheckStatusChecksum(uint8_t *buffer, uint8_t len);
	void ProcessPosition(uint8_t id, uint8_t *buffer, uint8_t len);

	void BlockingReadPositions();
	void initTimer();
	int64_t getTimer();
	bool ProcessReadPositions(uint8_t *buffer, int bytesRead);
	bool ProcessReadStatus(uint8_t *buffer, int len);
	void BlockingReadStatus();

	enum EReadPositionsState
	{
		WaitForHeader,
		WaitForHeader2,
		WaitForID,
		WaitForLen,
		WaitForData
	} readPositionsState;

public:
	Dynamixel(std::string portName, int baud);
	~Dynamixel();
	void addDevice(Device *device);
	void readPositions();
	void setVelocities();
	void enableTorque(bool enable);
	void ping(int id);
	void setWheelMode();
	bool open();
	void close();
	int write(uint8_t *buffer, uint8_t len);
	int read(uint8_t *buffer, uint8_t len);
	int bytesAvailable();
};

