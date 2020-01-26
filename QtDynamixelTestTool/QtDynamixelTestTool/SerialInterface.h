#pragma once

#include <QtCore/QObject>
#include <QtCore/QSharedPointer>
#include <QtSerialPort/QSerialPort>
#include "Protocol1.h"

#define DEFAULT_TIMEOUT	100		// ms

class SerialInterface : public QObject, public Protocol1
{
	Q_OBJECT

	QSharedPointer<QSerialPort> port;
	QString portName;

	void ReadConfig();
	bool SendAndWait(uint8_t *packet, int packet_len, uint8_t *return_buffer = NULL, int timeout=DEFAULT_TIMEOUT);

public:
	SerialInterface(QObject *parent);
	~SerialInterface();
	bool isConnected() const { return port && port->isOpen(); }
	bool OpenPort(QString port);
	void ClosePort();
	void ReopenPort();
	void setBaud(int baudRate);

	bool Ping(uint8_t id);
	bool Read(uint8_t id, uint8_t address, uint8_t len, uint8_t* buffer);
};
