#pragma once
#include <QtSerialPort/QSerialPortInfo>
#include <QtCore/QSharedPointer>
#include "SerialInterface.h"

class SerialPorts
{
public:
	SerialPorts();
	~SerialPorts();
public:
	static QList<QSerialPortInfo> GetAllPorts();
	static QSharedPointer<SerialInterface> OpenSerialInterface(QString portName);
};
