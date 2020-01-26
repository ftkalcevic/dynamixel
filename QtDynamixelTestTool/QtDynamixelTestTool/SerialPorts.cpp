#include "SerialPorts.h"
#include <QtSerialPort/QSerialPort>
#include <QtCore/QDebug>
#include <QtWidgets/QMessageBox>

SerialPorts::SerialPorts()
{
}

SerialPorts::~SerialPorts()
{
}

QList<QSerialPortInfo> SerialPorts::GetAllPorts()
{
	return QSerialPortInfo::availablePorts();
}

QSharedPointer<SerialInterface> SerialPorts::OpenSerialInterface(QString portName)
{
	QSharedPointer<QSerialPort> port( new QSerialPort(portName) );
	if (!port->open(QIODevice::ReadWrite))
	{
		QMessageBox::critical(NULL, "Error", QString("Failed to open port - ")  + port->error() );
	}
	return QSharedPointer<SerialInterface>(new SerialInterface(NULL));
}
