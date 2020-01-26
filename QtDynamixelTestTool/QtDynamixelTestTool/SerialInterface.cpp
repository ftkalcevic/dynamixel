#include "SerialInterface.h"
#include <QtWidgets/QMessageBox>
#include <QtXml/QDomDocument>
#include <QtCore/QFile>

SerialInterface::SerialInterface(QObject *parent)
	: QObject(parent)
{
	ReadConfig();
}

SerialInterface::~SerialInterface()
{
}

void SerialInterface::ClosePort()
{
	if (port && port->isOpen())
	{
		port->close();
	}
	port = nullptr;
}

bool SerialInterface::OpenPort(QString portName)
{
	ClosePort();

	this->portName = portName;
	port = QSharedPointer<QSerialPort>(new QSerialPort(portName));
	bool success = port->open(QIODevice::ReadWrite);
	if (!success)
	{
		QMessageBox::critical(nullptr, "Error", QString("Failed to open port '%1' - '%2'").arg(portName).arg(port->errorString()));
		port = nullptr;
	}

	return success;
}

void SerialInterface::ReopenPort()
{
	ClosePort();
	OpenPort(portName);
}

void SerialInterface::setBaud(int baudRate)
{
	if (port)
		port->setBaudRate(baudRate);
}


void SerialInterface::ReadConfig()
{
	// Get the xml
	QDomDocument doc;
	QFile file("Config.xml");
	if (!file.open(QIODevice::ReadOnly))
		return;
	if (!doc.setContent(&file)) {
		file.close();
		return;
	}
	file.close();

	// Parse the devices
	QDomElement nodeTestTool = doc.documentElement();
	QDomNode nodeDevices = nodeTestTool.elementsByTagName("Devices").at(0);
	QDomNodeList deviceList = nodeDevices.toElement().elementsByTagName("Device");
	for (int d = 0; d < deviceList.count(); d++)
	{
		QDomNode device = deviceList.at(d);
		// This section contains the data used by devices.  Multiple devices can share the same data.
		// ControlTableEEPROM
		QDomNodeList eepromDataList = device.toElement().elementsByTagName("ControlTableEEPROM").at(0).toElement().elementsByTagName("Data");
		for (int i = 0; i < eepromDataList.count(); i++)
		{
			QDomNode node = eepromDataList.at(i);
			QString address = node.attributes().namedItem("Address").nodeValue();
			QString size = node.attributes().namedItem("Size").nodeValue();
			QString dataname = node.attributes().namedItem("DataName").nodeValue();
			QString description = node.attributes().namedItem("Description").nodeValue();
			QString access = node.attributes().namedItem("Access").nodeValue();
			QString initialvalue = node.attributes().namedItem("InitialValue").nodeValue();
			QString editor = node.attributes().namedItem("Editor").nodeValue();
		}

		// ControlTableRAM
		QDomNodeList ramDataList = device.toElement().elementsByTagName("ControlTableRAM").at(0).toElement().elementsByTagName("Data");
		for (int i = 0; i < ramDataList.count(); i++)
		{
			QDomNode node = ramDataList.at(i);
			QString address = node.attributes().namedItem("Address").nodeValue();
			QString size = node.attributes().namedItem("Size").nodeValue();
			QString dataname = node.attributes().namedItem("DataName").nodeValue();
			QString description = node.attributes().namedItem("Description").nodeValue();
			QString access = node.attributes().namedItem("Access").nodeValue();
			QString initialvalue = node.attributes().namedItem("InitialValue").nodeValue();
			QString editor = node.attributes().namedItem("Editor").nodeValue();
		}

		// This identifies each device.  A dataset can be used by multiple devices.  We create a different device for each row.
		// Identifiers
		QDomNodeList idDataList = device.toElement().elementsByTagName("Identifiers").at(0).toElement().elementsByTagName("Identifier");
		for (int i = 0; i < idDataList.count(); i++)
		{
			QDomNode node = idDataList.at(i);
			QString modelNumber = node.attributes().namedItem("ModelNumber").nodeValue();
			QString name = node.attributes().namedItem("Name").nodeValue();
		}
	}
}

uint8_t CalculateChecksum(uint8_t *packet, int packet_len)
{
	uint8_t chksum = 0;
	for (int i = 2; i < packet_len - 1; i++)
		chksum += packet[i];
	return ~chksum;
}

void UpdateChecksum(uint8_t* packet, int packet_len)
{
	uint8_t chksum = CalculateChecksum(packet, packet_len);
	packet[packet[3] + 3] = chksum;
}

bool ValidChecksum(uint8_t* packet, int packet_len)
{
	uint8_t chksum = CalculateChecksum(packet, packet_len);
	return packet[packet[3] + 3] == chksum;
}

#define INS_PING 0x01
#define INS_READ 0x02

#define MAX_PACKET	(0xFF+4)	// len + (header1,header2,id,chksum)

bool SerialInterface::Ping(uint8_t id)
{
	uint8_t packet[6] = { 0xFF, 0xFF, id, 0x02, INS_PING, 0 };
	UpdateChecksum(packet, sizeof(packet));

	return SendAndWait(packet, sizeof(packet));
}

bool SerialInterface::Read(uint8_t id, uint8_t address, uint8_t len, uint8_t *buffer)
{
	uint8_t packet[8] = { 0xFF, 0xFF, id, 0x04, INS_READ, address, len, 0 };
	UpdateChecksum(packet, sizeof(packet));

	if (SendAndWait(packet, sizeof(packet), buffer ))
	{
		return true;
	}
	return false;
}


bool SerialInterface::SendAndWait(uint8_t* packet, int packet_len, uint8_t* return_buffer, int timeout)
{
	port->write((const char *)packet, packet_len);
	while (port->bytesToWrite() > 0)
	{
		if (!port->waitForBytesWritten(timeout))
			return false;
	}

	uint8_t status_packet[MAX_PACKET];
	uint8_t* ptr = status_packet;
	int len = 0; 
	while (true)
	{
		if (!port->waitForReadyRead(timeout))
			return false;
		int bytes_read = port->read((char*)ptr, sizeof(status_packet) );
		if (bytes_read <= 0)
			return false;

		ptr += bytes_read;
		len += bytes_read;
		if (len >= 4 && status_packet[3] + 4 >= len)
		{
			// got valid status reply
			if (ValidChecksum(status_packet, status_packet[3] + 4))
			{
				if (return_buffer)
				{
					memcpy(return_buffer, status_packet + 5, status_packet[3] - 1);
				}
				return true;
			}
			else
				return false;
		}

	}
}
