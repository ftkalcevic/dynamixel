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

DeviceData ReadDeviceData(QDomNode& node)
{
	int address = node.attributes().namedItem("Address").nodeValue().toInt();;
	int size = node.attributes().namedItem("Size").nodeValue().toInt();
	QString dataname = node.attributes().namedItem("DataName").nodeValue();
	QString description = node.attributes().namedItem("Description").nodeValue();
	bool readOnly = node.attributes().namedItem("ReadOnly").nodeValue() == "true" ? true : false;
	QString editor = node.attributes().namedItem("Editor").nodeValue();
	DeviceData data(address, size, dataname, description, readOnly, editor);

	QDomNode child = node.firstChild();
	if (child.nodeName() == "EditorInt")
	{
		if (child.attributes().contains("Min"))
			data.min = child.attributes().namedItem("Min").nodeValue().toInt();
		if (child.attributes().contains("Max"))
			data.max = child.attributes().namedItem("Max").nodeValue().toInt();
	}
	else if (child.nodeName() == "EditorEnum")
	{
		QDomNode enumNode = child.firstChildElement();
		while (!enumNode.isNull())
		{
			int id = enumNode.attributes().namedItem("Id").nodeValue().toInt();
			QString text = enumNode.attributes().namedItem("Text").nodeValue();
			data.AddEnum(id, text);
			enumNode = enumNode.nextSiblingElement();
		}
	}

	return data;
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
		Device device;

		QDomNode xmlDevice = deviceList.at(d);
		int inheritsModelId = -1;
		if (xmlDevice.attributes().contains("Inherits"))
		{
			inheritsModelId = xmlDevice.attributes().namedItem("Inherits").nodeValue().toInt();
		}
		device.setInherits(inheritsModelId);

		// This section contains the data used by devices.  Multiple devices can share the same data.
		// ControlTableEEPROM
		QDomNodeList eepromDataList = xmlDevice.toElement().elementsByTagName("ControlTableEEPROM").at(0).toElement().elementsByTagName("Data");
		for (int i = 0; i < eepromDataList.count(); i++)
		{
			QDomNode node = eepromDataList.at(i);
			device.addEEPROMData(ReadDeviceData(node));
		}

		// ControlTableRAM
		QDomNodeList ramDataList = xmlDevice.toElement().elementsByTagName("ControlTableRAM").at(0).toElement().elementsByTagName("Data");
		for (int i = 0; i < ramDataList.count(); i++)
		{
			QDomNode node = ramDataList.at(i);
			device.addRamData(ReadDeviceData(node));
		}

		// This identifies each device.  A dataset can be used by multiple devices.  We create a different device for each row.
		// Identifiers
		QDomNodeList idDataList = xmlDevice.toElement().elementsByTagName("Identifiers").at(0).toElement().elementsByTagName("Identifier");
		for (int i = 0; i < idDataList.count(); i++)
		{
			QDomNode node = idDataList.at(i);
			int modelNumber = node.attributes().namedItem("ModelNumber").nodeValue().toInt();
			QString name = node.attributes().namedItem("Name").nodeValue();

			Device newDevice;
			newDevice = device;
			newDevice.setName(name);
			newDevice.setModelNumber(modelNumber);

			devices.insert(modelNumber, newDevice);
		}
	}
	// Find the devices that inherit, and copy the parent data across.
	for ( auto d = devices.begin(); d != devices.end(); d++ )
	{
		if (d.value().inheritedDeviceNumber >= 0)
		{
			// Find the parent and copy the eeprom and ram rows.
			const Device& parent = devices.value(d.value().inheritedDeviceNumber);
			for each (const DeviceData & r in parent.eepromData)
				d.value().addEEPROMData(r);
			for each (const DeviceData & r in parent.ramData)
				d.value().addRamData(r);
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
#define INS_WRITE 0x03

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

bool SerialInterface::Write(uint8_t id, uint8_t address, uint8_t len, uint8_t* buffer)
{
	assert(len < (255 - 3));
	uint8_t packet[255+4] = { 0xFF, 0xFF, id, len+3, INS_WRITE, address };
	for (int i = 0; i < len; i++)
		packet[6 + i] = buffer[i];
	int packet_len = 6 + len + 1;
	UpdateChecksum(packet, packet_len);

	if (SendAndWait(packet, packet_len, buffer))
	{
		return true;
	}
	return false;
}

bool SerialInterface::SendAndWait(uint8_t* packet, int packet_len, uint8_t* return_buffer, int timeout)
{
	if (!port->isOpen())
		return false;

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
