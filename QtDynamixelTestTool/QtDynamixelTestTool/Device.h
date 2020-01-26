#pragma once
#include <QtCore/QMap>


class DeviceData
{
public:
	int address;
	int size;
	QString dataname;
	QString description;
	bool readOnly;
	QString editor;


	DeviceData(int address, int size, QString dataname, QString description, bool readOnly, QString editor)
		: address(address)
		, size(size)
		, dataname(dataname)
		, description(description)
		, readOnly(readOnly)
		, editor(editor)
	{
	}
};

class Device
{
public:
	QString name;
	int modelNumber;
	int inheritedDeviceNumber;
	QMap<int, DeviceData> eepromData;
	QMap<int,DeviceData> ramData;

public:
	Device()
	{
		inheritedDeviceNumber = -1;
	}

	const Device& operator = (const Device& that)
	{
		this->name = that.name;
		this->modelNumber = that.modelNumber;
		this->inheritedDeviceNumber = that.inheritedDeviceNumber;
		this->eepromData = that.eepromData;
		this->ramData = that.ramData;
		return that;
	}

	void setInherits(int deviceNumber) { inheritedDeviceNumber = deviceNumber; }
	void setName(QString deviceName) { name = deviceName; }
	void setModelNumber(int number) { modelNumber = number; }
	void addEEPROMData(const DeviceData& data) { eepromData.insert(data.address, data); }
	void addRamData(const DeviceData& data) { ramData.insert(data.address, data); }
};

