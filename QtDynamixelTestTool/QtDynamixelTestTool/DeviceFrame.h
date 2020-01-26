#pragma once

#include <QFrame>
#include <QtCore/QSettings>
#include "ui_DeviceFrame.h"
#include "SerialInterface.h"

class DeviceFrame : public QFrame
{
	Q_OBJECT

public:
	DeviceFrame(QWidget *parent = Q_NULLPTR);
	~DeviceFrame();
	void writeSettings(QSettings& qsettings);
	void readSettings(QSettings& qsettings);
	void InitialiseData(QSharedPointer<SerialInterface> iface, int model);

private:
	Ui::DeviceFrame ui;
	QSharedPointer<SerialInterface> iface;
};
