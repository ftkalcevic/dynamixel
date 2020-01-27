#pragma once

#include <QFrame>
#include <QtCore/QSettings>
#include <QtCore/QTimer>
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
	void InitialiseData(QSharedPointer<SerialInterface> iface, int model, int id, int baudRate);

public slots:
	void onDataPollTimeout();
	void onSelectionChanged();
	void onDataChanged(int address, int size, int value);

private:
	Ui::DeviceFrame ui;
	QSharedPointer<SerialInterface> iface;
	QTimer dataPollTimer;
	int deviceId;
	int baud;
	int modelNumber;

	QString FormatData(QByteArray& buf, int addr, int size);
};

void CreateDataRow(int row, QTableWidgetItem& proto, DeviceData& dd);
