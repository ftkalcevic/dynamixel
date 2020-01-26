#pragma once

#include <QWidget>
#include "ui_ConnectivityFrame.h"
#include "SerialInterface.h"
#include "ScanDevicesThread.h"

class ConnectivityFrame : public QFrame
{
	Q_OBJECT

public:
	ConnectivityFrame(QWidget *parent = Q_NULLPTR);
	~ConnectivityFrame();

	void InitialiseData(QSharedPointer<SerialInterface> iface);

private:
	Ui::Frame ui;

	void EnableControls();
	QSharedPointer<SerialInterface> iface;
	QSharedPointer<ScanDevicesThread> scanThread;

signals:
	void ScanChange(int baud, int id);
	void FoundDevice(int baud, int id, int model);

public slots:
	void onConnectClicked(void);
	void onDisconnectClicked(void);
	void onStopScanClicked(void);
	void onScanClicked(void);
	void onScanFinished();
	void onScanChanged(int baud, int id);
	void onFoundDevice(int baud, int id, int model);
};


