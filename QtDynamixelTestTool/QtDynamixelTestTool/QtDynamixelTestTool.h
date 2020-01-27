#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtDynamixelTestTool.h"
#include "SerialInterface.h"


class QtDynamixelTestTool : public QMainWindow
{
	Q_OBJECT

public:
	QtDynamixelTestTool(QWidget *parent = Q_NULLPTR);

private:
	Ui::QtDynamixelTestToolClass ui;
	QSharedPointer<SerialInterface> iface;

	virtual void closeEvent(QCloseEvent*);
	void writeSettings();
	void readSettings();
	void EnableControls();

public slots:
	void onFoundDevice(int baud, int id, int model);
	void onScanStart();
	void onDeviceSelectionChanged();
	void contextMenu(const QPoint& pos);
	void onFactoryReset();
};
