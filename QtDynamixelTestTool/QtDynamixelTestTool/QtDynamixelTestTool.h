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
	void writePositionSettings();
	void readPositionSettings();

public slots:
	void onFoundDevice(int baud, int id, int model);
};
