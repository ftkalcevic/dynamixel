#include "DeviceFrame.h"

DeviceFrame::DeviceFrame(QWidget *parent)
	: QFrame(parent)
{
	ui.setupUi(this);
	ui.tableDevice->setColumnCount(3);
	
	ui.tableDevice->setHorizontalHeaderLabels(QStringList() << "Addr" << "Description" << "Value");
	ui.tableDevice->setColumnWidth(0, 50);
	ui.tableDevice->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
	ui.tableDevice->setColumnWidth(2, 100);
}

DeviceFrame::~DeviceFrame()
{
}


void DeviceFrame::writeSettings(QSettings& qsettings)
{
}

void DeviceFrame::readSettings(QSettings& qsettings)
{
}

void DeviceFrame::InitialiseData(QSharedPointer<SerialInterface> iface, int model)
{
	ui.tableDevice->clearContents();

	// Populate table with eeprom and ram data.
	int row = 0;
	for each(DeviceData dd in iface->getDevices().find(model).value().eepromData)
	{
		ui.tableDevice->setRowCount(row + 1);
		ui.tableDevice->setItem(row, 0, new QTableWidgetItem(QString::number(dd.address)));
		ui.tableDevice->item(row, 0)->setTextAlignment(Qt::AlignCenter);
		ui.tableDevice->setItem(row, 1, new QTableWidgetItem(dd.dataname));
		row++;
	}
	for each (DeviceData dd in iface->getDevices().find(model).value().ramData)
	{
		ui.tableDevice->setRowCount(row + 1);
		ui.tableDevice->setItem(row, 0, new QTableWidgetItem(QString::number(dd.address)));
		ui.tableDevice->item(row, 0)->setTextAlignment(Qt::AlignCenter);
		ui.tableDevice->setItem(row, 1, new QTableWidgetItem(dd.dataname));
		row++;
	}
	ui.tableDevice->resizeRowsToContents();
}