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

	dataPollTimer.setSingleShot(true);

	connect(&dataPollTimer, SIGNAL(timeout()), SLOT(onDataPollTimeout()));
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

static QTableWidgetItem* CloneNewCell(QTableWidget* tableDevice, int row, int col, const QTableWidgetItem& proto, const QString& text)
{
	QTableWidgetItem* item = proto.clone();
	item->setText(text);
	tableDevice->setItem(row, col, item);
	return item;
}

void DeviceFrame::InitialiseData(QSharedPointer<SerialInterface> serialIFace, int model, int id, int baudRate)
{
	iface = serialIFace;
	baud = baudRate;
	deviceId = id;
	modelNumber = model;
	ui.tableDevice->clearContents();

	iface->ReopenPort();
	iface->setBaud(baud);

	// Populate table with eeprom and ram data.
	QTableWidgetItem proto;
	proto.setBackgroundColor("lavenderblush");
	int row = 0;
	for each(DeviceData dd in iface->getDevices().find(model).value().eepromData)
	{
		ui.tableDevice->setRowCount(row + 1);
		CloneNewCell(ui.tableDevice, row, 0, proto, QString::number(dd.address))->setTextAlignment(Qt::AlignCenter);
		CloneNewCell(ui.tableDevice, row, 1, proto, dd.dataname);
		CloneNewCell(ui.tableDevice, row, 2, proto, "")->setBackgroundColor(dd.readOnly ? "whitesmoke" : "white");
		row++;
	}
	proto.setBackgroundColor("aliceblue");
	for each (DeviceData dd in iface->getDevices().find(model).value().ramData)
	{
		ui.tableDevice->setRowCount(row + 1);
		CloneNewCell(ui.tableDevice, row, 0, proto, QString::number(dd.address))->setTextAlignment(Qt::AlignCenter);
		CloneNewCell(ui.tableDevice, row, 1, proto, dd.dataname);
		CloneNewCell(ui.tableDevice, row, 2, proto, "")->setBackgroundColor(dd.readOnly ? "whitesmoke" : "white");
		row++;
	}
	ui.tableDevice->resizeRowsToContents();

	dataPollTimer.start(0);
}

QString DeviceFrame::FormatData(QByteArray &buf, int addr, int size)
{
	if (size == 1)
		return QString::number((uint8_t)buf.at(addr));
	return QString::number((uint8_t)buf.at(addr) + ((uint8_t)buf.at(addr + 1) << 8));
}

void DeviceFrame::onDataPollTimeout()
{
	auto it = iface->getDevices().constFind(modelNumber);
	const Device& d = iface->getDevices().find(modelNumber).value();
	int dataCount = d.ramData.last().address + d.ramData.last().size;

	QByteArray buf(dataCount, 0);
	if (iface->Read(deviceId, 0, dataCount, (uint8_t*)buf.data()))	// TODO make an async call
	{
		// Process the data
		int row = 0;
		for each (DeviceData dd in d.eepromData)
		{
			ui.tableDevice->item(row, 2)->setText(FormatData(buf, dd.address, dd.size));
			row++;
		}
		for each (DeviceData dd in d.ramData)
		{
			ui.tableDevice->item(row, 2)->setText(FormatData(buf, dd.address, dd.size));
			row++;
		}
	}

	dataPollTimer.start(1000);
}
