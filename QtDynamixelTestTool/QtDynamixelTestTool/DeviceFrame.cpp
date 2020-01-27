#include "DeviceFrame.h"
#include "EditInt.h"
#include "EditEnum.h"
#include "EditBoolean.h"

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
	connect(ui.tableDevice, SIGNAL(itemSelectionChanged()), SLOT(onSelectionChanged()));
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

void CreateDataRow(QTableWidget *table, int row, QTableWidgetItem& proto, DeviceData& dd)
{
	table->setRowCount(row + 1);
	QTableWidgetItem* item = CloneNewCell(table, row, 0, proto, QString::number(dd.address));
	item->setTextAlignment(Qt::AlignCenter);
	item->setData(Qt::UserRole, dd.address);
	CloneNewCell(table, row, 1, proto, dd.dataname);
	CloneNewCell(table, row, 2, proto, "")->setBackgroundColor(dd.readOnly ? "whitesmoke" : "white");
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
		CreateDataRow(ui.tableDevice, row, proto, dd);
		row++;
	}
	proto.setBackgroundColor("aliceblue");
	for each (DeviceData dd in iface->getDevices().find(model).value().ramData)
	{
		CreateDataRow(ui.tableDevice, row, proto, dd);
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


void DeviceFrame::onSelectionChanged()
{
	if (ui.tableDevice->selectedItems().count() > 0)
	{
		int row = ui.tableDevice->selectedItems().at(0)->row();
		QTableWidgetItem* item = ui.tableDevice->item(row,0);
		int address = item->data(Qt::UserRole).toInt();
		const Device& device = iface->getDevices().constFind(modelNumber).value();

		QMap<int,DeviceData>::ConstIterator it = device.eepromData.find(address);
		if (it == device.eepromData.end())
			it = device.ramData.find(address);

		const DeviceData& dd = it.value();
		QFrame * frame = nullptr;
		if (dd.editor == "Int")
		{
			// pass in - current value, min, max, name, description
			frame = new EditInt(this, dd.address, dd.size, ui.tableDevice->item(row, 2)->text().toInt(), dd.min, dd.max, dd.dataname, dd.description, dd.readOnly );
			connect(frame, SIGNAL(DataChanged(int,int,int)), SLOT(onDataChanged(int,int,int)));
		}
		else if (dd.editor == "Enum")
		{
			frame = new EditEnum(this, dd.address, dd.size, ui.tableDevice->item(row, 2)->text().toInt(), dd.enums, dd.dataname, dd.description, dd.readOnly);
			connect(frame, SIGNAL(DataChanged(int, int, int)), SLOT(onDataChanged(int, int, int)));
		}
		else if (dd.editor == "Boolean")
		{
			frame = new EditBoolean(this, dd.address, dd.size, ui.tableDevice->item(row, 2)->text().toInt(), dd.dataname, dd.description, dd.readOnly);
			connect(frame, SIGNAL(DataChanged(int, int, int)), SLOT(onDataChanged(int, int, int)));
		}

		if ( frame != nullptr )
		{
			if (ui.frame->layout() != nullptr)
			{
				QLayoutItem* item;
				while ((item = ui.frame->layout()->takeAt(0)) != nullptr)
				{
					QWidget* w = item->widget();
					w->hide();
					delete w;
				}
				delete ui.frame->layout();
			}

			QGridLayout *layout = new QGridLayout(ui.frame);
			layout->addWidget(frame);
			ui.frame->setLayout(layout);
		}
		// BooleanOnOff
		// Baud - enum
	}
	else
	{
		// Show device info
	}

}


void DeviceFrame::onDataChanged(int address, int size, int value)
{
	uint16_t uvalue = value;

	uint8_t buffer[20];
	buffer[0] = uvalue & 0xFF;
	buffer[1] = (uvalue>>8) & 0xFF;

	iface->Write(deviceId, address, size, buffer);
}

