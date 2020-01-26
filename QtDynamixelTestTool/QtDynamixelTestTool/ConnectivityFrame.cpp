#include "ConnectivityFrame.h"
#include "SerialPorts.h"

ConnectivityFrame::ConnectivityFrame(QWidget *parent)
	: QFrame(parent)
{
	ui.setupUi(this);
}

ConnectivityFrame::~ConnectivityFrame()
{
	onStopScanClicked();
}

void ConnectivityFrame::InitialiseData(QSharedPointer<SerialInterface> iface)
{
	this->iface = iface;

	for each (auto info in SerialPorts::GetAllPorts())
	{
		ui.cboPorts->addItem(info.portName() + " " + info.description() + " " + info.manufacturer(), info.portName());
	}

	for each (auto baud in iface->getBauds())
	{
		ui.listBauds->addItem(baud.desc);
		auto item = ui.listBauds->item(ui.listBauds->count() - 1);
		item->setData(Qt::UserRole, baud.value);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Unchecked);
	}

	connect(ui.cboPorts, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int index) { EnableControls(); });
	connect(ui.listBauds, &QListWidget::itemChanged, this, [=](QListWidgetItem* item) { EnableControls(); });
	connect(ui.btnConnect, SIGNAL(clicked(bool)), this, SLOT(onConnectClicked()));
	connect(ui.btnDisconnect, SIGNAL(clicked(bool)), this, SLOT(onDisconnectClicked()));
	connect(ui.btnScan, SIGNAL(clicked(bool)), this, SLOT(onScanClicked()));
	connect(ui.btnStopScan, SIGNAL(clicked(bool)), this, SLOT(onStopScanClicked()));
	EnableControls();
}

static int checkedItems(QListWidget* widget)
{
	int count = 0;
	for (int i = 0; i < widget->count(); i++)
		if (widget->item(i)->checkState() == Qt::Checked)
			count++;
	return count;
}

void ConnectivityFrame::EnableControls()
{
	bool connected = !iface.isNull() && iface->isConnected();
	ui.cboPorts->setEnabled(!connected);
	ui.listBauds->setEnabled(connected);
	ui.btnConnect->setEnabled(!connected && ui.cboPorts->currentIndex() >= 0);
	ui.btnDisconnect->setEnabled(connected);
	ui.btnScan->setEnabled(connected && checkedItems(ui.listBauds) > 0);
}


void ConnectivityFrame::onConnectClicked(void)
{
	iface->OpenPort(ui.cboPorts->itemData(ui.cboPorts->currentIndex()).toString());
	EnableControls();
}


void ConnectivityFrame::onDisconnectClicked(void)
{
	onStopScanClicked();
	iface->ClosePort();
	EnableControls();
}

void ConnectivityFrame::onStopScanClicked(void)
{
	if (!scanThread.isNull())
	{
		scanThread->Abort();
	}
}

void ConnectivityFrame::onScanClicked(void)
{
	// Launch this in a worker thread
	std::list<int> bauds;
	for (int i = 0; i < ui.listBauds->count(); i++)
	{
		QListWidgetItem* item = ui.listBauds->item(i);
		if (item->checkState() == Qt::Checked)
		{
			//QString baud = item->text();
			int baudRate = item->data(Qt::UserRole).toInt();
			bauds.push_back(baudRate);
		}
	}
	// Port will be reopened on thread
	iface->ClosePort();

	scanThread = QSharedPointer<ScanDevicesThread>(new ScanDevicesThread(bauds, iface));
	connect(scanThread.data(), SIGNAL(ScanChange(int, int)), SLOT(onScanChanged(int, int)));
	connect(scanThread.data(), SIGNAL(FoundDevice(int, int, int)), SLOT(onFoundDevice(int, int, int)));
	connect(scanThread.data(), SIGNAL(finished()), SLOT(onScanFinished()));
	scanThread->start();
}

void ConnectivityFrame::onScanFinished()
{
	ui.lblScan->setText("");
	scanThread = nullptr;
	iface->ReopenPort();
	EnableControls();
}

void ConnectivityFrame::onScanChanged(int baud, int id)
{
	ui.lblScan->setText(QString("Baud=%1 id=%2").arg(baud).arg(id));
	emit ScanChange(baud, id);
}

void ConnectivityFrame::onFoundDevice(int baud, int id, int model)
{
	emit FoundDevice(baud, id, model);
}

void ConnectivityFrame::writeSettings(QSettings& qsettings)
{
	// Save baud list
	qsettings.beginWriteArray("selectedBauds");
	int index = 0;
	for (int i = 0; i < ui.listBauds->count(); i++)
	{
		QListWidgetItem* item = ui.listBauds->item(i);
		if (item->checkState() == Qt::Checked)
		{
			qsettings.setArrayIndex(index);
			qsettings.setValue("baud", item->data(Qt::UserRole).toInt());
			index++;
		}
	}
	qsettings.endArray();
	qsettings.setValue("port", ui.cboPorts->currentData(Qt::UserRole).toString());
}

void ConnectivityFrame::readSettings(QSettings& qsettings)
{
	// Save baud list
	int size = qsettings.beginReadArray("selectedBauds");
	for (int j = 0; j < size; j++)
	{
		qsettings.setArrayIndex(j);
		int baud = qsettings.value("baud").toInt();
		for (int i = 0; i < ui.listBauds->count(); i++)
		{
			QListWidgetItem* item = ui.listBauds->item(i);
			if ( item->data(Qt::UserRole).toInt() == baud)
			{
				item->setCheckState(Qt::Checked);
				break;
			}
		}
	}
	qsettings.endArray();

	QString port = qsettings.value("port").toString();
	for (int i = 0; i < ui.cboPorts->count(); i++)
	{
		if (ui.cboPorts->itemData(i, Qt::UserRole).toString() == port)
		{
			ui.cboPorts->setCurrentIndex( i );
			break;
		}
	}
}

