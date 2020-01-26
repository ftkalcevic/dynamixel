#include "QtDynamixelTestTool.h"
#include <QtCore/QDebug>
#include <QtCore/QSettings>


QtDynamixelTestTool::QtDynamixelTestTool(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	ui.frameConnectivity->setEnabled(true);
	ui.frameConnectivity->setVisible(true);
	ui.frameDevice->setEnabled(true);
	ui.frameDevice->setVisible(true);

	iface = QSharedPointer<SerialInterface>(new SerialInterface(this));
	ui.frameConnectivity->InitialiseData(iface);

	connect(ui.frameConnectivity, SIGNAL(ScanStart()), SLOT(onScanStart()));
	connect(ui.frameConnectivity, SIGNAL(FoundDevice(int, int, int)), SLOT(onFoundDevice(int, int, int)));
	connect(ui.treeDevices, SIGNAL(itemSelectionChanged()), SLOT(onDeviceSelectionChanged()) );
	readSettings();
	EnableControls();
}

void QtDynamixelTestTool::closeEvent(QCloseEvent*)
{
	writeSettings();
}

void QtDynamixelTestTool::readSettings()
{
	QSettings qsettings("FranksWorkshop", "QtDynamixelTestTool");

	qsettings.beginGroup("mainwindow");

	restoreGeometry(qsettings.value("geometry", saveGeometry()).toByteArray());
	restoreState(qsettings.value("savestate", saveState()).toByteArray());
	move(qsettings.value("pos", pos()).toPoint());
	resize(qsettings.value("size", size()).toSize());
	if (qsettings.value("maximized", isMaximized()).toBool())
		showMaximized();

	QList<int> sizes;
	sizes.append( qsettings.value("splitterSize1").toInt() );
	sizes.append( qsettings.value("splitterSize2").toInt());
	ui.splitter->setSizes(sizes);

	qsettings.endGroup();

	ui.frameConnectivity->readSettings(qsettings);
	ui.frameDevice->readSettings(qsettings);
}

void QtDynamixelTestTool::writeSettings()
{
	QSettings qsettings("FranksWorkshop", "QtDynamixelTestTool");

	qsettings.beginGroup("mainwindow");

	qsettings.setValue("geometry", saveGeometry());
	qsettings.setValue("savestate", saveState());
	qsettings.setValue("maximized", isMaximized());
	if (!isMaximized()) {
		qsettings.setValue("pos", pos());
		qsettings.setValue("size", size());
	}
	QList<int> sizes = ui.splitter->sizes();
	qsettings.setValue("splitterSize1", sizes[0]);
	qsettings.setValue("splitterSize2", sizes[1]);

	qsettings.endGroup();

	ui.frameConnectivity->writeSettings(qsettings);
	ui.frameDevice->writeSettings(qsettings);
}


void QtDynamixelTestTool::onScanStart()
{
	ui.treeDevices->clear();
}

void QtDynamixelTestTool::onFoundDevice(int baud, int id, int model)
{
	// Find or create baud node
	QString baudText = QString::number(baud) + " bps";
	QTreeWidgetItem* parent = nullptr;
	for (int i = 0; i < ui.treeDevices->topLevelItemCount(); i++)
	{
		if (ui.treeDevices->topLevelItem(i)->text(0) == baudText)
		{
			parent = ui.treeDevices->topLevelItem(i);
			break;
		}
	}
	if (!parent)
	{
		parent = new QTreeWidgetItem((QTreeWidget*)nullptr, QStringList(baudText));
		ui.treeDevices->insertTopLevelItem(ui.treeDevices->topLevelItemCount(), parent);
	}
	ui.treeDevices->expandAll();


	// Append device
	QString modelName = "Unknown";
	if (iface->getDevices().contains(model))
	{
		modelName = iface->getDevices().value(model).name;
	}

	QString desc = QString("[ID:%1] %2").arg(id).arg(modelName);
	QTreeWidgetItem* deviceItem = new QTreeWidgetItem(parent, QStringList(desc));
	deviceItem->setData(0, Qt::UserRole, model);
}


void QtDynamixelTestTool::onDeviceSelectionChanged()
{
	EnableControls();

	if (ui.treeDevices->selectedItems().count() > 0)
	{
		QTreeWidgetItem* item = ui.treeDevices->selectedItems().first();
		ui.frameDevice->InitialiseData(iface,item->data(0,Qt::UserRole).toInt());
	}
}

void QtDynamixelTestTool::EnableControls()
{
	bool hasSelection = ui.treeDevices->selectedItems().count() != 0;
	ui.frameConnectivity->setEnabled(!hasSelection);
	ui.frameConnectivity->setVisible(!hasSelection);
	ui.frameDevice->setEnabled(hasSelection);
	ui.frameDevice->setVisible(hasSelection);
}


/*

- Serial Ports
- Serial Interface
	- protocol
		- baud rates
	- scan for devices
	- Devices



*/