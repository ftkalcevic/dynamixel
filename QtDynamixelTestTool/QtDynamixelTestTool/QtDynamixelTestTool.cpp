#include "QtDynamixelTestTool.h"
#include <QtCore/QDebug>
#include <QtCore/QSettings>


QtDynamixelTestTool::QtDynamixelTestTool(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	iface = QSharedPointer<SerialInterface>(new SerialInterface(this));

	ui.frameConnectivity->InitialiseData(iface);

	connect(ui.frameConnectivity, SIGNAL(FoundDevice(int, int, int)), SLOT(onFoundDevice(int, int, int)));

	readPositionSettings();
}

void QtDynamixelTestTool::closeEvent(QCloseEvent*)
{
	writePositionSettings();
}

void QtDynamixelTestTool::readPositionSettings()
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
}

void QtDynamixelTestTool::writePositionSettings()
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
}


void QtDynamixelTestTool::onFoundDevice(int baud, int id, int model)
{
	// Find or create baud node
	QString baudText = QString::number(baud);
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
		ui.treeDevices->insertTopLevelItem(0, parent);
	}

	// Append device
	QString desc = QString("#%2 %3").arg(id).arg(model);
	QTreeWidgetItem* deviceItem = new QTreeWidgetItem(parent, QStringList(desc));
	//ui.treeDevices->insertTopLevelItem(0, deviceItem);
}

/*

- Serial Ports
- Serial Interface
	- protocol
		- baud rates
	- scan for devices
	- Devices



*/