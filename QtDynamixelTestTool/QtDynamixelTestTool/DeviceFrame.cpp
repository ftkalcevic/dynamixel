#include "DeviceFrame.h"

DeviceFrame::DeviceFrame(QWidget *parent)
	: QFrame(parent)
{
	ui.setupUi(this);
	ui.tableDevice->setColumnCount(3);
	ui.tableDevice->setHorizontalHeaderLabels(QStringList() << "Addr" << "Description" << "Value" );
	
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

