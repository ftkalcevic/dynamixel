#pragma once

#include <QtCore/QThread>
#include <list>
#include "SerialInterface.h"

class ScanDevicesThread : public QThread
{
	Q_OBJECT

	std::list<int> baudList;
	QSharedPointer<SerialInterface> iface;
	bool abort;

signals:
	void ScanChange(int baud, int id);
	void FoundDevice(int baud, int id, int model);

public:
	ScanDevicesThread(std::list<int> bauds, QSharedPointer<SerialInterface> iface, QObject *parent=NULL);
	~ScanDevicesThread();

	virtual void run();
	void Abort();
};
