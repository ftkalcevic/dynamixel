#include "ScanDevicesThread.h"
#include <QtCore/QDebug>

ScanDevicesThread::ScanDevicesThread(std::list<int> bauds, QSharedPointer<SerialInterface> iface, QObject *parent)
	: QThread(parent)
	, baudList(bauds)
	, iface(iface)
	, abort(false)
{
}

ScanDevicesThread::~ScanDevicesThread()
{
}

void ScanDevicesThread::run()
{
	iface->ReopenPort();

	// For each baud
	for each(int baudRate in baudList)
	{
		// Change baud rate
		iface->setBaud(baudRate);
		qDebug() << "Scanning baud " << baudRate;

		for (int id = iface->minDeviceId(); id <= iface->maxDeviceId(); id++)
		{
			emit ScanChange(baudRate, id);
			if (iface->Ping(id))
			{
				// bingo!
				qDebug() << "Found device id " << id;
				union {
					struct {
						uint16_t model;
						uint8_t firmware;
						uint8_t id;
						uint8_t baud;
						uint8_t returnDelayTime;
					};
					uint8_t data[];
				} buffer;
				if (iface->Read(id, 0, 6, buffer.data))
				{
					qDebug() << "Read device id " << id;
					qDebug() << "  model " << buffer.model;
					qDebug() << "  firmware " << buffer.firmware;
					qDebug() << "  id " << buffer.id;
					qDebug() << "  baud " << buffer.baud;
					qDebug() << "  return delay time " << buffer.returnDelayTime;
				}
				emit FoundDevice(baudRate,id,buffer.model);
			}

			if (abort)
			{
				qDebug() << "Scan aborted";
				iface->ClosePort();
				return;
			}
		}
	}
	iface->ClosePort();
	qDebug() << "Scan complete";
}

void ScanDevicesThread::Abort() 
{ 
	abort = true; 
	wait(500); // Block max 500ms for thread to terminate
}
