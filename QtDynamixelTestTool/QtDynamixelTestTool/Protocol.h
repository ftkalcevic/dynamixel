#pragma once

#include <QtCore/QString>
#include <list>
#include "Baud.h"

class Protocol
{
public:
	Protocol();
	~Protocol();

	virtual std::list<Baud> getBauds() const = 0;
	
	int minDeviceId() const { return 0; }
	int maxDeviceId() const { return 0xFC; }
};
