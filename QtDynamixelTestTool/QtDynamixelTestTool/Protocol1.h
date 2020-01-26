#pragma once

#include "Protocol.h"

class Protocol1 : public Protocol
{
public:
	Protocol1();
	~Protocol1();

	// Inherited via Protocol
	virtual std::list<Baud> getBauds() const override;

private:
	std::list<Baud> bauds;
};
