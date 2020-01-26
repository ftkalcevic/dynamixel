#include "Protocol1.h"

Protocol1::Protocol1()
{
	bauds.push_back(Baud("9,600", 9600, 207));
	bauds.push_back(Baud("19,200", 19200, 103));
	bauds.push_back(Baud("57,600", 57600, 34));
	bauds.push_back(Baud("115,200", 115200, 16));
	bauds.push_back(Baud("200k", 200000, 9));  
	bauds.push_back(Baud("250k", 250000, 7));
	bauds.push_back(Baud("400k", 400000, 4));
	bauds.push_back(Baud("500k", 500000, 3));
	bauds.push_back(Baud("1M", 1000000, 1));
	bauds.push_back(Baud("2M", 2000000, 0));
	bauds.push_back(Baud("2.25M", 2250000, 250));
	bauds.push_back(Baud("2.5M", 2500000, 251));
	bauds.push_back(Baud("3M", 3000000, 252));
}

Protocol1::~Protocol1()
{
}

std::list<Baud> Protocol1::getBauds() const
{
	return bauds;
}
