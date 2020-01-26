#pragma once
#include <QtCore/QObject>

class Baud
{
public:
	Baud(QString desc, int value, int code);
	~Baud();

	QString desc;
	int value;
	int code;
};
