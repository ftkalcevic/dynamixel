#pragma once

#include <QFrame>
#include "ui_EditInt.h"

class EditInt : public QFrame
{
	Q_OBJECT

public:
	EditInt(QWidget *parent, int address, int size, int value, int min, int max, QString name, QString desc, bool readOnly);
	~EditInt();

signals:
	void DataChanged(int address, int size, int value);

private:
	Ui::EditInt ui;
};
