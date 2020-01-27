#pragma once

#include <QFrame>
#include "ui_EditEnum.h"
#include "Device.h"

class EditEnum : public QFrame
{
	Q_OBJECT

public:
	EditEnum(QWidget *parent, int address, int size, int value, const QList<EnumData> &enums, QString name, QString desc, bool readOnly);
	~EditEnum();

signals:
	void DataChanged(int address, int size, int value);

private:
	Ui::EditEnum ui;
};
