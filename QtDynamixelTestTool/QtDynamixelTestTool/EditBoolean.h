#pragma once

#include <QFrame>
#include "ui_EditBoolean.h"

class EditBoolean : public QFrame
{
	Q_OBJECT

public:
	EditBoolean(QWidget* parent, int address, int size, int value, QString name, QString desc, bool readOnly);
	~EditBoolean();

signals:
	void DataChanged(int address, int size, int value);

private:
	Ui::EditBoolean ui;
};
