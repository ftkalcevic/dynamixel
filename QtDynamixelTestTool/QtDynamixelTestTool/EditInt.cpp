#include "EditInt.h"
#include <QDebug>

EditInt::EditInt(QWidget *parent, int address, int size, int value, int min, int max, QString name, QString desc, bool readOnly)
	: QFrame(parent)
{
	ui.setupUi(this);
	ui.groupBox->setTitle(name);
	ui.label->setText(desc);
	ui.pushButton->setEnabled(!readOnly);
	ui.spinBox->setReadOnly(readOnly);
	if ( min >= 0 )
		ui.spinBox->setMinimum(min);
	else
		ui.spinBox->setMinimum(0);
	if ( max >= 0 )
		ui.spinBox->setMaximum(max);
	else
		ui.spinBox->setMaximum((1<<(size*8))-1);
	ui.spinBox->setValue(value);

	connect(ui.pushButton, &QPushButton::clicked, [=]()
		{
			int newValue = ui.spinBox->value();
			emit DataChanged(address, size, newValue);
		});

}

EditInt::~EditInt()
{
}
