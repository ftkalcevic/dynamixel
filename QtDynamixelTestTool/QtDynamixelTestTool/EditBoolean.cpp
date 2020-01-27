#include "EditBoolean.h"
#include <QDebug>

EditBoolean::EditBoolean(QWidget *parent, int address, int size, int value, QString name, QString desc, bool readOnly)
	: QFrame(parent)
{
	ui.setupUi(this);
	ui.groupBox->setTitle(name);
	ui.label->setText(desc);
	ui.checkBox->setEnabled(!readOnly);
	ui.pushButton->setVisible(!readOnly);
	ui.checkBox->setCheckState(value != 0 ? Qt::Checked : Qt::Unchecked);

	connect(ui.pushButton, &QPushButton::clicked, [this, address, size]()
		{
			int newValue = ui.checkBox->checkState() == Qt::Checked ? 1 : 0;
			emit DataChanged(address, size, newValue);
		});
}

EditBoolean::~EditBoolean()
{
}

