#include "EditEnum.h"
#include <QDebug>

EditEnum::EditEnum(QWidget* parent, int address, int size, int value, const QList<EnumData>& enums, QString name, QString desc, bool readOnly)
	: QFrame(parent)
{
	ui.setupUi(this);

	ui.groupBox->setTitle(name);
	ui.label->setText(desc);
	ui.comboBox->setEnabled(!readOnly);
	ui.pushButton->setEnabled(!readOnly);
	for each (auto e in enums)
	{
		ui.comboBox->addItem(e.text, e.id);
		if (e.id == value)
			ui.comboBox->setCurrentIndex(ui.comboBox->count()-1);
	}

	connect(ui.pushButton, &QPushButton::clicked, [=]()
		{
			int newValue = ui.comboBox->currentData().toInt();
			emit DataChanged(address, size, newValue);
		});
}

EditEnum::~EditEnum()
{
}
