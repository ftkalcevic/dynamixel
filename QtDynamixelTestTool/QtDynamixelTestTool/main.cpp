#include "QtDynamixelTestTool.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QtDynamixelTestTool w;
	w.show();
	return a.exec();
}
