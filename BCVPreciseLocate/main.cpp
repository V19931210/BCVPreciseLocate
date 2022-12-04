#include "MainWindow.h"
#include <QtWidgets/QApplication>
#include "vtkoutputwindow.h"

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);

    QApplication a(argc, argv);
    TMainWindow w;
    w.show();
    return a.exec();
}
