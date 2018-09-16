#include <QtCore/QCoreApplication>
#include <QTimer>
#include "server.h"
int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    // create the main class
    server myServer;

    // connect up the signals
    QObject::connect(&myServer, SIGNAL(finished()),
             &app, SLOT(quit()));
    QObject::connect(&app, SIGNAL(aboutToQuit()),
             &myServer, SLOT(aboutToQuitApp()));

    // This code will start the messaging engine in QT and in
    // 10ms it will start the execution in the MainClass.run routine;
    QTimer::singleShot(10, &myServer, SLOT(run()));

    return app.exec();
}
