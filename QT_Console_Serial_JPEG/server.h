#ifndef SERVER_H
#define SERVER_H

#include <QObject>
#include <QCoreApplication>
#include <QtNetwork>
#include <QUdpSocket>
#include <QTimer>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <QSerialPort>
#include <QImage>
class server : public QObject
{
    Q_OBJECT
private:
     QCoreApplication *app;
     QByteArray pixels;
     QUdpSocket* UDPSock;

     //v4l2 objects
     int fd;
     v4l2_capability capability;
     v4l2_format imageFormat;
     v4l2_requestbuffers requestBuffer;
     v4l2_buffer queryBuffer;
     char* buffer;
     v4l2_buffer bufferinfo;
     int type;
     int initCamera();
     int initRobot();
     QByteArray fromYUY(QByteArray pixels);
     QUdpSocket* comUDP;
     QTimer* eventTimer;
     QSerialPort* robot;
     int frameNumber;
     void getTelemetry();
     qreal pitch;
     qreal yaw;
     QString Controller_IP;
     bool isConnected;
     void textDebug(QString outText);
     char* buffer2;  //trying to fix memory problem
public:
    explicit server(QObject *parent = nullptr);
     void quit();
signals:
     void finished();
public slots:
     void run();
     void aboutToQuitApp();
     void recCommand();
     void sendFrame();
     void readSerial();
};

#endif // SERVER_H
