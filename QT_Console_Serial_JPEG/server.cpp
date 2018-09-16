#include "server.h"
#include "stdio.h"
#include <QtNetwork>
#include <QtCore>
#include <QTextStream>
#include <QFile>
#include <QImage>
#include <iostream>
 #include <stdio.h>
 #include <stdlib.h>
 #include <linux/ioctl.h>
 #include <linux/types.h>
 #include <linux/v4l2-common.h>
 #include <linux/v4l2-controls.h>
 #include <linux/videodev2.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <sys/ioctl.h>
 #include <sys/mman.h>
 #include <string.h>
 #include <fstream>
 #include <string>
 #include <const_defines.h>


server::server(QObject *parent) : QObject(parent)
{
 app = QCoreApplication::instance();
 comUDP = new QUdpSocket(this);
 comUDP->bind(3491, QUdpSocket::ShareAddress);
   robot = new QSerialPort("ttyUSB0");

 eventTimer = new QTimer(this);
 connect(eventTimer, &QTimer::timeout, this, &server::sendFrame);
 connect(comUDP, &QUdpSocket::readyRead, this, &server::recCommand);
 connect(robot, &QSerialPort::readyRead, this, &server::readSerial);
 buffer2 = new char[IMAGE_W * IMAGE_H * 2];
 isConnected = false;

}

void server::sendFrame()
{

        // Queue the buffer
         if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
             perror("Could not queue buffer, VIDIOC_QBUF");
       //      return 1;
         }


         // Dequeue the buffer
         if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0){
             perror("Could not dequeue the buffer, VIDIOC_DQBUF");
         //    return 1;
         }

         //char* buffer2 = new char[sizeof(char) * bufferinfo.bytesused];
         if (bufferinfo.bytesused != IMAGE_H*IMAGE_W*2){
             perror("Buffer length incorrect");
             return;
         }
         memcpy(buffer2, buffer, bufferinfo.bytesused);
         QByteArray pixels = QByteArray::fromRawData(buffer2, bufferinfo.bytesused);
         QByteArray curImage = fromYUY(pixels);

         //now let's just toss the file to the wind
         int bytesRead = 0;
         frameNumber++;
         for (int i = 0; i < curImage.size()/BLOCK_SIZE; i++){

             QByteArray curByte;
             QDataStream output(&curByte, QIODevice::WriteOnly);
             output << (int) frameNumber;
             output << (int) curImage.size();
             output << (int) (i * BLOCK_SIZE);
             output << curImage.mid(i*BLOCK_SIZE, BLOCK_SIZE);
             bytesRead = bytesRead + curByte.size();
             UDPSock->writeDatagram(curByte.data(), curByte.size(), QHostAddress(Controller_IP),(quint16) 3490);
         }
         getTelemetry();
         eventTimer->setInterval(UPDATE_TIME);
         eventTimer->start();


 }

void server::getTelemetry()
{
    QByteArray toSerial;
    QDataStream toSerStream(&toSerial, QIODevice::WriteOnly);

    toSerStream << (quint8) SEND_PITCH;
    toSerStream << (quint8) SEND_YAW;
    toSerStream << (quint8) SEND_SETTINGS;
    robot->write(toSerial);

   // textDebug("Asking for telemetry");
}
void server::readSerial()
{
    QByteArray fromSerial;
    QDataStream serialStream(&fromSerial, QIODevice::ReadOnly);
    QByteArray toHost;
    QDataStream toHostStream(&toHost, QIODevice::WriteOnly);
    fromSerial =  robot->readAll();

    union flByte{
        float val;
        quint8 bytes[4];
    } nextByte;
    int z = 0;

    while (z < fromSerial.size())
    {
        //Check to make sure there are enough bytes
        //Then check to make sure the data type was encoded
        //if so convert and send to the controller
        bool success = false;
        if (fromSerial.size() - z > 8){
                quint8 oneByte;
                quint8 eightBytes[8];

                for (int i = 0; i < 8; i++){
                    serialStream >> eightBytes[i];
                }
                if (eightBytes[0] == eightBytes[2] && eightBytes[1] == 0 && eightBytes[3] ==0)
                  {
                    nextByte.bytes[0] = eightBytes[4];
                    nextByte.bytes[1] = eightBytes[5];
                    nextByte.bytes[2] = eightBytes[6];
                    nextByte.bytes[3] = eightBytes[7];
                    qreal outFloat = (qreal) nextByte.val;
                    toHostStream << eightBytes[0] << outFloat;
                    UDPSock->writeDatagram(toHost, toHost.size(), QHostAddress(Controller_IP), (qint16) PORT_TELEMETRY);
                    success = true;
                  }
         }
        if (success){ z = z + 8;} else {
            z = z + 1;
            toHostStream << REC_ERROR << (float) 0;
            UDPSock->writeDatagram(toHost, toHost.size(), QHostAddress(Controller_IP), (qint16) PORT_TELEMETRY);
        }
    }
}
int server::initCamera()
{
    frameNumber = 0;
    // open device
    fd = open("/dev/video0",O_RDWR);
      if(fd < 0){
        //  perror("Failed to open device, OPEN");
          return 1;
      }

      if(ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0){
             // something went wrong... exit
          //   perror("Failed to get device capabilities, VIDIOC_QUERYCAP");
             return 1;
         }

      //set the format
         imageFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         imageFormat.fmt.pix.width = IMAGE_W;
         imageFormat.fmt.pix.height = IMAGE_H;
         imageFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
         imageFormat.fmt.pix.field = V4L2_FIELD_NONE;
         // tell the device you are using this format
         if(ioctl(fd, VIDIOC_S_FMT, &imageFormat) < 0){
  //           perror("Device could not set format, VIDIOC_S_FMT");
             return 1;
         }


         requestBuffer.count = 1; // one request buffer
         requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // request a buffer wich we an use for capturing frames
         requestBuffer.memory = V4L2_MEMORY_MMAP;
         if(ioctl(fd, VIDIOC_REQBUFS, &requestBuffer) < 0){
            //     perror("Could not request buffer from device, VIDIOC_REQBUFS");
                 return 1;
             }

         queryBuffer = {0};
         queryBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            queryBuffer.memory = V4L2_MEMORY_MMAP;
            queryBuffer.index = 0;
            if(ioctl(fd, VIDIOC_QUERYBUF, &queryBuffer) < 0){
              //  perror("Device did not return the buffer information, VIDIOC_QUERYBUF");
                return 1;
            }
            // use a pointer to point to the newly created buffer
                // mmap() will map the memory address of the device to
                // an address in memory
                buffer = (char*)mmap(NULL, queryBuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                   fd, queryBuffer.m.offset);
                memset(buffer, 0, queryBuffer.length);

                memset(&bufferinfo, 0, sizeof(bufferinfo));
                   bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                   bufferinfo.memory = V4L2_MEMORY_MMAP;
                   bufferinfo.index = 0;

                   type = bufferinfo.type;
                       if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
                           perror("Could not start streaming, VIDIOC_STREAMON");
                           return 1;
                       }
            eventTimer->start();
            return 0; //success
}
void server::run()
{
    QTextStream output(stdout);
    QTextStream input(stdin);
    //Prepare the video stream; we may need to try more than once
    bool keepTrying = true;
    int tryNum = 0;
    bool success = false;
    output << "Aquiring camera..." << endl;
    while (keepTrying)
    {
        int worked = initCamera();
        if (worked == 0) {
            success = true;
            keepTrying = false;
        } else {
            output << "Failed to aquire, trying again..." << endl;
        }
        tryNum++;
        if (tryNum > NUM_OF_TRIES){
            keepTrying = false;
        }
     }

    if (!success){
        output << "Failed to aquire video device";
        quit();

    }


    output << "Width: " << imageFormat.fmt.pix.width << endl;
    output << "Height: " << imageFormat.fmt.pix.height << endl;
    output << "Bytes per line: " << imageFormat.fmt.pix.bytesperline << endl;
    output << "Format: " << imageFormat.fmt.pix.pixelformat << endl;
    output << "Press 'q' to quit" << endl;

    //declare the UDP server
    if (!initRobot()){
        output << "Couldn't find robot" << endl;
        quit();
    }


    UDPSock = new QUdpSocket(this);

    output << "Waiting For Connection...";
    QString inStr = "";

 }

void server::quit(){

    emit finished();
}

void server::aboutToQuitApp(){

}
int server::initRobot(){

    robot->setBaudRate(9600);
    if (robot->open(QIODevice::ReadWrite)) { return 1;} else {return 0;};
}
void server::recCommand()
{
    QTextStream output(stdout);
    QByteArray curData;
    QDataStream input(&curData, QIODevice::ReadOnly);
    QByteArray outMessg;
    QDataStream toBot(&outMessg, QIODevice::WriteOnly);

    curData.resize(comUDP->pendingDatagramSize());
    comUDP->readDatagram(curData.data(), curData.size());
    quint8 message;
    input >> message;
    bool connected = false;
    switch(message){
        case SET_CONTROLLER_IP:
            input >> Controller_IP;
            if (!isConnected){
                isConnected = true;
                output << "Connecting to " << Controller_IP;
                sendFrame();
            }
        break;
        case RESET_ARDUINO:
             output << "Resetting Arduino";
             toBot << message;
             connected = false;
             robot->close();
             while (!connected){
                 output << "Reconnecting";
                 if (initRobot() == 1){connected = true;}
             }
        break;
        default:
            output << message;
            toBot << message;
            robot->write(outMessg);
        break;
    }



}
void server::textDebug(QString outText)
{
    QTextStream output(stdout);
    output << outText;
}

QByteArray server::fromYUY(QByteArray pixels){
    QImage myImage = QImage(IMAGE_W,IMAGE_H,QImage::Format_ARGB32);

    myImage.fill(QColor(0,0,0));
    for (int row = 0; row < IMAGE_H; row++){
        for (int col = 0; col < IMAGE_W/2; col++){
            //YUY2 format, every 4 bytes encodes 2 pixels
            int curPix = row*2*IMAGE_W+col*4;
            uint8_t Y1 = pixels[curPix];
            uint8_t Cb = pixels[curPix +1];
            uint8_t Y2 = pixels[curPix + 2];
            uint8_t Cr = pixels[curPix + 3];

            int R1 = 1.0*(Y1-0) + 0*(Cb-128) + 1.4*(Cr-128);
            int G1 = 1.0*(Y1-0) + -.343*(Cb-128) + -.711*(Cr-128);
            int B1 = 1.0*(Y1-0) + 1.765*(Cb-128) + 0*(Cr-128);
            int R2 = 1.0*(Y2-0) + 0*(Cb-128) + 1.4*(Cr-128);
            int G2 = 1.0*(Y2-0) + -.343*(Cb-128) + -.711*(Cr-128);
            int B2 = 1.0*(Y2-0) + 1.765*(Cb-128) + 0*(Cr-128);

            QRgb curVal1 = qRgb(R1, G1, B1);
            QRgb curVal2 = qRgb(R2, G2, B2);
            myImage.setPixel( col*2,row, curVal1);
            myImage.setPixel(col*2+1, row, curVal2);
        }
       }

     QByteArray asJpeg;
     QBuffer buffer1(&asJpeg);
     buffer1.open(QIODevice::WriteOnly);
     myImage.save(&buffer1, "JPEG");
     buffer1.close();
    return(asJpeg);
}
