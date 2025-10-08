#ifndef SERIALPORT_H
#define SERIALPORT_H

class serialport
{
public:
    serialport();
   int open_port(const char* portName);
   int set_port(int fd,int iBaudRate,int iDataSize,char cParity,int iStopBit);
   int read_port(int fd,char *buf,int iByte);
   int write_port(int fd,unsigned char *buf,int iByte);
   int close_port(int fd);
};

#endif // SERIALPORT_H
