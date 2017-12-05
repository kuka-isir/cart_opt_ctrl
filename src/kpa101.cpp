#include "cart_opt_ctrl/kpa101.h"
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>


KPA101::KPA101()
{

}

bool KPA101::open_device(std::string serial)
{
	//std::cout << "Starting communication with KPA101...";
	m_deviceHandle = open("/dev/ttyUSB0", O_RDWR |O_NOCTTY | O_NONBLOCK);

	if(m_deviceHandle < 0)
	{
		std::cerr << "Unable to open KPA101 communication." << std::endl;
		return false;
	}
	//std::cout << "OK" << std::endl;

	return true;
}

bool KPA101::close_communication()
{
	//std::cout << "Closing communication with KPA101...";
	if(close(m_deviceHandle) < 0)
	{
		std::cerr << "Unable to close KPA101 communication." << std::endl;
		return false;
	}
	//std::cout << "OK" << std::endl;


	return true;
}


bool KPA101::init_communication()
{
	struct termios tty;
	int flags;
	memset (&tty, 0, sizeof(tty));
	if(tcgetattr (m_deviceHandle, &tty) != 0)
	{
		std::cerr << "Unable to get current KPA101 parameters." << std::endl;
		return false;
	}

	cfsetospeed (&tty, B115200);
	cfsetispeed (&tty, B115200);

	// 8-bit chars
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

	// Setting no parity, 1 stop bit
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~PARODD;
	tty.c_cflag &= ~CSTOPB;

	usleep(500);

	tcflush(m_deviceHandle,TCIOFLUSH);

	usleep(500);

	// Set RTS
	ioctl(m_deviceHandle, TIOCMGET, &flags);
	flags |= TIOCM_RTS ;
	ioctl(m_deviceHandle, TIOCMSET, &flags);

	// Set flow control to RTS/CTS
	tty.c_cflag |= CRTSCTS;

	if(tcsetattr (m_deviceHandle, TCSANOW, &tty) != 0)
	{
		std::cerr << "Unable to set KPA101 communication parameters." << std::endl;
		return false;
	}

    //std::cout << "Serial communication set." << std::endl;
    return true;
}

bool KPA101::init_device()
{
	std::cout << "Init KPA101...";

	// Set K-Cube in open loop
	/*char txBuf[14] = {0x70,0x08,0x04,0x00,0x50,0x01,0x07,0x00,0x02,0x00};

	int n = write(m_deviceHandle,txBuf,14);
	if(n < 0)
	{
		std::cerr << "init_device::Write error, " << strerror(errno) << "("<< errno << ")" << std::endl;
		return false;
	}
	if(n != 14)
	{
		std::cerr << "Error while sending init sequence to KPA101." << std::endl;
		return false;
	}

	usleep(150000);*/

	// 50: generic usb
	// 21 bay 0
	char blinkBuf[6] = {0x23,0x02,0x00,0x00,0x50,0x01};
	int n = write(m_deviceHandle,blinkBuf,6);
	if(n < 0)
	{
		std::cerr << "init_device::Write error, " << strerror(errno) << "("<< errno << ")" << std::endl;
		return false;
	}
	if(n != 6)
	{
		std::cerr << "Error while sending blink sequence to KPA101." << std::endl;
		return false;
	}

	usleep(150000);

	std::cout << "OK" << std::endl;

	return true;
}


bool KPA101::print_device_infos()
{
	char txBuf[6] = {0x05,0x00,0x00,0x00,0x50,0x01};
	int n = write(m_deviceHandle,txBuf,6);
	if(n < 0)
	{
		std::cerr << "print_device_infos::Write error, " << strerror(errno) << "("<< errno << ")" << std::endl;
		return false;
	}
	if(n != 6)
	{
		std::cerr << "Unable to send info request to KPA101." << std::endl;
		return false;
	}

	usleep(150000);

	char rxBuf[90];
	n = read(m_deviceHandle, rxBuf, 90);
	if(n < 0)
	{
		std::cerr << "print_device_infos::Read error, " << strerror(errno) << "("<< errno << ")" << std::endl;
		return false;
	}
	if(n != 90)
	{
		std::cerr << "Wrong KPA101 infos length received." << std::endl;
		return false;
	}

	char firmwareVersion[32];
	sprintf(firmwareVersion,"%d.%d.%d",rxBuf[22],rxBuf[21],rxBuf[20]);
	std::cout << "Firmware version: " << firmwareVersion << std::endl;

	char modelNumber[8];
	memcpy(modelNumber,rxBuf+10,8);
	std::cout << "Model number: " << modelNumber << std::endl;

	char deviceNotes[48];
	memcpy(deviceNotes,rxBuf+24,48);
	std::cout << "Device notes: " << deviceNotes << std::endl;
}

bool KPA101::get_data(unsigned int timeout, float* xpos, float* ypos, float* sum)
{
	// Send data request to the K-Cube
	char txBuf[6] = {0x71,0x08,0x03,0x00,0x50,0x01};
	int n = write(m_deviceHandle,txBuf,6);
	if(n < 0)
	{
		std::cerr << "get_data::Write error, " << strerror(errno) << "("<< errno << ")" << std::endl;
		return false;
	}
	if(n != 6)
	{
		std::cerr << "Unable to send data request to KPA101." << std::endl;
		return false;
	}

	// Wait for answer
	char rxBuf[18];

	usleep(10000);

	n = read (m_deviceHandle, rxBuf, 18);
	if(n < 0)
	{
		std::cerr << "get_data::Read error, " << strerror(errno) << "("<< errno << ")" << std::endl;
		return false;
	}
	if(n != 18)
	{
		std::cerr << "Wrong KPA101 message length received." << std::endl;
		return false;
	}

	unsigned short m = ((rxBuf[7] << 8) | rxBuf[6]);
	if(m != 3)
	{
		std::cerr << "Wrong KPA101 message received." << std::endl;
		return false;
	}

	// Else populate xpos, ypos and sum
	short *xp = (short int*)(rxBuf+8);//((rxBuf[15] << 8) | rxBuf[14]);;
	short *yp = (short int*)(rxBuf+10);//((rxBuf[17] << 8) | rxBuf[16]);
	unsigned short *s = (unsigned short int*)(rxBuf+12);//((rxBuf[13] << 8) | rxBuf[12]);

	float tmpsum = 10.0*(*s)/65535.0;
	float tmpx = 10.0*(*xp)/32767.0;
	float tmpy = 10.0*(*yp)/32767.0;

	tmpx = (10*tmpx)/(2*tmpsum);
	tmpy = (10*tmpy)/(2*tmpsum);

	*sum = tmpsum;
	*xpos = tmpx;
	*ypos = tmpy;

	return true;
}
