#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

// (rtwang20150824)
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <syslog.h>
static int detectReceivingData = 1;

// ablerex.jbus.c and ablerex.jbus.h (rtwang20150824)
unsigned int calculateCRC(unsigned char *msg, int size) {
  unsigned int crc = 0xFFFF;
  int i = 0;
  int j = 0;
  for (i = 0; i < size; i++) {
    crc ^= msg[i];
    for (j = 0; j < 8; j++) {
      if ((crc % 2) == 0) {
        crc >>= 1;
      } else {
        crc >>= 1;
        crc ^= 0xA001;
      }
    }
  }

  return crc;
}

void dumpRequest(unsigned char *request,int n) {
int i;	
for (i = 0; i < n; i++) {
		printf("request[%d] = %x\n",i, request[i]);
	}return;
}


void initRequest(unsigned char *request, int len, int addr, int mytime) {
	unsigned int crc;
	/*static int functioncode = 0x00;
	static int IDcode = 0x00;
	static int returnvalue = 0x00;
	static int addressnumber = 0x00;
	//ªì©l³]©w¨C¤@­ÓID
	//request[0] = slaveNumber;

	printf("\nEnter ID code : ");
	scanf("%x", &IDcode);
	printf("\nEnter function code : ");
	scanf("%x", &functioncode);
	printf("\nEnter the address  : ");
	scanf("%x", &addressnumber);
	if (functioncode == 0x06)
	{
		printf("\nEnter returnvalue: ");
		scanf("%x", &returnvalue);
	}
	else if (functioncode == 0x03)
	{
		returnvalue = 0x01;

	}
	request[0] = IDcode;
	request[1] = functioncode;
	request[2] = 0x00;
	request[3] = addressnumber; //0x02 .. 0x04

	request[4] = 0x00;
	request[5] = returnvalue;//Write: give data;  Read:How many register you want to read
	crc = calculateCRC(request, len - 2);
	//printf("crc = %x\n", crc);
	request[6] = crc & 0x00FF;
	request[7] = crc >> 8;*/
	int i = 0,test;
	for (i = 0;; i++)
	{
		printf("request[%d] = ", i);
		scanf("%x", &test);
		if (test == 0xff)break;
		request[i] = test;
	}
	crc = calculateCRC(request, i);
	request[i] = crc & 0x00FF;
	i++;
	request[i] = crc >> 8;
	dumpRequest(request, i+1);
	return;
}
int setupSerialPort(char devNumber, int baudRate, char *control) {
  int handle = 0;
  int brate;
  struct termios term;
  char dev[11] = "/dev/ttyS0";
  dev[9] = devNumber;

  //printf("open %s with %d\n", dev, baudRate);
  syslog(LOG_INFO, "open %s with %d\n", dev, baudRate);
  //open device
  handle = open(dev, O_RDWR|O_NOCTTY|O_NONBLOCK);

  if (handle <= 0) {
      //printf("Serial Init: Open failed\n");
      syslog(LOG_ERR, "Serial Init: Open failed\n");
      return -1;
  }

  switch (baudRate) {
      case 9600:
          brate = B9600;
          break;
      case 19200:
          brate = B19200;
          break;
      case 38400:
          brate = B38400;
          break;
      case 57600:
          brate = B57600;
          break;
      case 115200:
          brate = B115200;
          break;
      default:
        //printf("Serial Init: Set baudrate failed\n");
        syslog(LOG_ERR, "Serial Init: Set baudrate failed\n");
        return -1;
  }

  //get device struct
  if(tcgetattr(handle, &term) != 0) {
    //printf("Serial_Init:: tcgetattr() failed\n");
    syslog(LOG_ERR, "Serial_Init:: tcgetattr() failed\n");
    return -1;
  }

  //input modes
  term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|INPCK|ISTRIP|INLCR|IGNCR|ICRNL
                    |IXON|IXOFF);
  term.c_iflag |= IGNPAR;

  //output modes
  term.c_oflag &= ~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET|OFILL
                    |OFDEL|NLDLY|CRDLY|TABDLY|BSDLY|VTDLY|FFDLY);
  //control modes
    if(!strcmp(control, "-8n1")){
        if(1) printf("[%s:%d] 8n1\n",__FUNCTION__,__LINE__);
        term.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | HUPCL | CRTSCTS);
        term.c_cflag |= CREAD | CS8 | CLOCAL;
    }else if(!strcmp(control, "-odd")){ //8o1
        if(1) printf("[%s:%d] odd\n",__FUNCTION__,__LINE__);
        term.c_cflag &= ~(CSIZE | CSTOPB | HUPCL | CRTSCTS);
        term.c_cflag |= CREAD | CS8 | CLOCAL | PARENB | PARODD;
    }else if(!strcmp(control, "-even")){ //8e1
        if(1) printf("[%s:%d] even\n",__FUNCTION__,__LINE__);
        term.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | HUPCL | CRTSCTS);
        term.c_cflag |= CREAD | CS8 | CLOCAL | PARENB;
    }else{ // 8n2
        if(1) printf("[%s:%d] 8n2\n",__FUNCTION__,__LINE__);
        term.c_cflag &= ~(CSIZE | PARENB | PARODD | HUPCL | CRTSCTS);
        term.c_cflag |= CREAD | CS8 | CSTOPB | CLOCAL;
    }

  //local modes
  term.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO);
  term.c_lflag |= NOFLSH;

  //set baud rate
  cfsetospeed(&term, brate);
  cfsetispeed(&term, brate);

  //set new device settings
  if(tcsetattr(handle, TCSANOW, &term)  != 0) {
    //printf("Serial_Init:: tcsetattr() failed\n");
    syslog(LOG_ERR, "Serial_Init:: tcgetattr() failed.\n");
    return -1;
  }

  //printf("Serial_Init:: success\n");
  syslog(LOG_ERR, "Serial_Init:: success\n");

  return handle;
}

int main(int argc, char **argv) {
	int ret = 0;
	int i = 0;
	int mytime=0;
	int j = 0;
	int k = 0;
	time_t now = time(NULL);
	struct tm *tm;
	time_t previous = now;
	char* now_string;
	struct tm *pretm = localtime(&previous);
	char fullPathName[128] = "NULL";
	char fname[32];
	char dev = '1';
	char control[6];
	int baudRate[5] = {9600, 19200, 38400, 57600, 115200};
	int response_H,response_L;
	//unsigned char request[8] = { 0x07, 0x03, 0xC0, 0x20, 0x00, 0x0A, 0xF8, 0x61};
	unsigned char request[8];
	unsigned char response[256];
	double outputBuffer[22];
	int handle = 0;
	int option = -1;
	int optionIndex = 0;
	int delay = 555000;
	printf("\n 0 = 9600\n 1 = 19200\n 2 = 38400\n 3 = 57600\n 4 = 115200\n Select Baudrate: ");
	printf("Select ttyS?\n");
	scanf("%d %c %s", &j, &dev, control); // baudRate , ttyS_ , -8n1  (must use root)
	now = (unsigned)time(NULL);
	now_string = ctime(&now);
	//dev = getchar();
	//scanf("%c",&dev);
	if ((handle = setupSerialPort(dev, baudRate[j], control)) < 0) {
		printf("Error in open /dev/ttyS%c in %d\n", dev, baudRate[j]);
		syslog(LOG_ERR, "Error in open /dev/ttyS%c in %d\n", dev, baudRate[j]);
		closelog();
		exit(EXIT_SUCCESS);
	}

	printf("Open create the Serial Port\n");
	printf("========================================\n");

	/* The Big Loop */
	while (detectReceivingData) {
		/* Do some task here ... */
		
		initRequest(request, sizeof(request), i, mytime++);
		printf("\nSet Delay (in u second):");
		scanf("%d",&delay);
		ret = write(handle, request, sizeof(request)); 
		printf("ret = %d\n", ret);
		usleep(delay);
		memset(response, 0, sizeof(response));
		ret = read(handle, response, 256);                

		printf("%s ret = %d\n", now_string, ret);
		if (ret > 0) {
			for (k = 0; k < ret; k++){
				printf("%d | ", response[k]);
				response_H = response[3];
				response_L = response[4];
			}
		}
		printf("\nValue_High = %i \n",response_H);
		printf("Value_Low = %i \n",response_L);
		printf("~~~~~~~~~~~~~~~~~\n\n");
		
	}
}