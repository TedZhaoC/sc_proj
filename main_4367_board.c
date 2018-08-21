///////////////////////////////////////////////////
///////////////////////////////////////////////////
//// Farmersedge Copyright
//// Author : Ted Zhao
//// Date   : July 25, 2017 & April, 2018
//// Purpose: ScaleCommand project
///////////////////////////////////////////////////
///////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "string.h"
#include <stdio.h>
#include "stdlib.h"
#include "time.h"

#include <stdbool.h>

#include<pthread.h>

#include <math.h>

#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>


// toradex cmd codes
#define TOR_CMD_PING				0x10
#define TOR_CMD_GET_SW_VER			0x11
#define TOR_CMD_SET_LEDS			0x12
#define TOR_CMD_READ_IO				0x13
#define TOR_CMD_CONFIGURE_INPUTS	0x14
#define TOR_CMD_READ_INPUTS			0x15
#define TOR_CMD_READ_GYROSCOPE		0x16
#define TOR_CMD_SET_COMP_FACTORS	0x17
#define TOR_CMD_RETURN_LC_MODE		0x18
#define TOR_CMD_SET_SERVICE_MODE	0x19
#define TOR_CMD_SET_ERROR_LIMITS	0x1A
#define TOR_CMD_ENABLE_EXCITATION	0x1B
#define TOR_CMD_DISABLE_EXCITATION	0x1C
#define TOR_CMD_MAIN_SWITCH			0x1D
#define TOR_CMD_EEPROM_READ			0x1E
#define TOR_CMD_EEPROM_WRITE		0x1F
#define TOR_CMD_CLAMP_TEST			0x20
#define TOR_CMD_READ_SDADC			0x21
#define TOR_CMD_SDADC_STATE_MACHINE 0x22
#define TOR_CMD_EXC_CHANNEL_SET		0x23
#define TOR_CMD_EEPROM_ERASEALL		0x24
#define TOR_CMD_CALIBRATION_MODE	0x25
#define TOR_CMD_EXC_CH_MUX			0x26
#define TOR_CMD_CONFIGURE_MODE		0x27
#define TOR_CMD_WRITE_SERIAL_NUM	0x28
#define TOR_CMD_READ_SERIAL_NUM		0x29

#define TOR_CMD_RESPONSE_REQUIRED	0x80

// toradex resp codes
#define TOR_RESP_OK							0x00
#define TOR_RESP_UNKNOWN_CMD				0x01
#define TOR_RESP_INVALID_DATA_LEN			0x02
#define TOR_RESP_INVALID_DATA_VALUE			0x03
#define TOR_RESP_UNKNOWN_ERROR				0x04
#define TOR_RESP_CRC_MISMATCH				0x05
#define TOR_RESP_FLASH_OPERATION_FAILED		0x06
#define TOR_RESP_EEPROM_OPERATION_FAILED	0x07
#define TOR_RESP_ACCESS_DENIED				0x08

// LED index
#define LD1 0
#define LD2 1
#define LD3 2
#define LD_ALL 8

// LED Source
#define LED_SOURCE_OFF 0
#define LED_SOURCE_ON 1
#define LED_SOURCE_CAN1 2		// LED on when CAN1 activity detected
#define LED_SOURCE_CAN2 3
#define LED_SOURCE_IGN_SENSE 4	// LED on when IgnSense power is present

// LED pattern
#define LED_PATTERN_OFF 0
#define LED_PATTERN_SOLID_ON 1
#define LED_PATTERN_SUPER_FAST_BLINK 2
#define LED_PATTERN_FAST_BLINK 3
#define LED_PATTERN_NORMAL_BLINK 4
#define LED_PATTERN_SLOW_BLINK 5
#define LED_PATTERN_SUPER_SLOW_BLINK 6

// LED colors
#define LED_COLOR_RED 0
#define LED_COLOR_ORANGE 2
#define LED_COLOR_YELLOW 4
#define LED_COLOR_GREEN 8
#define LED_COLOR_BLUE 16

// FE 
#define LOADCELL_NUM	5

#define NL "\n"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

//////////////////////////////////////////////////////////////////
//Ted global variable

int fd = 0;
struct termios  config;

unsigned sequenceCode;

int lcReadings[8];  // load cell measured value
char data[256];
double gyroReadings[6];	// gyroscope/acceleration readings
//double Vehicle_Angle = 0.0;

int soc;
int read_can_port;

double gyroOffset[6] = { 0.0 };	// gyroscope/acceleration readings
int LoadcellOffset[8] = { 0 };	// up to 8 load cell input readings
//int LoadcellOffset[8] = { 13511};	// up to 8 load cell input readings
int lc_enable[8];
int lc_sensitivity[8];
int lc_range[8];


bool DI_01 = false;
//bool DI_02 = false;
bool DI_02 = true;

bool bt_thread_once = false;
bool bt_switch_first_on = false;
bool bt_switch_first_off = false;

//Ted put them as global on 2018-07-09
float lc_value = 0.0;
float multiplicator = 1.0;
int active_lc_num = 0;
int lc_num = 0;
int sc_mode = 0; // loadcell  mode, parallel-0; single-1
int sc_mode_tmp = 99; // loadcell  mode, parallel-0; single-1
int sc_respCode = 0; //Ted for test right now
int lcStatus[8];	// status of load cell input
bool app_first = true;
bool DI_02_TED = true;

float acc_x_ave_eng[5]={0.0}, acc_y_ave_eng[5]={0.0}, acc_z_ave_eng[5]={0.0};
int acc_x_ave_eng_can, acc_y_ave_eng_can, acc_z_ave_eng_can;
int acc_x_ave=0, acc_y_ave=0, acc_z_ave=0;
int excitation_v=0;

double Azr = 0.0;

bool bt_init_on = false;
bool start_script_run = false;
bool stop_script_run = false;

bool rf_port_close = false;

//Ted adds for multiplicator on 2018-08-09
const char* lc_multiplicator_file = "/etc/fe_escale/multiplicator.conf";
int kkk=0;

//////////////////////////////////////////////////////////////////

// sleep for specified number of miliseconds
void Sleep(int milisec)
{
	usleep(milisec * 1000);
}

int OpenSerialPort(char* portName)
{

	fd = open(portName, O_RDWR | O_NOCTTY |O_NDELAY );
	if(fd == -1)
	{
		printf("failed to open serial port: %s", portName);
		return 1;
	}

	//
	// Get the current configuration of the serial interface
	//
	if(tcgetattr(fd, &config) < 0)
	{
		printf("tcgetattr() failed" NL);
		return 2;
	}

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
					 INLCR | PARMRK | INPCK | ISTRIP | IXON);

	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off, 
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	//
	// Turn off character processing
	//
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;

	//
	// Communication speed (simple version, using the predefined
	// constants)
	//
	//if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) 
	if(cfsetispeed(&config, B2000000) < 0 || cfsetospeed(&config, B2000000) < 0) 
	{
		printf("cfsetispeed() failed" NL);
		return 3;
	}

	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		printf("tcsetattr() failed" NL);
		return 4;
	}

	return 0;
}

//
// function converts array of bytes to string in hex format that can be used to log transmited or received msgs
//
// transmit: set to 1 for trasmited smg, set to 0 for recevied msg
// msgDescr: resutl string will be placed here, there needs to be enough chars allocated for max valid msg (3*256 + 32)
// msgBuf: msg bytes, msgBuf[0] must contain valid msg len
// len: total number iof bytes in msg
//
void MsgToStr(int transmit, char *msgDescr, unsigned char *msgBuf, int len)
{
	char *p;
	int i;
	char *cmdName = "";

	if(transmit)
	{
		switch(msgBuf[2] & 0x7f)
		{
			case 0x10: cmdName = "ping"; break;
			case 0x11: cmdName = "get_fw_ver"; break;
			case 0x12: cmdName = "set_leds"; break;
			case 0x13: cmdName = "read_io"; break;
			case 0x14: cmdName = "ConfigureInputs"; break;
			case 0x15: cmdName = "ReadInputs"; break;
			case 0x16: cmdName = "ReadGyro"; break;
		}
	}
	else
	{
		switch(msgBuf[2])
		{
			case 0x00: cmdName = "ok"; break;
			case 0x01: cmdName = "unknown_cmd"; break;
			case 0x02: cmdName = "invalid_data_len"; break;
			case 0x03: cmdName = "invalid_data_value"; break;
			case 0x04: cmdName = "unknown_error"; break;
		}
	}

	// prepare msg header
	sprintf(msgDescr, "%s: len:%d, seq:%x, cmd:%x (%s), data:", transmit ? "Tx" : "Rx", (unsigned)msgBuf[0], (unsigned)msgBuf[1], (unsigned)msgBuf[2], cmdName);
	p = msgDescr + strlen(msgDescr);

	// print all data bytes
	for(i=3; i < len - 1; ++i)
		p += sprintf(p, " %02x", (unsigned char) msgBuf[i]);

	// add checksum
	sprintf(p, ", chk:%x" NL, msgBuf[len - 1]);
}

//
// compose and send request to card,
//
// Cmd - comamnd code, one of CC_CMD_xxx codes
// data - poitner to data to send, can be NULL is no data should be includes in request,
// dataLen - number of data bytes, can be zero if ther are no data bytes,
//
// function returns 0 if ok
int SendMsg(int Cmd, unsigned char *data, int dataLen)
{
	unsigned char buf[256];
	int len = 4 + dataLen;
	int checksum;
	int i;
	unsigned char msg[256*3 + 32];
	int sum;

	int n;

	buf[0] = len;				// length of request including checksum

	if(++sequenceCode == 0)
		sequenceCode = 1;
	buf[1] = sequenceCode;		// sequence code - card will return this in response, used to verify that response to last requested cmd was received

	buf[2] = Cmd;				// requested command

	// data go to buf[3 ... x]
	if(dataLen > 0)
	{
		memcpy(&buf[3], data, dataLen);
	}

	// calc checksum
	// valid chechsum is valid if you add all msg bytes inclusing checsum the low 8 bits or result is 0
	sum = 0;
	for(i=0; i<len-1; ++i)
		sum += (unsigned char) buf[i];

	sum &= 0xff;
	checksum = -sum;
	buf[len-1] = checksum;

	// log msg
	MsgToStr(1, msg, buf, len);

	//Ted may comment it out
	//printf("%s", msg);

	// now we have msg ready

	// clear all rx and tx buffers - we want to remove any chars from previous messages
	// this needs to be done for faster recovery after communication errors
	//PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	// send it out on serial port
	n = write(fd, buf, len);
    if (n < 0)
    {
		printf("write failed" NL);
		return 1;
    }

	// OK
	return 0;
}

//
// function waits for response from card,
// function check response checksum and len and if all is OK it will set:
//		dataLen to number of received data bytes (this will be 0 for responses that do not return data), pointer can be NULL if this info is not required
//		and store response bytes to *data, data can be NULL if response should not be copied,
// timeoutMs is value in miliseconds for how long to wait for response,
// function returns 0 if ok
// 
//
int WaitForMsg(char *data, int *dataLen, int timeoutMs)
{
	char buf[1024];
	int len;
	int received;
	int checksum;
	int i;
	char msg[256*3 + 32];
	clock_t startTime;
	clock_t currentTime;
	int dataLength;
	unsigned char receivedSeqCode;
	int respCode;
	double durationSec;

	int readCount;

	startTime = clock();
	received = 0;
	while(1)
	{
		// check if there are any bytes ready
		readCount = read(fd, buf + received, sizeof(buf) - received);
		if(readCount > 0)
			received += readCount;

		if(received > 0)
		{
			// check if first byte is valid
			len = (unsigned char) buf[0];
			dataLength = len - 4;
			if(dataLen != NULL)
				*dataLen = dataLength;

			if(len < 4)
			{
				MsgToStr(0, msg, buf, received);
				printf("invalid len byte in receveid msg: %s" NL, msg);
				return 2;
			}

			// check if complete msg was recevied
			if(received >= len)
			{
				// we have complete msg
				// check checksum
				// valid chechsum is valid if you add all msg bytes inclusing checsum the low 8 bits of result is 0
				checksum = 0;
				for(i=0; i<len; ++i)
					checksum = (checksum + (unsigned char) buf[i]) & 0xff;
				if(checksum != 0)
				{
					MsgToStr(0, msg, buf, len);
					printf("invalid checksum in received msg: %s" NL, msg);
					return 3;
				}

				// check if sequence number in response matches seq number in request
				receivedSeqCode = (unsigned char) buf[1];
				//if(sequenceCode != receivedSeqCode)
				//{
				//	MsgToStr(0, msg, buf, len);
				//	printf("sequence code mismatch in received msg: %s" NL, msg);
				//	return 4;
				//}

				// get resp code
				respCode = (unsigned char) buf[2];
				//if(respCode != 0)
				//{
				//	MsgToStr(0, msg, buf, len);
				//	printf("error response code received from card: %d: %s" NL, respCode, msg);
				//	return respCode;
				//}

				// copy response
				if(data != NULL)
					memcpy(data, &buf[0], len);

				// return OK
				MsgToStr(0, msg, buf, len);

				//Ted may comment out the following line for test
				//printf("%s" NL, msg);
				return 0;
			}
		}

		// check for timeout
		currentTime = clock();
		durationSec = (double)(currentTime - startTime) / CLOCKS_PER_SEC;
		if(durationSec * 1000.0 > (double) timeoutMs)
		{
			//MsgToStr(0, msg, buf, received);
			//printf("timeout waiting for response, received so far: %s" NL, msg);
			return 5;
		}

		if(readCount == 0)
		{
			// we did not received anything now, sleep a bit
			Sleep(1);
		}
	}
}

//
// compose and send request to card, function then waits for response from card,
// function check response checksum and len and if all is OK it will set:
//
// Cmd - comamnd code
// cmdData - pointer to data to send, can be NULL is no data should be includes in request,
// cmdDataLen - number of data bytes, can be zero if ther are no data bytes,
// respData - pointer to buffer where to copy response data, can be set to NULL if not response data should be copied
// respDataLen - number of received data bytes (this will be 0 for responses that do not return data), can be NULL
// timeoutMs - value in miliseconds for how long to wait for response,
// maxRetryCount - how many times to retry to send request after error or timeot, if 0 then no retries will be done (we wait inly once), if <0 then function will not wait for reaponse at all,
//
// function returns one of CC_STAT_xxx code, these values include internal errors and also return codes from card,
// if all is OK and card returned OK then function returns CC_STAT_OK
//
int SendAndWait(int cmd, char *cmdData, int cmdDataLen, char *resp, int *respDataLen, int timeoutMs, int maxRetryCount)
{
	int ret;
	int retryCounter;

	// wait for response
	for(retryCounter = 0; retryCounter <= maxRetryCount; ++retryCounter)
	{
		// send request
		ret = SendMsg(cmd, cmdData, cmdDataLen);
		if(ret != 0)
			return ret;

		ret = WaitForMsg(resp, respDataLen, timeoutMs);
		if(ret == 0)
		{
			// we have received something from card, return now
			return ret;
		}

		// some internal error happend, probably timeout
		// wait a bit then try again
		Sleep(100);
	}

	return ret;
}

//Ted modify on July 24, 2017

void read_version()
{
	int ret;
	char resp[256];
	int respDataLen;
	int respCode;
	unsigned short swVer;

	printf("micro text app v1.0A: " __DATE__ " " __TIME__ NL);

// send read sw ver
	printf("--- read SW ver ---" NL);

	ret = SendAndWait(TOR_CMD_GET_SW_VER | TOR_CMD_RESPONSE_REQUIRED, NULL, 0, resp, &respDataLen, 100, 0);
	//ret = SendAndWait(TOR_CMD_READ_INPUTS, data, 0, resp, &respDataLen, 100, 0);
	respCode = resp[2];

	if(ret != 0)
	{
		printf("failed to get response" NL);
	}
	else if(respCode == 0)
	{
		memcpy(&swVer, &resp[3], 2);
		printf("sw ver: %x.%x%x%x" NL NL, (swVer >> 12) & 0x0f, (swVer >> 8) & 0x0f, (swVer >> 4) & 0x0f, (swVer >> 0) & 0x0f);
	}
	else
	{
		printf("error response code: %d" NL, respCode);
	}
}

void write_led()
{
	int ret;
	char resp[256];
	int respDataLen;
	char data[4];

	// turn LD8 on for 0.5 second, fast blinking in green, then turn it off
	printf("--- turn LD8 on for 1 second, fast blinking in green ---" NL);
//	data[0] = LD4;

	data[0] = LD1;
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_SUPER_FAST_BLINK;
	data[3] = LED_COLOR_GREEN;
	ret = SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

	Sleep(500);

	data[0] = LD2;
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_SUPER_FAST_BLINK;
	data[3] = LED_COLOR_GREEN;
	ret = SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

	Sleep(500);


	data[0] = LD3;
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_SUPER_FAST_BLINK;
	data[3] = LED_COLOR_GREEN;
	ret = SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

	Sleep(500);

	/*
	data[0] = LD4;
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_SUPER_FAST_BLINK;
	data[3] = LED_COLOR_GREEN;
	ret = SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

	Sleep(500);
	*/
	// turn LD4 off
	/*
	printf("--- turn LD8 off ---" NL);
	data[0] = LD4;
	data[1] = LED_SOURCE_OFF;
	data[2] = LED_PATTERN_OFF;
	ret = SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);
	*/
}

void led_bluetooth()
{
	int		ret;
	char 	resp[256];
	int 	respDataLen;
	char 	data[4];

	data[0] = LD1; 
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_SOLID_ON;
	data[3] = LED_COLOR_GREEN;
	SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

}

void led_rfcomm_on()
{
	int		ret;
	char 	resp[256];
	int 	respDataLen;
	char 	data[4];

	data[0] = LD2; 
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_SOLID_ON;
	data[3] = LED_COLOR_GREEN;
	SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

}

void led_rfcomm_off()
{
	int		ret;
	char 	resp[256];
	int 	respDataLen;
	char 	data[4];

	data[0] = LD2; 
	data[1] = LED_SOURCE_ON;
	data[2] = LED_PATTERN_OFF;
	data[3] = LED_COLOR_GREEN;
	SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

}

void led_canbus()
{
	int		ret;
	char 	resp[256];
	int 	respDataLen;
	char 	data[4];

	data[0] = LD3; 
	data[1] = LED_SOURCE_CAN1;
	data[2] = LED_PATTERN_FAST_BLINK;
	data[3] = LED_COLOR_GREEN;
	SendAndWait(TOR_CMD_SET_LEDS, data, 4, resp, &respDataLen, 100, 0);

}

void send_ping()
{
	int ret;
	char resp[256];
	int respDataLen;


	// send ping
	printf("--- ping ---" NL);
	ret = SendAndWait(TOR_CMD_PING, NULL, 0, resp, &respDataLen, 100, 0);
	if(ret != 0)
		printf("SendAndWait() returned error code: %d" NL, ret);
	else
		printf("SendAndWait() OK");

}

void read_IO()
{
	int ret;
	char resp[256];
	int respDataLen;
	int respCode;
	char data[4];

	// read IO
	printf("--- read IO ---" NL);
	ret = SendAndWait(TOR_CMD_READ_IO, data, 4, resp, &respDataLen, 100, 0);
	//Ted add 
	respCode = resp[2];
	if(ret != 0)
	{
		printf("failed to get response" NL);
	}
	else if(respCode == 0)
	{
		printf("Vin: %3.1fV, IgnSense: %3.1fV, DO1: %3.1fV, DO2: %3.1fV, V5V: %4.2fV, V5VA: %4.2fV, LCVref: %4.2fV, DI1: %s, DI2: %s" NL,
			((double)resp[3 + 0]) * 0.1,    // Vin
			((double)resp[3 + 1]) * 0.1,    // ign sense
			((double)resp[3 + 2]) * 0.1,    // DO1
			((double)resp[3 + 3]) * 0.1,    // DO2
			((double)resp[3 + 4]) * 0.025,    // V5V
			((double)resp[3 + 5]) * 0.025,	// V5VA
			((double)resp[3 + 6]) * 0.025,	// LCVref
			(resp[3 + 7] & 0x01) == 0 ? "off" : "on",	// DI1
			(resp[3 + 7] & 0x02) == 0 ? "off" : "on"	// DI2
			);

		//Ted add on Oct 17, 2017 to deal with DI 
		if ((resp[3 + 7] & 0x01) == 0)
		{
			DI_01 = false;
		}
		else
		{
			DI_01 = true;
		}

		if ((resp[3 + 7] & 0x02) == 0)
		{
			DI_02 = false;
		}
		else
		{
			DI_02 = true;
		}
	}
	else
	{
		printf("error response code: %d" NL, respCode);
	}

}

void lc_write_multiplicator(const char* filename)
{
	FILE * fp;

	fp = fopen(filename, "w");

	if (fp == NULL)
	{
		printf("oops, file can't be read\n");
		exit(EXIT_FAILURE);
	}

	fprintf(fp, "%f\n", multiplicator); // write to file

	fclose(fp);

}

void write_conf()
{
	FILE * fp;
	int i;

	fp = fopen("/opt/scale-command/offset.conf", "w");

	if (fp == NULL)
	{
		printf("oops, file can't be open for write.\n");
		exit(EXIT_FAILURE);
	}

	// write to file vs write to screen
	for (i = 0; i< 14; i++) // 6 + 8
	{
		if (i < 6)
		{
			fprintf(fp, "%f\n", gyroOffset[i]); // write to file
		}
		else 
		{
			fprintf(fp, "%d\n", LoadcellOffset[i]); // write to file
		}

	}

	fclose(fp);

}

/////////////////////////////////////////////////////////////////////////////////////////
int ReadWithTimeout(char *data, int dataLen, int fdes)
{
	int readCount = 0;

	readCount = read(fdes, data, dataLen);

	//  printf("received data is %d and  %s\n", readCount, data);

	if(readCount > 0)
		return 0; //success
	else
		return 1; // error
}

//Ted modify on 2018-01-03
//Part - 2
void* bluetooth_link()
{
	int fdes = 0;
    char rec_cmd[100];
	char send_data[100];
    char bt_buf[100];
    char bt_buf_show[500];
    char bt_buf_menu[500];
  	char *bt_portName = "/dev/rfcomm0";
	int i;
	int fake_weight;

	int	len = 8;
	int max_rsp = 255;

	//Ted adds on 2018-07-09
    char cali_buf[100];
    char *token;
	int digi_display = 50000; // assume 50000 lbs
	int index = 0;
	int lc_value_caltest = 1000;
	float ref_ratio[10] = {0.0};
	float sum_temp = 0.0;
	char data_led[4];
	
	//Ted field test 8.7
	char mode_config_bluetooth[2];
	char resp[83];  //3+8*10
    int respDataLen;

	char* p;
	char* menu = "commands list:\
			show, \
			zero, \
			REF###(### is number, e.g. REF1000), \
			REFEND (used when finishing all REF###), \
			help";
	char* menu2[] = { "==========================",
		              "show - return final weight",
					  "zero - zero calibration" ,
		              "help - show this menu",
		              "REF### - (e.g. REF1000)",
			          "REFEND (after all REF###)", 
					  "==========================",
					  "\0"};

//	int flags = IREQ_CACHE_FLUSH;

    memset( (void*) bt_buf, 0, sizeof(bt_buf));

	fake_weight = rand() % 100;

    printf(" bluetooth link start ->>>>>>>>>>>- %d ! \n\n", fake_weight);

	while(1)
    {

    	printf("**** We are in the link thread 111 ****! \n");
/*
		//1.
		while(!DI_02)
		{
			//bit_init_on == driver load and never unload
			if (bt_init_on == false)
			{

    			printf(" 111111111111111111111111111111! \n");
				printf("**** Bluetooth function is inactive now!  **** \n\n\n");
				sleep(10);

			}
			else if (bt_init_on == true && stop_script_run == false)
			{
				stop_script_run = true;
				start_script_run = false;
				
				if (fdes)
					close(fdes);

				system("pkill rfcomm"); // there is hanging rfcomm process.

				sleep(2);

				printf("**** 77777777777777777777777777777 ****! \n");
				system("/home/root/lci/scripts/bt/ted_stop_bt.sh &");

				printf("**** Bluetooth function is inactive now!  **** \n\n\n");
				sleep(10);

			}
			else
			{
				//Ted TODO 
				//check unrevoked resources 
				// 1. rfcomm process
				// 2. /dev/rfcomm0
				// 3 (optional) hci0 not power off?  ...
				// 4. fdes close or not? necessary? close twice?
				// 5. status variable go to stable default

				//,,,,,,,,,,,,,,,,,,
				//system("pkill rfcomm");
				//system("rfcomm release xxx");

				//close(fdes);
				if (fdes)
					close(fdes);

				//start_script_run = false;
				//stop_script_run = false;

				printf("**** 999999999 ****! \n");
				printf("**** Ted test %d %d %d****! \n\n", start_script_run, stop_script_run, bt_init_on);
				printf("**** Bluetooth function is inactive now!  **** \n\n\n");
				sleep(16);
			}
		}
*/

		//2. Ted rewrite this part, if later when need use previous better logic, please refer to 
		//   the source code with hardware DI
		while(DI_02_TED)  //no difference just to make it more readable
		{
			///////////// START //////////////////////////////////////////////////////////
			//Ted adds on 2018-07-26 
			//An expedient solution before CANBUS turn on/off command not implemented.
			if (app_first)
			{
				system("pkill rfcomm"); // To kill any possible hanging rfcomm process.
				app_first = false;

				sleep(1);
			}

			///////////// END ////////////////////////////////////////////////////////////

			if (start_script_run == false) // the first time turn on the switch - init script
			{
				printf("Bluetooth switch is turned on!  " NL);

				bt_init_on = true;
				start_script_run = true;
				stop_script_run = false;

//				printf("****222222222222222222222222222222****! \n");
				//Ted changes on 2018-08-20 
				//Improve the deployment  
				//system("/home/root/lci/scripts/bt/ted_start_bt.sh &");
				system("/home/root/fe/scripts/bt/ted_start_bt.sh &");

				//TED set LED on 2018-07-27
				led_bluetooth();

				sleep(10);

				while (access(bt_portName, F_OK) && DI_02_TED)
				{
					printf("--- ted waiting for connection ..... ! \n\n\n");
					sleep(10);
				}

				//Ted insert LED for connected bluetooth channel, ie rfcomm0
				//2018-07-27
				led_rfcomm_on();

				if (DI_02_TED)
				{
					fdes = open(bt_portName, O_RDWR | O_NOCTTY | O_SYNC);

					if (fdes == -1)
						printf("ERROR: Not be able to open the port. \n");
					else
						printf("SUCCESS: open the port for bluetooth channel. \n");
				}

			}

			printf("---------- we come here again! ----------- \n\n\n");
			printf("Ted test- is /dev/rfcomm0 there ?  %d \n\n\n", access(bt_portName, F_OK));

			if (access(bt_portName, F_OK) && DI_02_TED) // file not exist - proving the connection is done and rfcomm process is gone.
			{
				printf("-- We try to restart a new listening port --! \n");
				printf("-- Ted since the /dev/rfcomm0 is gone \n\n\n");

				system("rfcomm --raw listen /dev/rfcomm0 1 &");

				while (access(bt_portName, F_OK) && DI_02_TED) //since external device status can change at any time
				{
					printf("--- ted waiting for connection again ..... ! \n\n\n");
					sleep(10);
				}

				//Ted insert LED for connected bluetooth channel 
				//2018-07-27
				led_rfcomm_on();

				if (DI_02_TED)
				{
					fdes = open(bt_portName, O_RDWR | O_NOCTTY | O_SYNC);

					if (fdes == -1)
						printf("ERROR: Not be able to open the port. \n");
					else
						printf("SUCCESS: open the port for bluetooth channel. \n");
				}
			}

			//Ted assume all commands are of 4 letters
			//Connection is established and waiting for incoming commands
			//3. Ted TODO add to checking condition of DI_02 as well  

			//if (ReadWithTimeout(rec_cmd, sizeof(rec_cmd), fdes) == 0) // waiting for incoming commands 
			if (ReadWithTimeout(rec_cmd, sizeof(rec_cmd), fdes) == 0 && DI_02_TED) // waiting for incoming commands 
			{

				printf("Ted field test 2018.7.24 - bluetooth  i am here ------ :  %d\n\n\n\n\n\n\n\n\n");

				//Ted mark : later use switch block to replace it 
				//when all the commands are of the same len, rules etc.
				//may be for the next release version
				if (strncmp(rec_cmd, "zero", 4) == 0)
				{
					printf("commands received ! \n\n\n");

					//set the offset to current value to cause the final result=0
					
					for(i=0; i<8; ++i)
					{
					   LoadcellOffset[i] = lcReadings[i];
					}

					//sprintf(bt_buf, "returned weight is %d", 0);
					//write(fdes, bt_buf, sizeof(bt_buf));

					//Ted test snippet
					//2018-01-05
					//TODO
					////////////////////////////////////

					fake_weight = 0;
					sprintf(bt_buf, "returned weight is %d", fake_weight);
					write(fdes, bt_buf, sizeof(bt_buf));

					////////////////////////////////////
				}
				else if (strncmp(rec_cmd, "exit", 4) == 0)
				{
					printf("commands received ! \n\n\n");

					//TODO
					//...
				}
				else if (strncmp(rec_cmd, "show", 4) == 0)
				{
					printf("Show command is received ! \n\n\n");

					//Ted test snippet
					//2018-01-05
					////////////////////////////////////
					//1. weight 
					//2. angle
					//3. multiplicator 
					//4. Accl_X
					//5. Accl_Y
					//6. Accl_Z
					//7. uncompensated weight, i.e. no mulitplicator applied, the SC system measurement 
					//8. mix of status code(4), SC mode(1) and loadcell#(3) 4-1-3

					
/*
					snprintf(bt_buf_show, sizeof(bt_buf_show), "\nCalibrated weight : %f\n\t \
														Angle : %f\n\t \
														Multiplicator : %f\n\t \
														Accl_X : %f\n \
														Accl_Y : %f\n \
														Accl_Z : %f\n \
														SC weight : %f\n \
														SC mode : %d\n \
														Active lc num: %d\n \
														ted test angle: %f\n \
														Status code : %d\n", 
														lc_value*multiplicator, 
														Vehicle_Angle, 
														multiplicator, 
														(float) gyroReadings[3], 
														(float) gyroReadings[4], 
														(float) gyroReadings[5], 
														lc_value, 
														sc_mode, 
														active_lc_num, 
														(float) Azr, 
														sc_respCode);
*/
					snprintf(bt_buf_show, sizeof(bt_buf_show), "Calibrated weight : %f \
														Multiplicator : %f \
														SC weight : %f \
														SC mode : %d \
														Status code : %d\n", 
														lc_value*multiplicator, 
														multiplicator, 
														lc_value, 
														sc_mode, 
														sc_respCode);

					write(fdes, bt_buf_show, sizeof(bt_buf_show));
	
					printf(" The lc_Value of SC is : %f\n", lc_value);
					printf("**** lc_value is %d ! \n", (int) lc_value);
					printf("**** multiplicator is %f ! \n", multiplicator);
					printf("**** sc_mode is %d ! \n", sc_mode);
/*
					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "Angle : %f\n", Vehicle_Angle);
					write(fdes, bt_buf, sizeof(bt_buf));

					snprintf(bt_buf, sizeof(bt_buf), "Multiplicator : %f\n", multiplicator);
					write(fdes, bt_buf, sizeof(bt_buf));

					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "Accl_X : %f\n", (float) gyroReadings[3]);
					write(fdes, bt_buf, sizeof(bt_buf));

					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "Accl_Y : %f\n", (float) gyroReadings[4]);
					write(fdes, bt_buf, sizeof(bt_buf));

					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "Accl_Z : %f\n", (float) gyroReadings[5]);
					write(fdes, bt_buf, sizeof(bt_buf));

					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "SC weight : %f\n", lc_value);
					//snprintf(bt_buf, sizeof(bt_buf), "Uncompensated weight : %f\n", fake_weight);
					write(fdes, bt_buf, sizeof(bt_buf));

					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "SC mode : %d\n", sc_mode);
					write(fdes, bt_buf, sizeof(bt_buf));

					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "Active lc num: %d\n", active_lc_num);
					write(fdes, bt_buf, sizeof(bt_buf));
	
					memset(bt_buf, 0, 100);	
					snprintf(bt_buf, sizeof(bt_buf), "Status code : %d\n", sc_respCode);
					write(fdes, bt_buf, sizeof(bt_buf));
*/						
				}
				///////// <<<<<<--- Ted adds on 2018-07-09 ///////////
				// To deal with calibration process in parallel with Digi-Star display reading
				// calculate multiplicator 
				else if (strncmp(rec_cmd, "help", 4) == 0)
				{
					//printf("help command received ! \n\n\n");
					printf("88888888888888888888888\n\n\n");

					i=0;
					printf("in 666666 - menu2[i++] is %d %s \n", i, menu2[i]);

					if (menu2[i++])
						printf("in if-loop - menu2[i++] is %d %s \n", i, menu2[i]);

					while (menu2[i++])
					{
						printf("in while-loop -  menu2[i++] is %d %s \n", i, menu2[i]);

						snprintf(bt_buf_menu, sizeof(bt_buf_menu),menu2[i++]);
						write(fdes, bt_buf_menu, sizeof(bt_buf_menu));
					}
				}
				else if (strncmp(rec_cmd, "REFxx", 3) == 0)
				{
					printf("ted test for index ! %d\n\n\0", index);

					if ((strncmp(rec_cmd, "REFEND", 6) == 0) && (index > 0))
					{

						memset(bt_buf, 0, 100);	

						snprintf(bt_buf, sizeof(bt_buf), "total index is %d\n\n\0", index);
						write(fdes, bt_buf, sizeof(bt_buf));

						for (i=0; i <10; i++)
						{
							sum_temp += ref_ratio[i];
							ref_ratio[i] = 0.0;
						}

						multiplicator = sum_temp / index;

						sum_temp = 0.0;
						index = 0;

						snprintf(bt_buf, sizeof(bt_buf), "after calibration the final multiplicator is %f\n", multiplicator);
						write(fdes, bt_buf, sizeof(bt_buf));


						//Ted save it permanently 2018-08-09
						//Due to time restriction, all of these urgent changes may be optimized later afer harvest
						lc_write_multiplicator(lc_multiplicator_file);

						printf("Calibration process end ! \n\n\n");

					}
					//else if (strcmp(rec_cmd, "REFEND") == 0 && (index == 0))
					else if (strncmp(rec_cmd, "REFEND", 6) == 0 && (index == 0))
				    {
						memset(bt_buf, 0, 100);	
	
						sprintf(bt_buf, "Must input calibration values first ! \n");
						write(fdes, bt_buf, sizeof(bt_buf));

						printf("Must input calibration values first ! \n\n\n");

					}
					else
					{

						printf("Calibration reference string from Digi-Star ! \n\n\n", rec_cmd);

						strcpy(cali_buf, rec_cmd + 3);

						printf("Calibration reference value from Digi-Star ! \n\n\n", cali_buf);

						digi_display = atoi(cali_buf);
						printf("Calibration display value :  %d\n\n\n", digi_display);

						//ref_ratio[index] = (float) digi_display / lc_value_caltest;
						ref_ratio[index] = (float) digi_display / lc_value;
						printf("the ref_ratio at this time is :  %f\n\n\n", ref_ratio[index]);

						snprintf(bt_buf, sizeof(bt_buf), "the ref_ratio at this time is %f\n\0", ref_ratio[index]);
						write(fdes, bt_buf, sizeof(bt_buf));

						index++;
					}
				}
				else if (strncmp(rec_cmd, "range", 5) == 0)
				{

					printf("Range rec_cmd -->   %s\n\n\n", rec_cmd);

					strcpy(cali_buf, rec_cmd + 6);
					printf("Range cali_buf -->   %s\n\n\n", cali_buf);

					token = strtok(cali_buf, ", ");

					i=0;

					while (token)
					{
					    printf("previous range configuration is :  %d %d %d\n", i, lc_enable[i], lc_range[i]);

						lc_range[i] = atoi(token);
						lc_enable[i] = 1;
					    printf("new range is :  %d %d %d\n", i, lc_enable[i], lc_range[i]);

						token = strtok(NULL, ", ");
						i++;
						active_lc_num = i;
					}
				}
				else if (strncmp(rec_cmd, "sen", 3) == 0)
				{

					printf("Sensitivity rec_cmd -->   %s\n\n\n", rec_cmd);

					strcpy(cali_buf, rec_cmd + 4);
					printf("Sensitivity cali_buf -->   %s\n\n\n", cali_buf);

					token = strtok(cali_buf, ", ");

					i=0;

					while (token)
					{
					    printf("previous Sensitivity configuration is :  %d %d\n", i, lc_sensitivity[i]);
						lc_sensitivity[i] = atoi(token);
					    printf("new Sensitivity is :  %d %d\n", i, lc_sensitivity[i]);

						token = strtok(NULL, ", ");
						i++;
					}

				}
				else if (strncmp(rec_cmd, "mode", 4) == 0)
				{
					sc_mode_tmp = atoi(rec_cmd+4);
					printf("SC side - new mode is  :  %d\n", sc_mode_tmp);

					if (sc_mode_tmp == 1)
					{
						mode_config_bluetooth[0] = 0x0; //mode of operation: 0 - pararllel; 1 - single 
						mode_config_bluetooth[1] = 0x1;  
					}
					else if (sc_mode_tmp == 2)
					{
				    	mode_config_bluetooth[0] = 0x1; 
						mode_config_bluetooth[1] = 0x1;  
					}

					SendAndWait(TOR_CMD_CONFIGURE_MODE, mode_config_bluetooth, 2, resp, &respDataLen, 100, 0);

					snprintf(bt_buf, sizeof(bt_buf), "New sc mode is %d\n", sc_mode_tmp);
					write(fdes, bt_buf, sizeof(bt_buf));

				}
				///////// Ted adds on 2018-07-09 --->>>>>>> ///////////
				///////// Ted adds on 2018-08-09 for reset multiplicator ///////////
				else if (strncmp(rec_cmd, "reset", 5) == 0)
				{

					multiplicator = 1.0;
					snprintf(bt_buf, sizeof(bt_buf), "multiplicator reset back to  %f\n", multiplicator);
					write(fdes, bt_buf, sizeof(bt_buf));

					lc_write_multiplicator(lc_multiplicator_file);

				}
				else
				{
					memset(bt_buf, 0, 100);	

					sprintf(bt_buf, "Unknown command ! \n");
					write(fdes, bt_buf, sizeof(bt_buf));

					printf("Receive command is invalid\n\n");
				}
			}
			else //cellphone app disconnection 
			{
				//Ted adds to LED for rfcomm off 
				//2018-07-027
				led_rfcomm_off();

				printf("-- Please turn off the switch now if you want! --! \n\n\n");

				//4. close twice?  
				close(fdes);
				sleep(2);
			}
		} // while(DI_02)
	} //while(1)

}
/////////////////////////////////////////////////////////////////////////////////////////
//Ted modify on July 24, 2017
//Read raw sensor data

void* read_raw_sensor(void* port_name)
{
	char s[256];
	int ret;
	//char data[256];
	char resp[83];	//3+8*10
	int respDataLen;
	int ledIndex;
	int ledSource;
	int ledPattern;
	int ledColor;
	int respCode;
	int i;
//	int lcStatus[8];	// status of load cell input
	char msg[512];
	int value;
	int blocksCount;	// how many data blocks we got from gyroscope
	int bi;				// block index
	int readGyro;

	//Ted test
	double Acc_R = 0.0;
	//double Azr = 0.0;
	double dt = 0.01;
	double previous_value=0.0, delta = 1.0;
	int count = 0;

	char calib_flag;

	//Ted adds on 2018-07-16

	char mode_config[2];

	//Ted 
	ret = OpenSerialPort(port_name);

	if(ret != 0)
    {
		printf("Failed to open the given canbus port " NL);

	}

	//Ted 2018-07-16
	//Initialization of configuration mode to parallel for this version
	//mode_config[0] = 0x2F; //mode pararllel and active loadcell num is 1
	//mode_config[0] = 0x1; //mode of operation: 0 - pararllel; 1 - single 

	//Ted test on 2018-08-13 for unchanged lc value???
	mode_config[0] = 0x0; //mode of operation: 0 - pararllel; 1 - single 
	mode_config[1] = 0x1; //number of loadcell channel

	ret = SendAndWait(TOR_CMD_CONFIGURE_MODE, mode_config, 2, resp, &respDataLen, 100, 0);

/*
	lc_enable[0] = 1;
	lc_enable[1] = 1;
	lc_enable[2] = 0;
	lc_enable[3] = 0;
	lc_enable[4] = 0;
	lc_enable[5] = 0;
	lc_enable[6] = 0;
	lc_enable[7] = 0;
*/
	if(ret != 0)
	{
		printf("-- Ted 2018 response" NL);
	}

	// read load cell inputs and gyroscope 
	while(1)
	//while(kkk < 10)
	{
//		kkk++;
		// 1. read loadcell inputs
		ret = SendAndWait(TOR_CMD_READ_INPUTS, data, 0, resp, &respDataLen, 100, 0);

		//Ted 
		//respCode = resp[2];
		//printf("ret and respCode is  - %d %s\n\n", ret, respCode );

		if(ret != 0)
		{
			printf("failed to get response" NL);
		}
		//else if(respCode == 0)
		else 
		{
			// check if there are some new readings
			// if(respDataLen < 32)
			if(respDataLen < 3) // 3 + n*10, n is the number of active loadcell 
			{
				// no readings ready, need to  wait longer
				// Ted mark
				printf("LC - no readings ready, need to  wait longer" NL);
			}
			else
			{
				//exact the active loadcell channel, ie [0-1-2]; mode [3-4]
				//FE need to send these onto CANBus for monitoring process on the cloud
				// #1 - active_lc_num
				// #2 - mode_read 
				// #3 - lcStatus
				// #4 - lcReadings
				// #5 - acc_x_ave
				// #6 - acc_y_ave
				// #7 - acc_z_ave
				// #8 - excitation_v
				active_lc_num = ((unsigned char)resp[3+0] & 0x03);
				sc_mode = ((unsigned char)resp[3+0] & 0x18) >> 3;

				excitation_v = (unsigned char)resp[3+1];
				excitation_v |= (int)(((unsigned char)resp[3+2]) << 8);

				//printf(">>>>>>>>>>> Ted 2018-08-14 raw resp[3-4-5-6] is : %x %x %x %x\n", resp[3], resp[4], resp[5], resp[6]);
				printf(">>>>>>>>>>> excitation_v is : %d mv \n\n", excitation_v);


				printf("Ted 2018-08-07 - active_lc_num and sc_mode are: %d %d \n", active_lc_num, sc_mode);

				// readings are ready
				strcpy(msg, "LC inputs: ");

				//for(i=0; i<LOADCELL_NUM; ++i)
				for(i=0; i<active_lc_num; ++i)
				{
					//Ted mark on 2018-07-15
	
					//1. loadcell channel status
					lcStatus[i] = (unsigned char)resp[3 + 10 * i + 3];
					//lcStatus[i] = resp[3 + 10 * i + 3];
			
					value = (unsigned char)resp[3 + 10 * i + 4];
					value |= (int)(((unsigned char)resp[3 + 10 * i + 5]) << 8);
					value |= (int)(((unsigned char)resp[3 + 10 * i + 6]) << 16);
				
					//printf(" lc_0%d raw data   :  %x %x %x \n", i+1, resp[3 + 10 * i + 4], resp[3 + 10 * i + 5], resp[3 + 10 * i + 6]);
				//	printf(" 55555555555 ---> raw count before minus 0x800000 is :  %d \n", value);

					value &= 0xffffff;
					value -= 0x800000;

					//printf("Ted - 2018-07-20 >>>>>>> final after minus----> : %d \n", value);

					acc_x_ave = (unsigned char)resp[3 + 10 * i + 7];
					acc_x_ave |= (int)(((unsigned char)resp[3 + 10 * i + 8]) << 8);
	
					acc_y_ave = (unsigned char)resp[3 + 10 * i + 9];
					acc_y_ave |= (int)(((unsigned char)resp[3 + 10 * i + 10]) << 8);
	
					acc_z_ave = (unsigned char)resp[3 + 10 * i + 11];
					acc_z_ave |= (int)(((unsigned char)resp[3 + 10 * i + 12]) << 8);

					acc_x_ave &= 0xffff;
					acc_x_ave -= 0x8000;

					acc_y_ave &= 0xffff;
					acc_y_ave -= 0x8000;

					acc_z_ave &= 0xffff;
					acc_z_ave -= 0x8000;

					//Ted need to account for load cell offset
					//TODO
					// because it has -7
					//...

					//Ted modify on 2018-07-16 
					// 2. loadcell raw weight
					lcReadings[i] = value;

					// 3. average acceleration during this loadcell is being read
					acc_x_ave_eng[i] = (float) ((double)acc_x_ave/(double)0x8000)*2.0;
					acc_y_ave_eng[i] = (float) ((double)acc_y_ave/(double)0x8000)*2.0; 
					acc_z_ave_eng[i] = (float) ((double)acc_z_ave/(double)0x8000)*2.0;
					
					//printf(" channel #%d value lcReadings ----> : %d \n", i, lcReadings[i]);
//					printf(" average accelerations are ----> : %d %d %d\n", acc_x_ave, acc_y_ave, acc_z_ave);
//					printf(" average accelerations (eng) are ----> : %f %f %f\n", acc_x_ave_eng[i], acc_y_ave_eng[i], acc_z_ave_eng[i]);

					sprintf(msg + strlen(msg), "ch%d: %s %d uVV, ", 
						i+1, 
						lcStatus[i] == 0 ? "ok" : lcStatus[i] == 1 ? "dis" : "err",
						lcStatus[i] == 0 ? lcReadings[i] : 0
						);
				}

				//Ted print mark
				printf("%s" NL, msg);

				//Ted would add to features, such as 
				//1. engineering unit conversion, 
				//2. average multiple enabled channels
				//TODO
				//.......
			}

		}

		// 2. read gyroscope
		//Later Ted may need to modify this line to adapt to some situation
		//TODO: 
		readGyro = 1;

		if(readGyro)
		{
			ret = SendAndWait(TOR_CMD_READ_GYROSCOPE, data, 0, resp, &respDataLen, 100, 0);

			//Ted not change first for this case, let's see what's the result first
			//2018-07-16
			//TODO .......
			respCode = resp[2];

			if(ret != 0)
			{
				printf("failed to get response" NL);
			}
			else if(respCode == 0)
			{
				// check how much data we got
				blocksCount = respDataLen / 12;

				//Ted test on 2018-08-13
				printf("Ted test Gyro len is : %d\n",  respDataLen);

				if(blocksCount < 1)
				{
					// no readings ready, need to  wait longer
					printf("Gyro - no readings ready, need to  wait longer" NL);
				}
				else
				{

					// readings are ready
					for(bi=0; bi<blocksCount; ++bi)
					{
						strcpy(msg, "Gyro: ");

						for (i = 0; i < 6; ++i)
						{

							value = resp[3 + bi * 12 + i * 2] & 0xff;
							value |= ((resp[3 + bi * 12 + i * 2 + 1] << 8) & 0xff00);
							//value += ((resp[3 + bi * 12 + i * 2 + 1] << 8) & 0xff00);

							if (resp[3 + bi*12 + i*2 + 1] & 0x80) 
							{
						        value |= 0xFFFF0000;
						    }

							//printf("Ted test gyro-offset %4.2f g\n", gyroOffset[i]);

							if (i == 0)
							{
								// angular rates
								gyroReadings[i] = ((double)value) * 500 / ((double)0x8000u);
								//sprintf(msg+strlen(msg), "xa: %5.1f dps", gyroReadings[i]);

								//Ted mark on 2018-07-26
								//LNC disable gyro angular rate reading for this new SC
								//sprintf(msg+strlen(msg), "xa: %5.1f dps", gyroReadings[i]-gyroOffset[i]);
							}
							else if (i == 1)
							{
								// angular rates
								gyroReadings[i] = ((double)value) * 500 / ((double)0x8000u);
								//sprintf(msg+strlen(msg), ", ya: %5.1f dps", gyroReadings[i]);

								//Ted mark on 2018-07-26
								//LNC disable gyro angular rate reading for this new SC
								//sprintf(msg+strlen(msg), ", ya: %5.1f dps", gyroReadings[i]-gyroOffset[i]);
							}
							else if (i == 2)
							{
								// angular rates
								gyroReadings[i] = ((double)value) * 500 / ((double)0x8000u);
								//sprintf(msg+strlen(msg), ", za: %5.1f dps", gyroReadings[i]);

								//Ted mark on 2018-07-26
								//LNC disable gyro angular rate reading for this new SC
								//sprintf(msg+strlen(msg), ", za: %5.1f dps", gyroReadings[i]-gyroOffset[i]);
							}
							else if (i == 3)
							{
								// accel
								gyroReadings[i] = ((double)value) * 2.0 / ((double)0x8000u);
								sprintf(msg+strlen(msg), ", x: %4.2f g", gyroReadings[i]);
							}
							else if (i == 4)
							{
								// accel
								gyroReadings[i] = ((double)value) * 2.0 / ((double)0x8000u);
								sprintf(msg+strlen(msg), ", y: %4.2f g", gyroReadings[i]);
							}
							else if (i == 5)
							{
								// accel
								gyroReadings[i] = ((double)value) * 2.0 / ((double)0x8000u);
								sprintf(msg+strlen(msg), ", z: %4.2f g", gyroReadings[i]);
							}
						}

						//Ted print mark
						//printf("%s" NL, msg);

						//--------------------------------------------------------------------
					    //step 2 - processing raw data

						//offset compensation
						gyroReadings[0] = gyroReadings[0] - gyroOffset[0];
						gyroReadings[1] = gyroReadings[1] - gyroOffset[1];
						gyroReadings[2] = gyroReadings[2] - gyroOffset[2];
						gyroReadings[3] = gyroReadings[3] - gyroOffset[3];
						gyroReadings[4] = gyroReadings[4] - gyroOffset[4];
						gyroReadings[5] = gyroReadings[5] - gyroOffset[5];

						Acc_R = sqrt(gyroReadings[3]*gyroReadings[3] + gyroReadings[4]*gyroReadings[4] + gyroReadings[5]*gyroReadings[5]);
						//Azr = acos((gyroReadings[5])/Acc_R);
						Azr = asin(sqrt(gyroReadings[3]*gyroReadings[3] + gyroReadings[4]*gyroReadings[4])/Acc_R)*180/3.1415926;

						//printf(" Ted AAAAAAAAA - is : %4.2f %4.2f %4.2f \n", gyroReadings[3], gyroReadings[4], gyroReadings[5]);
						//printf(" Ted bbbbbbbbb - Azr is : %f %f %f \n", Acc_R, gyroReadings[5]/Acc_R, (float) Azr);
						//printf(" Azr (angle in degree) is : %f in degree. \n", (float) Azr*180/3.1415926);
						//printf(" Azr (angle in degree) is : %f in degree. \n", (float) Azr);

						// 2018-07-20 Ted disable this part because LNC firmware disable the gyro angular ones  

						// Ted implement complementary filter from here
						// To combine both data from gyro and accelerometer
						// apply the filter 
					/*
						while (delta > .001 && count < 500)
						{
							count++;
							previous_value = Vehicle_Angle;
							Vehicle_Angle = 0.98*(Vehicle_Angle - sqrt(gyroReadings[0] * gyroReadings[0] + gyroReadings[1] * gyroReadings[1])*dt) + 0.02*Azr;
							//printf(" --222-- the current count is : %d \r\n", count++);
							//printf(" --333-- Vehicle angle is : %5.3f \r\n\n", Vehicle_Angle);
							delta = fabs(Vehicle_Angle - previous_value);

						}

						//printf(" last round convergence count is :  %d \r\n", count);
						//printf(" last round Vehicle angle is : %5.3f \r\n\n", Vehicle_Angle);

						//reset 
						delta = 1; 
						count = 0;

				   */
					}

				}

			}
			else
			{
				printf("error response code: %d" NL, respCode);
			}
		}


		//printf("------ We're in the thread reading ------ " NL);

		Sleep(1000);
	}

	printf("Normally we shouldn't reach here"  NL);

	return NULL;
}

//Ted open canbus port - SocketCan
// run in the main thread
int open_port(const char *port)
{
	struct ifreq ifr;
	struct sockaddr_can addr;

	/* open socket */
	soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(soc < 0)
	{
		return (-1);
	}

	addr.can_family = AF_CAN;
	strcpy(ifr.ifr_name, port);

	if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
	{
		return (-1);
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	fcntl(soc, F_SETFL, O_NONBLOCK);

	if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		return (-1);
	}

	return 0;
}

int send_port(struct can_frame *frame)
{
	int retval;
	retval = write(soc, frame, sizeof(struct can_frame));

	if (retval != sizeof(struct can_frame))
	{
		return (-1);
	}
	else
	{
		return (0);
	}
}

/* this is just an example, run in a thread. It may be useful for future receviving data from canbus */
void read_port()
{
	struct can_frame frame_rd;
	int recvbytes = 0;

	read_can_port = 1;

	while(read_can_port)
	{
		struct timeval timeout = {1, 0};
		fd_set readSet;

		FD_ZERO(&readSet);
		FD_SET(soc, &readSet);

		if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
		{
			if (!read_can_port)
			{
				break;
			}

			if (FD_ISSET(soc, &readSet))
			{
				recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));

				if(recvbytes)
				{
					printf("dlc = %d, data = %s\n", frame_rd.can_dlc,frame_rd.data);
				}
			}
		}

	}

}

int close_port()
{
    close(soc);
    return 0;
}

void lc_read_conf(const char* filename)
{
	FILE * fp;

    //int i=0, j=0;

	fp = fopen(filename, "r");

	if (fp == NULL)
	{
		printf("oops, file can't be read\n");
		exit(EXIT_FAILURE);
	}

	fscanf(fp,"%d %d %d %d %d %d %d %d", &lc_enable[0], &lc_enable[1], &lc_enable[2], &lc_enable[3], &lc_enable[4], &lc_enable[5], &lc_enable[6], &lc_enable[7]); 
	
	fscanf(fp,"%d %d %d %d %d %d %d %d", &lc_sensitivity[0], &lc_sensitivity[1], &lc_sensitivity[2], &lc_sensitivity[3], &lc_sensitivity[4], &lc_sensitivity[5], &lc_sensitivity[6], &lc_sensitivity[7]); 

	fscanf(fp,"%d %d %d %d %d %d %d %d", &lc_range[0], &lc_range[1], &lc_range[2], &lc_range[3], &lc_range[4], &lc_range[5], &lc_range[6], &lc_range[7]);

	fclose(fp);

}

void lc_read_multiplicator(const char* filename)
{
	FILE * fp;

	fp = fopen(filename, "r");

	if (fp == NULL)
	{
		printf("oops, file can't be read\n");
		exit(EXIT_FAILURE);
	}

	fscanf(fp,"%f ", &multiplicator); 
	
	fclose(fp);

}



////////////////////////////////////////////////////////////////////

void main()
{

	// step 1 - reading raw sensor data
    pthread_t pth;
    pthread_t pth2 = NULL;
	struct can_frame frame;
	int err;
	int ret;
	int angle_value_canbus = 0, lc_value_canbus = 0;
	int multi_canbus=0, acc_canbus=0;

	//Ted change it on 2018-07-09
	//make it become global variable 
	//float lc_value = 0.0;
	int lc_value_sc = 0;

	int lc_hitch_value = 0;

	double lc_hitch_temp = 0;
	double lc_compression_temp = 0;
	
	int i, j;

//	int motion = -1;

	//Ted add to a new frame to deal with motion status?
	//struct can_frame frame_motion;

	//const char* port_name = "/dev/ttymxc2";
	void* port_name = "/dev/ttymxc2";


	//Ted adds and changes on April 2018
	//Ted adds and changes on Aug 9, 2018 for saving multipicator 
	//void* lc_conf_file = "/etc/fe_sc/loadcell.conf";
	const char* lc_conf_file = "/etc/fe_escale/loadcell.conf";
	//const char* lc_conf_file = "/etc/fe_escale/multiplicator.conf";

	//////////////////// - Reading parameters of loadcell - ///////////////////////
	lc_read_conf(lc_conf_file);
	lc_read_multiplicator(lc_multiplicator_file);
	printf("The previous calibrated multiplicator is %f !\n\n", multiplicator);

	//Ted test
	for (j=0; j < 8; j++)
	{   
		
		printf("%d \n", lc_enable[j]);
		printf("%d \n", lc_sensitivity[j]);
		printf("%d \n\n", lc_range[j]);

	} 

	//2018-07-11
	for (j=0; j < 8; j++)
	{
		if (lc_enable[j])
			lc_num++;
	}
	printf("Ted 2018-07-11 test for lc_num: %d \n", lc_num);
	//return; //For test

	// configure load cell input 
	/*
	data[0] = 1;	// configure ch1 as load cell input
	data[1] = 0;	//configure as disabled
	data[2] = 0;	//configure as disabled	

	data[3] = 0;	// configure as disabled 
	data[4] = 0;	// configure as disabled
	data[5] = 0;	// configure as disabled
	data[6] = 0;	// configure as disabled
	data[7] = 0;	// configure as disabled
*/
	//Ted test
	data[0] = lc_enable[0];	// configure ch1 as load cell input
	data[1] = lc_enable[1];	//configure as disabled
	data[2] = lc_enable[2];	//configure as disabled	
	data[3] = lc_enable[3];	// configure as disabled 
	data[4] = lc_enable[4];	// configure as disabled
	data[5] = lc_enable[5];	// configure as disabled
	data[6] = lc_enable[6];	// configure as disabled
	data[7] = lc_enable[7];	// configure as disabled

	/////////////////////End of reading loadcell parameters///////////////////

	//Thread #1 - sensor reading
	err = pthread_create(&pth, NULL, &read_raw_sensor, port_name);

	if (err != 0)
		printf("\ncan't create thread :[%s] for sensor reading", strerror(err));
	else
		printf("\n Sensor reading thread is created successfully\n");

	//Give child thread enough time to get data ready
	Sleep(3000);

	//Thread #2 - Bluetooth channel

	err = pthread_create(&pth2, NULL, &bluetooth_link, NULL);

	if (err != 0)
		printf("\ncan't create thread :[%s] for bluetooth link", strerror(err));
	else
		printf("\n Bluetooth link thread is created successfully\n");

	//Ted test minor functions
	/*
	while (1)
	{
		read_version();

		send_ping();

		write_led();

		read_IO();
	}
	*/

	//step 3 - sending processed data to Canplug
	//Ted main thread

	ret = open_port("can0");

	if(ret != 0)
    {
		printf("Failed to open the given canbus port " NL);

	}

	//Ted proprietary PGN/SPN etc

	//	frame.can_id  = 0x123;
	frame.can_id  = 0x14ff0108 | CAN_EFF_FLAG; // PGN is ff01=65281 and source address = 0x08, Priority = 5 
	frame.can_dlc = 8;

	//Ted test 
	led_canbus();

	//Ted test 2018-08-13 
	read_version();

	while (1)
	{

		//Ted prepare package to send
		//TODO : 
		// plan 1 - final averaged loadcell value + vehicle angle 
		// plan 2 - final averaged and angle-corrected loadcell value  
		// plan 3 - all useful raw data, 8 loadcell value, 6 gyro values, offsets, and vehicle angle etc. 

		Sleep(5000);

		//Option 1 : only send the final calculated weight
		// Note: 
		// 1. If one is used to read hitch load cell, the value cannot be used in this way.
		// 2. the server side (cloud) should divide 100 to get the right value 

		///ed read io, which can be optimized to decrease the possibility of io delay or block .
		//setinterval? 

		//Ted disable it on 2018-07-27
		//read_IO();

		////////////////////// Ted modifies on April 2018 - Start /////////////////////
		//Ted mark, write magical number, which should be changed later 
		//lc_hitch_value = lcReadings[2];//assume the third port is connect with hitch sensor 
		//printf("-------------------------------------------------------------------" NL);
		//printf("Logging the offset of each load cell  : %d %d %d" NL, LoadcellOffset[0], LoadcellOffset[1], LoadcellOffset[2]);
		//lc_hitch_value = lcReadings[2] - LoadcellOffset[2];//assume the third port is connect with hitch sensor 
		//lc_value = lcReadings[0] + lcReadings[1];
		//lc_value = lcReadings[0] + lcReadings[1] - LoadcellOffset[0] - LoadcellOffset[1];

		//printf("Ted check log - initial lc_value is : %f\n" NL, lc_value);

		//Ted reset to zero at the beginning
		lc_value = 0.0 ;

		for (i=0; i < LOADCELL_NUM; i++)
		{
			if (lc_enable[i] > 0)
			{
				lc_value += (float)(lcReadings[i]-LoadcellOffset[i])/lc_sensitivity[i]*lc_range[i]/49; 
			//	printf("The configuraton data for test are  : %d %d" NL, lc_sensitivity[i], lc_range[i]);
			}
		}  

		printf(" The total lc_Value (pounds) of SC is : %f\n", lc_value);
		//printf(" The total lc_Value (pounds) with multiplicator is : %f\n\n\n", lc_value*multiplicator);
		//printf(" The total lc_Value (pounds) with multiplicator is : %f\n\n\n", lc_value*multiplicator);

		time_t clk = time(NULL);
		//printf("Timestamp is :%s", ctime(&clk));

		//Ted test cos(x)
		//printf(" Ted test cos(-1.34) is  -------------->  : %f \r\n\n", cos(-1.34));
		//printf("-------------------------------------------------------------------" NL);

		//////////////////////////////////////////////////////
		//Ted adds to more data to send on 2018-07-11
		//1. weight 
		//2. angle
		//3. multiplicator 
		//4. Accl_X
		//5. Accl_Y
		//6. Accl_Z
		//7. uncompensated weight, i.e. no mulitplicator applied, the SC system measurement 
		//8. mix of status code(4), SC mode(1) and loadcell#(3) 4-1-3

		// lc_value_canbus = abs((int) (lc_value * 100)); //Ted mark here
		lc_value_canbus = abs((int) (lc_value * multiplicator * 100)); //Ted mark here

		//Ted test on July 17th, 2018
		//lc_value_canbus = 1111; 

		frame.data[0] = 0x01; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = (lc_value_canbus) & 0xff;
		frame.data[5] = (lc_value_canbus >> 8 ) & 0xff;
		frame.data[6] = (lc_value_canbus >> 16 ) & 0xff;
		frame.data[7] = (lc_value_canbus >> 24 ) & 0xff;

		send_port(&frame); //frame-1, weight

		//Sleep(1000);
		//---------------------------------

		//angle_value_canbus = abs((int) (Vehicle_Angle * 100)); //Ted mark here
		angle_value_canbus = abs((int) (Azr * 100)); //Ted mark here

		//Ted test on July 17th, 2018
		//angle_value_canbus = 30;

		frame.data[0] = 0x02; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = (angle_value_canbus) & 0xff;
		frame.data[5] = (angle_value_canbus >> 8 ) & 0xff;
		frame.data[6] = (angle_value_canbus >> 16 ) & 0xff;
		frame.data[7] = (angle_value_canbus >> 24 ) & 0xff;

		send_port(&frame); //frame-2, angle
		//Sleep(1000);
		//---------------------------------

		multi_canbus = (int) (multiplicator * 100); //Ted mark here

		//Ted test on July 17th, 2018
		//multi_canbus = 160 ; //Ted mark here 1.6

		frame.data[0] = 0x03; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		multi_canbus = (int) (gyroReadings[3] * 100); //Ted mark here

		frame.data[4] = (multi_canbus) & 0xff;
		frame.data[5] = (multi_canbus >> 8 ) & 0xff;
		frame.data[6] = (multi_canbus >> 16 ) & 0xff;
		frame.data[7] = (multi_canbus >> 24 ) & 0xff;

		send_port(&frame);//frame-3, multiplicator
		//Sleep(1000);
		//---------------------------------

		acc_canbus = (int) (gyroReadings[3] * 100); //Ted mark here
		//Ted test on July 17th, 2018
		//acc_canbus = 5555; //Ted mark here

		frame.data[0] = 0x04; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = (acc_canbus) & 0xff;
		frame.data[5] = (acc_canbus >> 8 ) & 0xff;
		frame.data[6] = (acc_canbus >> 16 ) & 0xff;
		frame.data[7] = (acc_canbus >> 24 ) & 0xff;

		send_port(&frame);//frame-4, accl_x
		//Sleep(1000);
		//---------------------------------

		acc_canbus = (int) (gyroReadings[4] * 100); //Ted mark here
		//Ted test on July 17th, 2018
		//acc_canbus = 6666; //Ted mark here

		frame.data[0] = 0x05; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = (acc_canbus) & 0xff;
		frame.data[5] = (acc_canbus >> 8 ) & 0xff;
		frame.data[6] = (acc_canbus >> 16 ) & 0xff;
		frame.data[7] = (acc_canbus >> 24 ) & 0xff;

		send_port(&frame);//frame-5, accl_y
		//Sleep(1000);
		//---------------------------------

		//Ted mark temporary disable it
		acc_canbus = (int) (gyroReadings[5] * 100); //Ted mark here
		//Ted test on July 17th, 2018
		//acc_canbus = 7777; //Ted mark here

		frame.data[0] = 0x06; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = (acc_canbus) & 0xff;
		frame.data[5] = (acc_canbus >> 8 ) & 0xff;
		frame.data[6] = (acc_canbus >> 16 ) & 0xff;
		frame.data[7] = (acc_canbus >> 24 ) & 0xff;

		send_port(&frame);//frame-6, accl_z
		//Sleep(1000);
		//---------------------------------

		//Ted mark 2018-07-11 for now just use it for test
		//uncompensated weight
		lc_value_canbus = abs((int) (lc_value * 100)); //Ted mark here
		//Ted test on July 17th, 2018
		//lc_value_sc = 3333; //Ted mark here

		frame.data[0] = 0x07; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = (lc_value_canbus) & 0xff;
		frame.data[5] = (lc_value_canbus >> 8 ) & 0xff;
		frame.data[6] = (lc_value_canbus >> 16 ) & 0xff;
		frame.data[7] = (lc_value_canbus >> 24 ) & 0xff;

		send_port(&frame);//frame-7, uncompensated weight
		//Sleep(1000);
		//-----------------------------------------------

		//Ted mark need double check this mix value
		//printf("ted test data[4] is %d %d \n\n\n",  sc_mode, active_lc_num);
	
		frame.data[0] = 0x08; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		//Ted find a bug on 2018-08-21
		//frame.data[4] = (sc_mode & 0x3) | 0xff; //wrong 
		frame.data[4] = (sc_mode & 0x3) & 0xff;
		frame.data[5] = 0xff;
		frame.data[6] = (active_lc_num & 0x7) & 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-8, sc_mode+active number
		//Sleep(1000);
		//------------------------------------------------
//		excitation_v_can = excitation_v * 10; //Ted mark - 4.9 -> 49

		frame.data[0] = 0x09; 
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = excitation_v & 0xff;
		frame.data[5] = (excitation_v >> 8 ) & 0xff;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;

		send_port(&frame);//frame-9, excitation voltage 
		//---------------------------------------------i

		frame.data[0] = 0x0a;
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = lcStatus[0] & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-10, channel 1 status 
		//Sleep(1000);
		//-------------------------------------------

		frame.data[0] = 0x0b;
		frame.data[1] = 0x00;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;

		frame.data[4] = lcStatus[1] & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-11, channel 2 status 
		//Sleep(1000);
		//---------------------------------------------
	
		frame.data[0] = 0x0c;
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		frame.data[4] = lcStatus[2] & 0xff; 
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;

		send_port(&frame);//frame-12, channel 3 status 
		//Sleep(1000);
		//-------------------------------------------

		frame.data[0] = 0x0d;
		frame.data[1] = 0x00;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		frame.data[4] = lcStatus[3] & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-13, channel 4 status 
		//Sleep(1000);
		//-------------------------------------	frame.data[0] = lcStatus[3] & 0xff; 

		frame.data[0] = 0x0e;
		frame.data[1] = 0x00;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		frame.data[4] = lcStatus[4] & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-14, channel 5 status 
		//Sleep(1000);
		//---------------------------------------------

		acc_x_ave_eng_can = (int) (acc_x_ave_eng[0] * 100); 

		frame.data[0] = 0x0f;
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		frame.data[4] = acc_x_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-15, loadcell #1 Ave Acc_x 
		//Sleep(1000);
		//-------------------------------------------------

		acc_y_ave_eng_can = (int) (acc_y_ave_eng[0] * 100); 

		frame.data[0] = 0x10;
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		frame.data[4] = acc_y_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-16, loadcell #1 Ave Acc_y 
		//Sleep(1000);
		//-------------------------------------------------

		acc_z_ave_eng_can = (int) (acc_z_ave_eng[0] * 100); 

		frame.data[0] = 0x11;
		frame.data[1] = 0x00; 
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
	
		frame.data[4] = acc_z_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-17, loadcell #1 Ave Acc_z 
		Sleep(1000);
		//-------------------------------------------------

/*
		//Ted disable the following just for this field trip on 2018-07-23
		acc_x_ave_eng_can = (int) (acc_x_ave_eng[1] * 100); 

		frame.data[0] = 0x12;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		//Ted mask them just for test 
		//frame.data[4] = acc_x_ave_eng_can & 0xff; 
		frame.data[4] = 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-18, loadcell #2 Ave Acc_x 
		//-------------------------------------------------

		acc_y_ave_eng_can = (int) (acc_y_ave_eng[1] * 100); 

		frame.data[0] = 0x13;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		//frame.data[4] = acc_y_ave_eng_can & 0xff; 
		//Ted mask them just for test 
		frame.data[4] = 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-19, loadcell #2 Ave Acc_y 
		//-------------------------------------------------

		acc_z_ave_eng_can = (int) (acc_z_ave_eng[1] * 100); 

		frame.data[0] = 0x14;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
//		frame.data[4] = acc_z_ave_eng_can & 0xff; 
		//Ted mask them just for test 
		frame.data[4] = 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-20, loadcell #2 Ave Acc_z 
		//-------------------------------------------------

		acc_x_ave_eng_can = (int) (acc_x_ave_eng[2] * 100); 

		frame.data[0] = 0x15;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		//Ted mask them just for test 
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_x_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-21, loadcell #3 Ave Acc_x 
		//-------------------------------------------------

		acc_y_ave_eng_can = (int) (acc_y_ave_eng[2] * 100); 

		frame.data[0] = 0x16;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;

		//Ted mask them just for test 
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_y_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-22, loadcell #3 Ave Acc_y 
		//-------------------------------------------------

		acc_z_ave_eng_can = (int) (acc_z_ave_eng[2] * 100); 

		frame.data[0] = 0x17;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;

		//Ted mask them just for test 
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_z_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-23, loadcell #3 Ave Acc_z 
		//-------------------------------------------------

		acc_x_ave_eng_can = (int) (acc_x_ave_eng[3] * 100); 

		frame.data[0] = 0x18;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		//Ted mask them just for test 
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_x_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-24, loadcell #4 Ave Acc_x 
		//-------------------------------------------------

		acc_y_ave_eng_can = (int) (acc_y_ave_eng[3] * 100); 

		frame.data[0] = 0x19;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
		
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_y_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-25, loadcell #4 Ave Acc_y 
		//-------------------------------------------------

		acc_z_ave_eng_can = (int) (acc_z_ave_eng[3] * 100); 

		frame.data[0] = 0x1a;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_z_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-26, loadcell #4 Ave Acc_z 
		//-------------------------------------------------

		acc_x_ave_eng_can = (int) (acc_x_ave_eng[4] * 100); 

		frame.data[0] = 0x1b;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_x_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-27, loadcell #5 Ave Acc_x 
		//-------------------------------------------------

		acc_y_ave_eng_can = (int) (acc_y_ave_eng[4] * 100); 

		frame.data[0] = 0x1c;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_y_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-28, loadcell #5 Ave Acc_y 
		//-------------------------------------------------

		acc_z_ave_eng_can = (int) (acc_z_ave_eng[4] * 100); 

		frame.data[0] = 0x1d;
		frame.data[1] = 0xff; 
		frame.data[2] = 0xff;
		frame.data[3] = 0xff;
	
		frame.data[4] = 0xff; 
		//frame.data[4] = acc_z_ave_eng_can & 0xff; 
		frame.data[5] = 0xff;
		frame.data[6] = 0xff;
		frame.data[7] = 0xff;

		send_port(&frame);//frame-29, loadcell #5 Ave Acc_ 
		//-------------------------------------------------
*/

		//Ted adds LED on 2018-07-27
		//led_canbus();

	//Ted 8.7 field test - error is here
		//reset to zero //double safety //in fact not necessary
		//lc_value = 0.0 ;
		//Vehicle_Angle = 0.0 ;
		//Azr = 0.0 ;

//		printf("/////////////////////////////////////////" NL);
//		printf(" ------ You're in the main thread ------ " NL);
//		printf("/////////////////////////////////////////" NL);

	}

	printf("Normally you should not reach here!" NL);

}
