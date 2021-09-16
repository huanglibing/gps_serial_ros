/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-16 15:14:38
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-16 15:26:19
 */
#include <iostream>
#include <termios.h>
#include <sys/ioctl.h>
#include "../include/UartCom.h"

int UART_Init_Baud(int fd,int baud)
{
	struct termios options;
    if(tcgetattr(fd, &options) != 0)
    {
        std::cout << "Fail to get the ttyAMA3 current optionsions!" << std::endl;
        return -1;
    }
	if(baud==230400)
	{
		cfsetispeed(&options, B230400);
		cfsetospeed(&options, B230400);
		options.c_cc[VINTR] = 0;/**//*Ctrl-c*/
		options.c_cc[VQUIT] =0;/**//*Ctrl-*/
		options.c_cc[VERASE] = 0;/**//*del*/
		options.c_cc[VKILL]=0;/**//*@*/
		options.c_cc[VEOF]=0;/**//*Ctrl-d*/
		options.c_cc[VTIME]=1;/**//**/
		options.c_cc[VMIN]=0;/**//**/
		options.c_cc[VSWTC]=0;/**//*''*/
		options.c_cc[VSTART]=0;/**//*Ctrl-q*/
		options.c_cc[VSTOP]=0;/**//*Ctrl-s*/
		options.c_cc[VSUSP]=0;/**//*Ctrl-z*/
		options.c_cc[VEOL]=0;/**//*''*/
		options.c_cc[VREPRINT]=0;/**//*Ctrl-r*/
		options.c_cc[VDISCARD]=0;/**//*Ctrl-u*/
		options.c_cc[VWERASE]=0;/**//*Ctrl-w*/
		options.c_cc[VLNEXT]=0;/**//*Ctrl-v*/
		options.c_cc[VEOL2]=0;/**//*''*/
	}
	else if(baud==115200)
	{
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		options.c_cc[VINTR] = 0;/**//*Ctrl-c*/
		options.c_cc[VQUIT] =0;/**//*Ctrl-*/
		options.c_cc[VERASE] = 0;/**//*del*/
		options.c_cc[VKILL]=0;/**//*@*/
		options.c_cc[VEOF]=1;/**//*Ctrl-d*/
		options.c_cc[VTIME]=0;/**//**/
		options.c_cc[VMIN]=0;/**//**/
		options.c_cc[VSWTC]=0;/**//*''*/
		options.c_cc[VSTART]=0;/**//*Ctrl-q*/
		options.c_cc[VSTOP]=0;/**//*Ctrl-s*/
		options.c_cc[VSUSP]=0;/**//*Ctrl-z*/
		options.c_cc[VEOL]=1;/**//*''*/
		options.c_cc[VREPRINT]=0;/**//*Ctrl-r*/
		options.c_cc[VDISCARD]=0;/**//*Ctrl-u*/
		options.c_cc[VWERASE]=0;/**//*Ctrl-w*/
		options.c_cc[VLNEXT]=0;/**//*Ctrl-v*/
		options.c_cc[VEOL2]=0;/**//*''*/
	}
	else 
		return -1;

    options.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
    ioctl( fd, TCSETS, &options);//TCSETS
                        
    options.c_cflag |= CREAD;  // enable the receiver
    options.c_cflag |= CLOCAL; // local line - do not change
    options.c_oflag &= ~OPOST; // use original value
	
	
    // update the options
    tcflush(fd, TCIOFLUSH);
    if(tcsetattr(fd, TCSANOW, &options) != 0)
    {
        std::cout << "Fail to set ttyAMA3 baudrate!" << std::endl;
        return -1;
    }
	return 0;
}


