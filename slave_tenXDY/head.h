#ifndef _HEAD_H_
#define _HEAD_H_

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <dirent.h>
#include <sys/wait.h>
#include <signal.h>
#include <linux/input.h>
#include <time.h>
#include <stdbool.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

/***********************************UDP**************************************/
//udp发送
int UDP_send_sockfd;
struct sockaddr_in srvaddr_send;
socklen_t len_send;

double UDP_send_buf[50];


//udp接收
int UDP_receivr_sockfd;
struct sockaddr_in srvaddr_receive;
socklen_t len_receive;
double UDP_receive_buf[50];


double Rad,
	   Rad_1,
	   Rad_2,
	   Rad_3,
	   Rad_4 = -0.79,
	   Rad_5,
	   Rad_6,
	   Rad_7,
	   Rad_8,
	   Rad_9 = -0.79,
	   Rad_10;


/*************************声明调用的函数***************************/

int fun_main();

#endif
