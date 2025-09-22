
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#define __USE_GNU
#include <pthread.h>
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h> //PISOX中定义的标准接口 
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "head.h" 
#include "ethercat.h"


#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

#define EEP_MAN_SYNAPTICON (0x000022d2)
#define EEP_ID_SYNAPTICON (0x00000201)

#define num_zero 0
#define num_six 6
#define num_seven 7
#define num_fif 15

#define PP 1
#define PV 3
#define PT 4

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
uint64_t diff, maxt, avg, cycle;
struct timeval t1, t2;
int dorun = 0;
int64 toff = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int slaveloop = 0;

int32 pos_set;
int32 pos_Target;
int32 pos_Target_Right;
int32 pos_Target_Left;

int32 pos_Target_1 = -30000;
int32 pos_Target_2 = -60000;
int32 pos_Target_3 = 80000;
int32 pos_Target_4 = 300000;
int32 pos_Target_5 = 0;
int32 pos_Target_6 = 30000;
int32 pos_Target_7 = 40000;
int32 pos_Target_8 = -80000;
int32 pos_Target_9 = -300000;
int32 pos_Target_10 = 200000;

int32 pos_Turns;
int32 pos_Actual;
int32 L_RPM1;
int32 L_RPM5;
int32 L_RPM6;
int32 L_RPM10;
int32 aPlus_RPM;
int32 aRedu_RPM;
int32 h1_6093;
int32 h2_6093;
int32 speed;

int16 Controlword_6040;
int16 Statusword_6041;

double n;

uint32 buf32;
uint16 buf16;
uint8 buf8;
uint buf;


clock_t start,finish;//计时计数
int CountSend;
double cost;
long TimeStart,TimeRun,TimeFinish;



enum {
   STATE_RESET,
   STATE_INIT,
   STATE_PREREADY,
   STATE_READY,
   STATE_ENABLE,
   STATE_DISABLE
};

int state = STATE_RESET;

typedef struct PACKED
{
   uint16_t Control;
   int16_t TargetTor;
   int32_t TargetPos;
   int32_t TargetVel;
   uint8_t ModeOp;
   int16_t TorOff;
} Drive_Outputs;

typedef struct PACKED
{
   uint16_t sta;
   int32_t ActualPos;
   int32_t ActualVel;
   int16_t ActualTor;
   uint8_t ModeOp;
   int32_t SecPos;
} Drive_Inputs;

static int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;

   wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);

   return wkc;
}

static int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;

   wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);

   return wkc;
}

static int drive_write32(uint16 slave, uint16 index, uint8 subindex, int32 value)
{
   int wkc;

   wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);

   return wkc;
}

#define WRITE(slaveId, idx, sub, buf, value) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
    }

#define READ(slaveId, idx, sub, buf,comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("从站: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf,comment);    \
     }

int drive_setup(uint16 slave)
{
   int wkc = 0;

   printf("Drive setup\n");

   wkc += drive_write16(slave, 0x1C12, 0, 0);
   wkc += drive_write16(slave, 0x1C13, 0, 0);

   wkc += drive_write16(slave, 0x1A00, 0, 0);                                           wkc += drive_write16(slave, 0x1600, 0, 0);
   wkc += drive_write32(slave, 0x1A00, 1, 0x60410010);                                  wkc += drive_write32(slave, 0x1600, 1, 0x60400010);
   wkc += drive_write32(slave, 0x1A00, 2, 0x60640020);                                  wkc += drive_write32(slave, 0x1600, 2, 0x60710010);
   wkc += drive_write32(slave, 0x1A00, 3, 0x606C0020);                                  wkc += drive_write32(slave, 0x1600, 3, 0x607A0020);
   wkc += drive_write32(slave, 0x1A00, 4, 0x60770010);                                  wkc += drive_write32(slave, 0x1600, 4, 0x60FF0020);
   wkc += drive_write32(slave, 0x1A00, 5, 0x60610008);                                  wkc += drive_write32(slave, 0x1600, 5, 0x60600008);
   wkc += drive_write32(slave, 0x1A00, 6, 0x230A0020);                                  wkc += drive_write32(slave, 0x1600, 6, 0x60B20010);
   wkc += drive_write8(slave, 0x1A00, 0, 6);                                            wkc += drive_write8(slave, 0x1600, 0, 6);

   // wkc += drive_write8(slave, 0x1600, 0, 0);
   // wkc += drive_write32(slave, 0x1600, 1, 0x60400010); 
   // wkc += drive_write32(slave, 0x1600, 2, 0x60710010); 
   // wkc += drive_write32(slave, 0x1600, 3, 0x607A0020); 
   // wkc += drive_write32(slave, 0x1600, 4, 0x60FF0020); 
   // wkc += drive_write32(slave, 0x1600, 5, 0x60600008); 
   // wkc += drive_write32(slave, 0x1600, 6, 0x60B20010); 
   // wkc += drive_write8(slave, 0x1600, 0, 6);

   wkc += drive_write16(slave, 0x1C12, 1, 0x1600);
   wkc += drive_write8(slave, 0x1C12, 0, 1);

   wkc += drive_write16(slave, 0x1C13, 1, 0x1A00);
   wkc += drive_write8(slave, 0x1C13, 0, 1);

   strncpy(ec_slave[slave].name, "从站", EC_MAXNAME);

   if (wkc != 22)
   {
      printf("驱动 %d 启动失败。\nwkc: %d\n", slave, wkc);
      return -1;
   }
   else
      printf("驱动 %d 启动成功。\n", slave);

   return 0;
}
//系统设置  无需修改 无用
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if (ts->tv_nsec > NSEC_PER_SEC)
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

//还是系统设置 无需修改
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if (delta > (cycletime / 2))
   {
      delta = delta - cycletime;
   }
   if (delta > 0)
   {
      integral++;
   }
   if (delta < 0)
   {
      integral--;
   }
   *offsettime = -(delta / 100) - (integral / 20);
}

static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2)
{
   int64_t tdiff;
   tdiff = NSEC_PER_SEC * (int64_t)((int)t1.tv_sec - (int)t2.tv_sec);
   tdiff += ((int)t1.tv_nsec - (int)t2.tv_nsec);
   return tdiff;
}

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

/* 消除系统时钟偏移函数，取自cyclic_test */
static void set_latency_target(void)
{
   struct stat s;
   int ret;

   if (stat("/dev/cpu_dma_latency", &s) == 0)
   {
      latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
      if (latency_target_fd == -1)
         return;
      ret = write(latency_target_fd, &latency_target_value, 4);
      if (ret == 0)
      {
         printf("# error setting cpu_dma_latency to %d!: %s\n", latency_target_value, strerror(errno));
         close(latency_target_fd);
         return;
      }
      printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
   }
}

void slave1_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[1].inputs;
   // optr = (Drive_Outputs *)ec_slave[1].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   if(rad < 0 && rad >=-0.26)
   {
      pos_Target_1 = (rad*337692.3)-135000;
      WRITE(1,0x607A,0x00,buf32,pos_Target_1);//controlword 0 
      WRITE(1,0x6040,0x00,buf16,63);//controlword 0
   }
   else if(rad == 0)
   {
      pos_Target_1 = -135000;
      WRITE(1,0x607A,0x00,buf32,pos_Target_1);//controlword 0 
      WRITE(1,0x6040,0x00,buf16,63);//controlword 0
   }
   else if(rad > 0 && rad <= 0.4)
   {
      pos_Target_1 = (rad * 337500) - 135000;
      WRITE(1,0x607A,0x00,buf32,pos_Target_1);//controlword 0 
      WRITE(1,0x6040,0x00,buf16,63);//controlword 0
   }

   double ActualPOS = iptr->ActualPos;
   //UDP发送
   rad = 0.4-((ActualPOS/(-222800))*0.66);
   // printf("rad: %f pos: %f\n",rad ,ActualPOS/222800);
   UDP_send_buf[1] = rad;
   UDP_send_buf[11] = ActualPOS;

   // usleep(500);
   // WRITE(1,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(1,0x607A,0x00,buf32,pos_Target_1);//controlword 0 
   // WRITE(1,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(1,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(1,0x6084,0x00,buf32,300000);//controlword 0
   //////////WRITE(1,0x6040,0x00,buf16,63);//controlword 0
   // WRITE(1,0x607A,0x00,buf32,pos_Target_1);//controlword 0 


   // if (iptr->ActualPos <= -130000)
   // {
   //    pos_Target_1 = -30000;
   // }
   // else if(iptr->ActualPos >= -30000)
   // {
   //    pos_Target_1 = -130000;
   // }


}

void slave2_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[2].inputs;
   // optr = (Drive_Outputs *)ec_slave[2].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;


   // WRITE(2,0x607A,0x00,buf32,pos_Target_2);//controlword 0 
   // WRITE(2,0x6081,0x00,buf32,800000);//controlword 0
   // WRITE(2,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(2,0x6084,0x00,buf32,300000);//controlword 0

   // usleep(500);
   // WRITE(2,0x6040,0x00,buf16,15);//controlword 0
   ////////////WRITE(2,0x607A,0x00,buf32,pos_Target_2);//controlword 0 
   // usleep(500);
   ///////////WRITE(2,0x6040,0x00,buf16,0x3F);//controlword 0
   // WRITE(2,0x607A,0x00,buf32,pos_Target_2);//controlword 0 


   // if (iptr->ActualPos <= -120000)
   // {
   //    pos_Target_2 = -60000;
   // }
   // else if(iptr->ActualPos >= -60000)
   // {
   //    pos_Target_2 = -120000;
   // }
   

   // else if(ModeOp == PT)
   // {
   //    WRITE(2,0x6071,0x00,buf32,300);
   //    WRITE(2,0x6087,0x00,buf32,1500);

   //    printf("+++++++++++++++++++++++++++++");
   // }


   // if(rad <= 0)
   // {
   //    // pos_Target_2 = ((0.24+rad)/0.24)*(-80000);
   //    pos_Target_2 = (0.54+rad)*(-333333);
   //    WRITE(2,0x607A,0x00,buf32,pos_Target_2);
   //    WRITE(2,0x6040,0x00,buf16,0x3F);
   // }
   // else if(rad > 0)
   // {
   //    // pos_Target_2 = ((rad+0.24)/0.78)*(-260000);
   //    pos_Target_2 = ((rad+0.54)/0.78)*(-260000);
   //    WRITE(2,0x607A,0x00,buf32,pos_Target_2);
   //    WRITE(2,0x6040,0x00,buf16,0x3F);
   // }

   if(rad >=-0.34 && rad <= 0.34)
   {
      pos_Target_2 = (0.34+rad)*(-382353);
      WRITE(2,0x607A,0x00,buf32,pos_Target_2);
      WRITE(2,0x6040,0x00,buf16,0x3F);
   }

    double ActualPOS = iptr->ActualPos;

   //UDP发送
   rad = (ActualPOS/(-382353))-0.34;
   // printf("rad: %f pos: %f\n",rad ,ActualPOS);
   UDP_send_buf[2] = rad;
   UDP_send_buf[12] = ActualPOS;

}

void slave3_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[3].inputs;
   // optr = (Drive_Outputs *)ec_slave[3].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   usleep(500);
   // WRITE(3,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(3,0x607A,0x00,buf32,pos_Target_3);//controlword 0 
   // WRITE(3,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(3,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(3,0x6084,0x00,buf32,300000);//controlword 0
   //////////WRITE(3,0x6040,0x00,buf16,63);//controlword 0

   // if (iptr->ActualPos <= 0)
   // {
   //    pos_Target_3 = 80000;
   // }
   //    else if(iptr->ActualPos >= 80000)
   // {
   //    pos_Target_3 = 0;
   // }
   
   // else if(ModeOp == PT)
   // {

   // }

   if(rad >= 0.35 && rad <= 1.05)
   {
      pos_Target_3 = ((rad-0.35)/0.7)*237000;
      WRITE(3,0x607A,0x00,buf32,pos_Target_3);
      WRITE(3,0x6040,0x00,buf16,63);
   }

   double ActualPOS = iptr->ActualPos;

   //UDP发送
   rad = ((ActualPOS/237000)*0.7)+0.35;
   UDP_send_buf[3] = rad;
   UDP_send_buf[13] = ActualPOS;
}

void slave4_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[4].inputs;
   // optr = (Drive_Outputs *)ec_slave[4].outputs;



   //////////WRITE(4,0x607A,0x00,buf32,pos_Target_4);//controlword 0  
   // WRITE(4,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(4,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(4,0x6084,0x00,buf32,300000);//controlword 0

   // usleep(500);
   // WRITE(4,0x6040,0x00,buf16,47);//controlword 0
   // WRITE(4,0x607A,0x00,buf32,pos_Target_4);//controlword 0  
   // WRITE(4,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(4,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(4,0x6084,0x00,buf32,300000);//controlword 0

   //////////WRITE(4,0x6040,0x00,buf16,63);//controlword 0
   // if (iptr->ActualPos >= 300000)
   // {
   //    pos_Target_4 = 200000;
   // }
   //    else if(iptr->ActualPos <= 200000)
   // {
   //    pos_Target_4 = 300000;
   // }

   if(rad <= -0.79 && rad >= -2.18)
   {
      pos_Target_4 = -((rad + 0.79)/1.39)*475000;
      WRITE(4,0x607A,0x00,buf32,pos_Target_4);
      WRITE(4,0x6040,0x00,buf16,63);

      // if(iptr->ActualPos >= 274500)
      // {
      //    pos_Target_4 = 274500;
      //    printf("4444444444444777777777777777777766666666666");
      // }
      // else 
      // {
      //    pos_Target_4 = -((rad + 0.79)/1.39)*475000;
      // }

      // WRITE(4,0x607A,0x00,buf32,pos_Target_4);
      // WRITE(4,0x6040,0x00,buf16,63);
   }

   double ActualPOS = iptr->ActualPos;

   //UDP发送
   rad = (((ActualPOS/(-475000))*1.39)-0.79);
   UDP_send_buf[4] = rad;
   UDP_send_buf[14] = ActualPOS;
   
}

void slave5_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[5].inputs;
   // optr = (Drive_Outputs *)ec_slave[5].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   // usleep(500);
   // WRITE(5,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(5,0x607A,0x00,buf32,pos_Target_5);//controlword 0 
   // WRITE(5,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(5,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(5,0x6084,0x00,buf32,300000);//controlword 0
   //////////WRITE(5,0x6040,0x00,buf16,63);//controlword 0


   // if (iptr->ActualPos >= 0)
   // {
   //    pos_Target_5 = -190000;
   // }
   //    else if(iptr->ActualPos <= -190000)
   // {
   //    pos_Target_5 = 0;
   // }

   if(rad >= -1.01 && rad <= 0)
   {
      pos_Target_5 = (0-rad)*(-200990);
      WRITE(5,0x607A,0x00,buf32,pos_Target_5);
      WRITE(5,0x6040,0x00,buf16,63);   
   }

   double ActualPOS = iptr->ActualPos;

   rad = ActualPOS/200990;
   UDP_send_buf[5] = rad;
   UDP_send_buf[15] = ActualPOS;
}

void slave6_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[6].inputs;
   // optr = (Drive_Outputs *)ec_slave[6].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   // usleep(500);
   // WRITE(6,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(6,0x607A,0x00,buf32,pos_Target_6);//controlword 0 
   // WRITE(6,0x6081,0x00,buf32,8000);//controlword 0 
   // WRITE(6,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(6,0x6084,0x00,buf32,300000);//controlword 0

   //////////WRITE(6,0x6040,0x00,buf16,63);//controlword 0

   // if (iptr->ActualPos >= 90000)
   // {
   //    pos_Target_6 = 30000;
   // }
   //    else if(iptr->ActualPos <= 30000)
   // {
   //    pos_Target_6 = 90000;
   // }

   if(rad >= -0.4 && rad <= 0.26)
   {
      pos_Target_6 = (rad+0.4)*337575.75;
      WRITE(6,0x607A,0x00,buf32,pos_Target_6);
      WRITE(6,0x6040,0x00,buf16,63);
   }
   // pos_Target_6 = 0;

   double ActualPOS = iptr->ActualPos;

   rad = (ActualPOS/337575.75)-0.4;
   UDP_send_buf[6] = rad;
   UDP_send_buf[16] = ActualPOS;
}

void slave7_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[7].inputs;
   // optr = (Drive_Outputs *)ec_slave[7].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   usleep(500);
   // WRITE(7,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(7,0x607A,0x00,buf32,pos_Target_7);//controlword 0 
   // WRITE(7,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(7,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(7,0x6084,0x00,buf32,300000);//controlword 0
   //////////WRITE(7,0x6040,0x00,buf16,63);//controlword 0

   // if (iptr->ActualPos >= 120000)
   // {
   //    pos_Target_7 = 40000;
   // }
   //    else if(iptr->ActualPos <= 40000)
   // {
   //    pos_Target_7 = 120000;
   // }

   if(rad >= -0.34 && rad <= 0.34 )
   {
      pos_Target_7 = (0.34-rad)*(382353);
      WRITE(7,0x607A,0x00,buf32,pos_Target_7);
      WRITE(7,0x6040,0x00,buf16,63);
   }

   double ActualPOS = iptr->ActualPos;

   rad = 0.34-(ActualPOS/382353);
   UDP_send_buf[7] = rad;
   UDP_send_buf[17] = ActualPOS;

}
void slave8_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[8].inputs;
   // optr = (Drive_Outputs *)ec_slave[8].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   usleep(500);
   // WRITE(8,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(8,0x607A,0x00,buf32,pos_Target_8);//controlword 0 
   // WRITE(8,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(8,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(8,0x6084,0x00,buf32,300000);//controlword 0
   ///////////WRITE(8,0x6040,0x00,buf16,63);//controlword 0

   // if (iptr->ActualPos >= 0)
   // {
   //    pos_Target_8 = -80000;
   // }
   //    else if(iptr->ActualPos <= -80000)
   // {
   //    pos_Target_8 = 0;
   // }

   if(rad >= 0.35 && rad <= 1.05)
   {
      pos_Target_8 = (rad-0.35)*(-334286);
      WRITE(8,0x607A,0x00,buf32,pos_Target_8);
      WRITE(8,0x6040,0x00,buf16,63);
   }

   double ActualPOS = iptr->ActualPos;

   rad = (ActualPOS/(-334286))+0.35;
   UDP_send_buf[8] = rad;
   UDP_send_buf[18] = ActualPOS;

}

void slave9_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[9].inputs;
   // optr = (Drive_Outputs *)ec_slave[9].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   // WRITE(9,0x607A,0x00,buf32,pos_Target_9);//controlword 0  
   // WRITE(9,0x6081,0x00,buf32,800000);//controlword 0 
   // WRITE(9,0x6083,0x00,buf32,300000);//controlword 0
   // WRITE(9,0x6084,0x00,buf32,300000);//controlword 0

   usleep(500);
   // WRITE(9,0x6040,0x00,buf16,47);//controlword 0
   //////////WRITE(9,0x607A,0x00,buf32,pos_Target_9);//controlword 0  
   //////////WRITE(9,0x6040,0x00,buf16,63);//controlword 0

   // if (iptr->ActualPos <= -300000)
   // {
   //    pos_Target_9 = -150000;
   // }
   //    else if(iptr->ActualPos >= -150000)
   // {
   //    pos_Target_9 = -300000;
   // }

   if(rad >= -2.18 && rad <= -0.79)
   {
      pos_Target_9 = ((rad + 0.79)/1.39)*475000;
      WRITE(9,0x607A,0x00,buf32,pos_Target_9);
      WRITE(9,0x6040,0x00,buf16,63);
   }

   double ActualPOS = iptr->ActualPos;

   rad = ((ActualPOS/475000)*1.39)-0.79;
   UDP_send_buf[9] = rad;
   UDP_send_buf[19] = ActualPOS;

}

void slave10_run(double rad)
{
   Drive_Inputs *iptr;
   // Drive_Outputs *optr;
   iptr = (Drive_Inputs *)ec_slave[10].inputs;
   // optr = (Drive_Outputs *)ec_slave[10].outputs;

   // iptr->sta = 0;
   // optr->Control = 0;

   if(rad >= -1.01 && rad <= 0)
   {
      pos_Target_10 = ((0-rad)/1.01)*203000;
      WRITE(10,0x607A,0x00,buf32,pos_Target_10);
      WRITE(10,0x6040,0x00,buf16,63);
   }

   double ActualPOS = iptr->ActualPos;

   rad = 0-((ActualPOS/203000)*1.01);
   UDP_send_buf[10] = rad;
   UDP_send_buf[20] = ActualPOS;

   // sendto(UDP_send_sockfd,UDP_send_buf,sizeof(UDP_send_buf),0,(struct sockaddr *)&srvaddr_send,len_send);
}

/**
 * Returns the current time in microseconds.
 */
long getMicrotime()
{
   struct timeval currentTime;
   gettimeofday(&currentTime, NULL);
   return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}


void SendData(int slave_loop,int ModeOp,double rad)
{
   // double Rad,Rad_1,Rad_2,Rad_3,Rad_4,Rad_5,Rad_6,Rad_7,Rad_8,Rad_9,Rad_10;
   if(ModeOp == PP)
   {
      switch(slave_loop)
      {
// 
         case 1:
            // printf("-0.26~0.4:");
            // scanf("%lf",&Rad);
            // rad = Rad;
            // start = clock();
            slave1_run(Rad_1);
            break;
         case 2:
            // printf("-0.34~0.34:");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave2_run(Rad_2);
            break;
         case 3:
            // printf("0.35~1.05:");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave3_run(Rad_3);
            break;
         case 4:
            // printf("-2.18~-0.79:");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave4_run(Rad_4);
            break;
         case 5:
            // printf("-1.01~0:");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave5_run(Rad_5);
            break;
         case 6:
            // printf("-0,4~0.26:");
            // scanf("%lf",&Rad);
            // rad = Rad;
            // rad_6 = -0.4;
            slave6_run(Rad_6);
            break;
         case 7:
            // printf("-0.34~0.34");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave7_run(Rad_7);
            break;
         case 8:
            // printf("0.35~1.05");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave8_run(Rad_8);
            break;
         case 9:
            // printf("-2.18~-0.79");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave9_run(Rad_9);
            break;
         case 10:
            // printf("-1.01~0");
            // scanf("%lf",&Rad);
            // rad = Rad;
            slave10_run(Rad_10);
            // CountSend++;
            // // finish = clock();
            // // cost = (double)(finish - start)/CLOCKS_PER_SEC;
            // // printf("startclock %ld\n",start);
            // // printf("finishclock %ld\n",finish);
            // // printf("cost %f\n",cost);

            // TimeFinish = getMicrotime();
            // printf("TimeFinish %ld\n",TimeFinish);
            // printf("(TimeFinish - TimeStart)/1000000 :%ld\n",(TimeFinish - TimeStart));
            // // if((finish - start)/1000 >= 1)
            // if((TimeFinish - TimeStart)/1000000 >= 1)
            // {
            //    printf("CountSend = %d\n", CountSend);
            // }
            break;
      }

   }
   else if(ModeOp == PT)
   {
      // WRITE(1,0x6071,0x00,buf32,250);
      // WRITE(2,0x6071,0x00,buf32,-200);
      // WRITE(3,0x6071,0x00,buf32,-200);
      // WRITE(4,0x6071,0x00,buf32,380);
      // WRITE(5,0x6071,0x00,buf32,200);
      // WRITE(6,0x6071,0x00,buf32,-300);
      // WRITE(7,0x6071,0x00,buf32,200);
      // WRITE(8,0x6071,0x00,buf32,200);
      // WRITE(9,0x6071,0x00,buf32,-380);
      // WRITE(10,0x6071,0x00,buf32,-200);

      // WRITE(1,0x6087,0x00,buf32,500);
      // WRITE(2,0x6087,0x00,buf32,1500); 
      // WRITE(3,0x6087,0x00,buf32,1500);
      // WRITE(4,0x6087,0x00,buf32,1500);
      // WRITE(5,0x6087,0x00,buf32,1500);
      // WRITE(6,0x6087,0x00,buf32,1500);
      // WRITE(7,0x6087,0x00,buf32,1500); 
      // WRITE(8,0x6087,0x00,buf32,1500);
      // WRITE(9,0x6087,0x00,buf32,1500);
      // WRITE(10,0x6087,0x00,buf32,1500);


      WRITE(10,0x6071,0x00,buf32,-200);
      WRITE(9,0x6071,0x00,buf32,-380);
      WRITE(8,0x6071,0x00,buf32,200);
      WRITE(7,0x6071,0x00,buf32,200);
      WRITE(6,0x6071,0x00,buf32,-300);
      WRITE(5,0x6071,0x00,buf32,200);
      WRITE(4,0x6071,0x00,buf32,380);
      WRITE(3,0x6071,0x00,buf32,-200);
      WRITE(2,0x6071,0x00,buf32,-200);
      WRITE(1,0x6071,0x00,buf32,250);

      WRITE(10,0x6087,0x00,buf32,1500);
      WRITE(9,0x6087,0x00,buf32,1500);
      WRITE(8,0x6087,0x00,buf32,1500);
      WRITE(7,0x6087,0x00,buf32,1500); 
      WRITE(6,0x6087,0x00,buf32,1500);
      WRITE(5,0x6087,0x00,buf32,1500);
      WRITE(4,0x6087,0x00,buf32,1500);
      WRITE(3,0x6087,0x00,buf32,1500);
      WRITE(2,0x6087,0x00,buf32,1500); 
      WRITE(1,0x6087,0x00,buf32,500);



      
   }
   // slave1_run();
   // slave2_run();
   // slave3_run();
   // slave4_run();
   // slave5_run();
   // slave6_run();
   // slave7_run();
   // slave8_run();
   // slave9_run();
   // slave10_run();
}

// SendData_PT(int slave_loop)
// {
//       switch(slave_loop)
//    {
//       case 1:
//          slave1_run(PT);
//          break;
//       case 2:
//          slave2_run(PT);
//          break;
//       case 3:
//          slave3_run(PT);
//          break;
//       case 4:
//          slave4_run(PT);
//          break;
//       case 5:
//          slave5_run(PT);
//          break;
//       case 6:
//          slave6_run(PT);
//          break;
//       case 7:
//          slave7_run(PT);
//          break;
//       case 8:
//          slave8_run(PT);
//          break;
//       case 9:
//          slave9_run(PT);
//          break;
//       case 10:
//          slave10_run(PT);
//          break;
//    }
// }


void test_driver(char *ifname,char *slavename)
{
   int ModeOp;
   int cnt, i ,SlaveName/*,j*/;
   Drive_Inputs *iptr;
   Drive_Outputs *optr;
   struct sched_param schedp;
   cpu_set_t mask;
   pthread_t thread;
   int ht;
   int chk = 2000;
   int64 cycletime;
   struct timespec ts, tnow;

   SlaveName = atoi(slavename);

   CPU_ZERO(&mask);
   CPU_SET(2, &mask);
   thread = pthread_self();
   pthread_setaffinity_np(thread, sizeof(mask), &mask);

   memset(&schedp, 0, sizeof(schedp));
   schedp.sched_priority = 99; /* 设置优先级为99，即RT */
   sched_setscheduler(0, SCHED_FIFO, &schedp);

   printf("Starting Redundant test\n");

   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);

      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);

         int slave_ix;
         for (slave_ix = 1; slave_ix <= ec_slavecount; slave_ix++)
         {
            ec_slavet *slave = &ec_slave[slave_ix];
            slave->PO2SOconfig = drive_setup;
         }

         ec_config_map(&IOmap); // 此处调用setup函数，PDO设置
         ec_configdc(); // 设置同步时钟，该函数必须在设置pdo之后 经过实验会有BUG 草草草

         // setup dc for devices
         for (slave_ix = 1; slave_ix <= ec_slavecount; slave_ix++)
         {
            ec_dcsync0(slave_ix, TRUE, 4000000U, 20000U);
            // ec_dcsync01(slave_ix, TRUE, 4000000U, 8000000U, 20000U);
         }

         printf("从站进入准备 SAFE_OP.\n");
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

 
         ec_readstate();
         for (cnt = 1; cnt <= ec_slavecount; cnt++)
         {
            printf("从站:%d 名:%s 输出字节大小:%3dbits 输入字节大小:%3dbits 当前状态:%2d 延迟:%d.%d\n",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
         }
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("计算 当前工作字 %d\n", expectedWKC);

         printf("对所有从站请求操作s\n");

         ec_slave[0].state = EC_STATE_OPERATIONAL;

         ec_writestate(0);

         clock_gettime(CLOCK_MONOTONIC, &ts);
         ht = (ts.tv_nsec / 1000000) + 1; 
         ts.tv_nsec = ht * 1000000;
         cycletime = 4000 * 1000; 

         /* 对PDO进行初始化 */
         for (i = 0; i < ec_slavecount; i++)
         {
            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
            if(optr == NULL)
            {
               printf("optr is NULL.\n");
            }
            optr->Control = 0;
            optr->TargetPos = 0;
            optr->ModeOp = 0;
            optr->TargetTor = 0;
            optr->TargetVel = 0;
            optr->TorOff = 0;
         }

         do
         {
            /* wait to cycle start */
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
            if (ec_slave[0].hasdc)
            {
               /* calulate toff to get linux time and DC synced */
               ec_sync(ec_DCtime, cycletime, &toff);
            }
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            ec_send_processdata();
            add_timespec(&ts, cycletime + toff);
         } while (chk-- && (wkc != expectedWKC)); 
         /* ec_statecheck函数消耗的时间较多，有可能超过循环周期 */

         if (wkc == expectedWKC)
         {
            printf("所有设备现在是操作状态.\n");
            inOP = TRUE;
            cnt = 0;
            // start = clock();
            // TimeStart = getMicrotime();
            // // printf("start clock : %ld\n",start);
            // printf("TimeStart : %ld\n",TimeStart);
            while (cnt<10)
            {
               /* 计算下一周期唤醒时间 */
               add_timespec(&ts, cycletime + toff);
               /* wait to cycle start */
               clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
               clock_gettime(CLOCK_MONOTONIC, &tnow);
               if (ec_slave[0].hasdc)
               {
                  /* calulate toff to get linux time and DC synced */
                  ec_sync(ec_DCtime, cycletime, &toff);
               }
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               diff = calcdiff_ns(tnow, ts);

               switch (state)
               {
               case STATE_RESET: /* 清障 */
                  for (i = 0; i < ec_slavecount; i++)
                  {
                     optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                     optr->Control = 128;
                  }
                  state = STATE_INIT;
                  //state = 0;
                  break;
               case STATE_INIT /* 初始化 */:
                  for (i = 0; i < ec_slavecount; i++)
                  {
                     optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                     iptr = (Drive_Inputs *)ec_slave[i + 1].inputs;
                     optr->Control = 0;
                  }
                  state = STATE_PREREADY;
                  //state = 0;
                  break;
               case STATE_PREREADY:
                  for (i = 0; i < ec_slavecount; i++)
                  {
                     optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                     optr->Control = 6;
                  }
                  state = STATE_READY;
                  //state = 0;
                  break;
               case STATE_READY:
                  for (i = 0; i < ec_slavecount; i++)
                  {
                     optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                     optr->Control = 7;
                     ModeOp = optr->ModeOp = 1;//pv:3 pp:1 csp:8 PT:4
                  }
                  state = STATE_ENABLE;
                  break;
               case STATE_ENABLE: //准备
                  for (i = 0; i < ec_slavecount; i++)
                  {
                     optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                     optr->Control = 15;
                  }
                  break;
               case STATE_DISABLE:
               /* 使电机失能并在10个循环之后退出循环 */
                  for (i = 0; i < ec_slavecount; i++)
                  {
                     optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                     optr->ModeOp = 0;
                     optr->TargetVel = 0;
                     optr->Control = num_six;
                     // usleep(5000);
                     // WRITE(1,0x6040,0x00,buf16,6);
                  }
                  
                  cnt ++;
                  break;
                  default:
                  break;
               }
               ec_send_processdata();
               // ec_dcsync0(slave_ix, TRUE, 4000000U, 20000U);
               cycle++;

               avg += diff;
               if (diff > maxt)
                  maxt = diff;
               optr = (Drive_Outputs *)ec_slave[1].outputs;


               for (int j = SlaveName-1; j < SlaveName; j++)
               {
                  iptr = (Drive_Inputs *)ec_slave[j + 1].inputs;
                  optr = (Drive_Outputs *)ec_slave[j + 1].outputs;
                  // printf("  %d: CW: %d, status: %d, pos: %d, 速度: %d", j + 1, optr->Control, iptr->sta, iptr->ActualPos, iptr->ActualVel);
               }

               // for(i = 0; i < ec_slavecount; i++)
               // {
               //    iptr = (Drive_Inputs *)ec_slave[i + 1].inputs;
               //    optr = (Drive_Outputs *)ec_slave[i + 1].outputs;

               //    if(iptr->ModeOp == 1)
               //    {
               //       SendData(slaveloop+1,PP);//pp
               //    }
               //    else if(iptr->ModeOp == 4)
               //    {  
               //       SendData(slaveloop+1,PT);
               //    }

               //    slaveloop = slaveloop + 1;
               //    if(slaveloop == 10)
               //    {
               //       slaveloop =0;
               //    }

               // }
if(ModeOp == PP)
{
   SendData(slaveloop+1,PP,0);//pp
}
else if(ModeOp == PT)
{
   SendData(slaveloop+1,PT,0);//pt
}

slaveloop = slaveloop + 1;
if(slaveloop == 10)
{
   slaveloop =0;
}

      // WRITE(2,0x6071,0x00,buf32,300);
      // WRITE(4,0x6071,0x00,buf32,350);
      // WRITE(2,0x6087,0x00,buf32,1500);

      
      // WRITE(4,0x6087,0x00,buf32,1500);



               // printf(", MaxLatency: %lu, avg: %lu    \r", maxt, avg / cycle);
               fflush(stdout);
            }
            dorun = 0;

            for(int i = 0; i < 10; i++)
            {
               WRITE(i+1,0x6040,0x00,buf16,6)
               // WRITE(i+1,0x6040,0x00,buf16,0)
               // WRITE(i+1,0x6081,0x00,buf32,0);//controlword 0 
               // WRITE(i+1,0x6083,0x00,buf32,0);//controlword 0
               // WRITE(i+1,0x6084,0x00,buf32,0);//controlword 0
            }
         }
         else /* ECAT进入OP失败 */
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         /* 断开ECAT通讯 */
         printf("\nRequest safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
         ec_slave[0].state = EC_STATE_PRE_OP;
         ec_writestate(0);
         ec_slave[0].state = EC_STATE_INIT;
         ec_writestate(0);
         ec_readstate();
         if (ec_statecheck(0, EC_STATE_SAFE_OP, 1000) == EC_STATE_INIT)
         {
            printf("ECAT changed into state init\n");
         }
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End driver test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", ifname);
   }
}

// 检测键盘输入，如检测到esc即关闭SOEM退出程序
OSAL_THREAD_FUNC scanKeyboard()
{
   int in;
   // int i;
   // Drive_Outputs *optr;
   struct sched_param schedp;
   cpu_set_t mask;
   pthread_t thread;
   struct termios new_settings;
   struct termios stored_settings;

   CPU_ZERO(&mask);
   CPU_SET(2, &mask);
   thread = pthread_self();
   pthread_setaffinity_np(thread, sizeof(mask), &mask);
   memset(&schedp, 0, sizeof(schedp));
   schedp.sched_priority = 20;
   sched_setscheduler(0, SCHED_FIFO, &schedp);

   tcgetattr(0, &stored_settings);
   new_settings = stored_settings;
   new_settings.c_lflag &= (~ICANON); //屏蔽整行缓存
   new_settings.c_cc[VTIME] = 0;

   /*这个函数调用把当前终端接口变量的值写入termios_p参数指向的结构。
   如果这些值其后被修改了，你可以通过调用函数tcsetattr来重新配置
   调用tcgetattr初始化一个终端对应的termios结构
   int tcgetattr(int fd, struct termios *termios_p);*/
   tcgetattr(0, &stored_settings);
   new_settings.c_cc[VMIN] = 1;

   /*int tcsetattr(int fd , int actions , const struct termios *termios_h)
   参数actions控制修改方式，共有三种修改方式，如下所示。
   1.TCSANOW：立刻对值进行修改
   2.TCSADRAIN：等当前的输出完成后再对值进行修改。
   3.TCSAFLUSH：等当前的输出完成之后，再对值进行修改，但丢弃还未从read调用返回的当前的可用的任何输入。*/
   tcsetattr(0, TCSANOW, &new_settings);
   in = getchar();
   tcsetattr(0, TCSANOW, &stored_settings);
   while (1)
   {
      if (in == 27)
      {
         state = STATE_DISABLE;

         printf("the keyboard input is: \n");
         putchar(in);
         break;
      }
      osal_usleep(10000); //间隔10ms循环一次;
   }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */
   struct sched_param schedp;
   cpu_set_t mask;
   pthread_t thread;
   time_t terr;

   /* 设定线程优先级为20 */
   CPU_ZERO(&mask);
   CPU_SET(2, &mask);
   thread = pthread_self();
   pthread_setaffinity_np(thread, sizeof(mask), &mask);

   memset(&schedp, 0, sizeof(schedp));
   schedp.sched_priority = 21;
   sched_setscheduler(0, SCHED_FIFO, &schedp);

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         time(&terr);
         printf("wkc: %d, expwkc: %d, docheckstate: %d, error time: %s\n", wkc, expectedWKC, ec_group[0].docheckstate, ctime(&terr));
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
   }
}

#define stack64k (64 * 1024)

void init_UDP_send()
{
   //1. 创建一个UDP套接字
   // int sockfd;
   UDP_send_sockfd = socket(AF_INET,SOCK_DGRAM,0);
   
   //2. 准备对方的地址
   // struct sockaddr_in srvaddr_send;
   len_send = sizeof(srvaddr_send);
   bzero(&srvaddr_send,len_send);
   
   srvaddr_send.sin_family = AF_INET;
   // srvaddr.sin_port = htons(atoi(argv[2]));
   srvaddr_send.sin_port = htons(50004);//接收50006
   // inet_pton(AF_INET,argv[1],&srvaddr.sin_addr);
   inet_pton(AF_INET,"192.168.3.58"/*目标IP*/,&srvaddr_send.sin_addr);
}

void init_UDP_receive()
{
   //1. 创建一个UDP套接字
   // int sockfd;
   UDP_receivr_sockfd = socket(AF_INET,SOCK_DGRAM,0);
   
   //2. 绑定IP地址，端口号到套接字上
   // struct sockaddr_in srvaddr;
   // socklen_t len = sizeof(srvaddr);
   len_receive = sizeof(srvaddr_receive);
   bzero(&srvaddr_receive,len_receive);
   
   srvaddr_receive.sin_family = AF_INET;
   // srvaddr.sin_port = htons(atoi(argv[1])); 50001
   srvaddr_receive.sin_port = htons(50006);//从50006端口接收 
   srvaddr_receive.sin_addr.s_addr = htonl(INADDR_ANY);
   
   bind(UDP_receivr_sockfd,(struct sockaddr *)&srvaddr_receive,len_receive);
}


void *UDP_receive() 
{
   //3. 不断接收对方给自己发送过来的信息
         printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

   struct sockaddr_in cliaddr; //谁给我寄信，这个cliaddr就存放着谁的地址
   // int ret_udp;
   while(1)
   {
      bzero(UDP_receive_buf,sizeof(UDP_receive_buf));
      recvfrom(UDP_receivr_sockfd,UDP_receive_buf,sizeof(UDP_receive_buf),0,(struct sockaddr *)&cliaddr,&len_receive);
      printf("%s:%f \n",(char *)inet_ntoa(cliaddr.sin_addr),UDP_receive_buf[0]);

      Rad_1 = UDP_receive_buf[0];            Rad_6 = UDP_receive_buf[5];
      Rad_2 = UDP_receive_buf[1];            Rad_7 = UDP_receive_buf[6];
      Rad_3 = UDP_receive_buf[2];            Rad_8 = UDP_receive_buf[7];
      Rad_4 = UDP_receive_buf[3];            Rad_9 = UDP_receive_buf[8];
      Rad_5 = UDP_receive_buf[4];            Rad_10 = UDP_receive_buf[9]; 
      printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      printf(" %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",	UDP_receive_buf[1],UDP_receive_buf[2],UDP_receive_buf[3],UDP_receive_buf[4],UDP_receive_buf[5],
      																	UDP_receive_buf[6],UDP_receive_buf[7],UDP_receive_buf[8],UDP_receive_buf[9],UDP_receive_buf[10]);  
   }
}

void *Send_Data()
{
   while(1)
   {
      int runtime = 1;
      int i = 0;
      TimeStart = getMicrotime();
      printf("TimeStart : %ld\n",TimeStart);
      while(1)
      {
         if(runtime == 1)
         {
            TimeRun = getMicrotime();
            runtime = 0;
         }
         TimeFinish = getMicrotime();

         if((TimeFinish - TimeRun) >= 989)
         {
            i++;
            
            if((TimeFinish - TimeStart)/1 >= 1000000)
            {
               printf("(TimeFinish - TimeStart) = %ld\n",(TimeFinish - TimeStart));
               printf("TimeFinish : %ld\n\n",TimeFinish);
               printf("频率 i = %dHz \n",i);
               break;
            }

            sendto(UDP_send_sockfd,UDP_send_buf,sizeof(UDP_send_buf),0,(struct sockaddr *)&srvaddr_send,len_send);

            if(i >= 995)
            {
               printf("i = %d\n",i);
            }

            runtime = 1; 
         }
      }
   }
} 

// int main(int argc, char *argv[])
// {
//    //int mode;

//    init_UDP_send();
//    init_UDP_receive();

//    printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");
//    if (argc > 1)
//    {
//       dorun = 0;

//       set_latency_target(); // 消除系统时钟偏移

//       /* create thread to handle slave error handling in OP */
//       osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);//ether自检
//       osal_thread_create(&thread1, stack64k * 4, &scanKeyboard, NULL);//监测键盘

//       pthread_t udp_receive;
//       pthread_create(&udp_receive,NULL,UDP_receive,NULL);//创建udp接收信息线程

//       pthread_t send_data;
//       pthread_create(&send_data,NULL,Send_Data,NULL);//创建udp发送信息线程

//       test_driver(argv[1],argv[2]);
//       for(int i = 0; i < 10; i++)
//       {
//          WRITE(i+1,0x6040,0x00,buf16,6)
//          // WRITE(i+1,0x6040,0x00,buf16,0)
//          // WRITE(i+1,0x6081,0x00,buf32,0);//controlword 0 
//          // WRITE(i+1,0x6083,0x00,buf32,0);//controlword 0
//          // WRITE(i+1,0x6084,0x00,buf32,0);//controlword 0

//       }

//       printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
//    }
//    else
//    {
//       printf("Usage: red_test ifname1 Mode_of_operation\nifname = eth0 for example\n");
//    }

//    printf("项目关闭成功\n");

//    return (0);
// }


int fun_main()
{
   //int mode;

   init_UDP_send();
   init_UDP_receive();

   printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");
   
   dorun = 0;

   set_latency_target(); // 消除系统时钟偏移

   /* create thread to handle slave error handling in OP */
   osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);//ether自检
   osal_thread_create(&thread1, stack64k * 4, &scanKeyboard, NULL);//监测键盘

   pthread_t udp_receive;
   pthread_create(&udp_receive,NULL,UDP_receive,NULL);//创建udp接收信息线程

   pthread_t send_data;
   pthread_create(&send_data,NULL,Send_Data,NULL);//创建udp发送信息线程

   // test_driver(argv[1],argv[2]);
   test_driver("enp45s0","1");
   for(int i = 0; i < 10; i++)
   {
      WRITE(i+1,0x6040,0x00,buf16,6)
      // WRITE(i+1,0x6040,0x00,buf16,0)
      // WRITE(i+1,0x6081,0x00,buf32,0);//controlword 0 
      // WRITE(i+1,0x6083,0x00,buf32,0);//controlword 0
      // WRITE(i+1,0x6084,0x00,buf32,0);//controlword 0

   }

   printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
   

   printf("项目关闭成功\n");

   return (0);
}
