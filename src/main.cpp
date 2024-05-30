#include <iostream>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include "soem/ethercat.h"
#include <cstdint>

#define EC_TIMEOUTMON 500
char IOmap[4096];
// OSAL_THREAD_HANDLE thread1;
pthread_t thread1;
static int32_t latency_target_value = 0;
static int latency_target_fd = -1;
#define stack64k (64 * 1024)
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;
int running = 1;
typedef struct
{
   uint16 control;
   uint32 velocity;
} TxPdo_t;
typedef struct
{
   uint16 status;
   int32 cposition;
} RxPdo_t;

TxPdo_t *tpdo;
RxPdo_t *rpdo;

// 从站进入op状态
void slavetop(int i)
{
	int loop = 0;
	do{
		ec_slave[i].state = EC_STATE_OPERATIONAL;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET);
		ec_writestate(i);
		ec_readstate();
		loop++;
		if(loop == 10000)
		{
			if(ec_slave[i].ALstatuscode != 0)
				printf("ALstate code:%x\n",ec_slave[i].ALstatuscode);
			loop = 0;
		}
	}while(ec_slave[i].state != EC_STATE_OPERATIONAL);

}
#define CHECKERROR(slaveId)   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}

void set_slave_op(int i)
{
   int loop = 0;
   do
   {
      ec_slave[i].state = EC_STATE_OPERATIONAL;
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_writestate(i);
      ec_readstate();
      loop++;
      if (loop == 10000)
      {
         if (ec_slave[i].ALstatuscode != 0)
            printf("ALstate code:%x\n", ec_slave[i].ALstatuscode);
         loop = 0;
      }
   } while (ec_slave[i].state != EC_STATE_OPERATIONAL);
}

static void set_latency_target(void) // 消除时钟偏移
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

void *ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
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

int SetupPDO(uint16 slave){
    int retval = 0;
    uint8 buf8;
    uint16 buf16;
    uint32 buf32;
    // SDO configure
    buf32 = 0;
    retval += ec_SDOwrite(slave, 0x2002, 1, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //use CiA402
    buf32 = 3;
    retval += ec_SDOwrite(slave, 0x6060, 0, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Speed
    buf32 = (1 << 17) * 100;
    retval += ec_SDOwrite(slave, 0x6083, 0, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Acc
    retval += ec_SDOwrite(slave, 0x6084, 0, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Dec

    // PDO configure
    buf16 = 0;
    retval += ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(buf16), &buf16, EC_TIMEOUTRXM); //Disable 1c12
    retval += ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(buf16), &buf16, EC_TIMEOUTRXM); //Disable 1c13
    // Send pdo configure
    buf8 = 0;
    retval += ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(buf8), &buf8, EC_TIMEOUTRXM); //Disable tpdo
    buf32 = 0x60400010; // 16bit
    retval += ec_SDOwrite(slave, 0x1600, 1, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Controlword
    buf32 = 0x60FF0020;
    retval += ec_SDOwrite(slave, 0x1600, 2, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Velocity
    buf8 = 2;
    retval += ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(buf8), &buf8, EC_TIMEOUTRXM); //2 items

    // Recv pdo configure
    buf8 = 0;
    retval += ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(buf8), &buf8, EC_TIMEOUTRXM); //Disable rpdo
    buf32 = 0x60410010;
    retval += ec_SDOwrite(slave, 0x1A00, 1, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Statusword
    buf32 = 0x60640020;
    retval += ec_SDOwrite(slave, 0x1A00, 2, FALSE, sizeof(buf32), &buf32, EC_TIMEOUTRXM); //Current position
    buf8 = 2;
    retval += ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(buf8), &buf8, EC_TIMEOUTRXM); //2 items

    buf16 = 0x1600;
    retval += ec_SDOwrite(slave, 0x1C12, 1, FALSE, sizeof(buf16), &buf16, EC_TIMEOUTRXM); //Pack 1600 to 1c12
    buf16 = 0x1A00;
    retval += ec_SDOwrite(slave, 0x1C13, 1, FALSE, sizeof(buf16), &buf16, EC_TIMEOUTRXM); //Pack 0x1A00 to 1c13

    buf16 = 1;
    retval += ec_SDOwrite(slave, 0x1C12, 0, FALSE, sizeof(buf16), &buf16, EC_TIMEOUTRXM); //Enable 1c12
    buf8 = 1;
    retval += ec_SDOwrite(slave, 0x1C13, 0, FALSE, sizeof(buf8), &buf8, EC_TIMEOUTRXM); //Enable 1c13
    return retval;
}

void motor_run(int *firstflag, int *activebit4)
{
   ec_receive_processdata(EC_TIMEOUTRET);
   if (*firstflag)
   {
      *firstflag = 0;
      tpdo->control = 0x00;
      tpdo->control = 0x40;
   }
   else if (((rpdo->status) & 0x4f) == 0x40)
   {
      tpdo->control = 0x06;
   }
   else if (((rpdo->status) & 0x6f) == 0x21)
   {
      tpdo->control = 0x07;
   }
   // else if (((rpdo->status) & 0x6f) == 0x23)
   // {
   //    int32 adv = 8388608; // 加减速
   //    tpdo->control = 0x0f;
   //    tpdo->mode = 1;
   //    tpdo->tposition = rpdo->cposition;
   //    ec_SDOwrite(1, 0x6083, 0x00, FALSE, 4, &adv, EC_TIMEOUTRXM);
   //    ec_SDOwrite(1, 0x6084, 0x00, FALSE, 4, &adv, EC_TIMEOUTRXM);
   //    tpdo->velocity = 838860;
   //    // tpdo->tposition = 214748364;
   //    tpdo->tposition = 0;
   // }
   else if (((rpdo->status) & 0x6f) == 0x27)
   {
      *activebit4++;
      if (*activebit4 < 1000)
      {
         if (((tpdo->control) & 0x10) == 0x10)
         {
            tpdo->control &= ~0x0010;
         }
         else
         {
            tpdo->control |= 0x0010;
         }
      }
   }
   else
   {
      tpdo->control = 0x40;
   }

   ec_send_processdata();
   usleep(2000); // 周期大小
}
void endsignal(int sig)
{
   running = 0;
   printf("EtherCAT stop.\n");
   signal(SIGINT, SIG_DFL);
}

void simpletest(char *ifname)
{
   int i, j, oloop, iloop, chk;
   int firstflag = 1;
   int64 dctime = 0;
   int activebit4 = 0;
   needlf = FALSE;
   inOP = FALSE;
   printf("Starting simple test\n");
   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);

      /* find and auto-config slaves */
      // set all slave to safe op
      if (ec_config_init(FALSE) > 0)
      {
         // pdo config
         ec_slavet *slave;
         slave = &ec_slave[1];
         slave->PO2SOconfig = SetupPDO; 
         ec_config_map(&IOmap);
         ec_configdc();  
         ec_dcsync0(1, TRUE, 2000000, 5000);
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0))
            oloop = 1;
         if (oloop > 8)
            oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0))
            iloop = 1;
         if (iloop > 8)
            iloop = 8;
         printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         /* request OP state for all slaves */
         slavetop(0);

         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            /* cyclic loop */
            // ec_send_processdata();
            // motor run
            int step = 0;
            uint16 buf16=128;
            int __s = sizeof(buf16);
            int i=0;
            while (running)
            {
               ec_receive_processdata(EC_TIMEOUTRET);
               tpdo = (TxPdo_t *)ec_slave[1].outputs;
               rpdo = (RxPdo_t *)ec_slave[1].inputs;
               printf("rpdo status : %x \n",rpdo->status);
               uint16_t motor[4] = {0x0040, 0x0006, 0x0007, 0x000f};
               tpdo->velocity = static_cast<uint32_t>((1<<17)*10);
               tpdo->control = motor[i];
               i++;
               printf("tpdo control : %x velocity: %x\n",tpdo->control, tpdo->velocity);
               ec_send_processdata();
               usleep(2000); // 周期大小

               // if (wkc >= expectedWKC)
               // {
                  printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

                  for (j = 0; j < oloop; j++)
                  {
                     printf(" %2.2x", *(ec_slave[0].outputs + j));
                  }

                  printf(" I:");
                  for (j = 0; j < iloop; j++)
                  {
                     printf(" %2.2x", *(ec_slave[0].inputs + j));
                  }
                  printf(" T:%" PRId64 "\r", ec_DCtime);
                  needlf = TRUE;
               // }
               // osal_usleep(5000);
            }
            inOP = FALSE;
         }
         else
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
         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End simple test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExecute as root\n", ifname);
   }
}

int main(int argc, char *argv[])
{
   int iret1;
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
   // signal(SIGINT, endsignal);

   if (argc > 1)
   {
      set_latency_target(); // 消除时钟偏移
      // osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      // iret1 = pthread_create(&thread1, NULL, &ecatcheck, (void(*)) & ctime);
      // osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);
      signal(SIGINT, endsignal);
      simpletest(argv[1]);
   }
   else
   {
      ec_adaptert *adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf("\nAvailable adapters:\n");
      adapter = ec_find_adapters();

      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }

   printf("End program\n");
   return (0);
}