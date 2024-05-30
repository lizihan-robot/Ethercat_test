#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

char IOmap[4096];
typedef struct{
	uint16 control;
	uint8 mode;
	int32 tposition;
	int32 velocity;
}TxPdo_t;
typedef struct{
	uint16 status;
	int32 cposition;
}RxPdo_t;

TxPdo_t *tpdo;
RxPdo_t *rpdo;

int pdo_config(uint16 i);
int run = 1;
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
void endsignal()
{
	run = 0;
	printf("EtherCAT stop.\n");
	signal( SIGINT, SIG_DFL );
}
void simpletest(char *ifname)
{
	int app = 0;
	int firstflag = 1;
	int64 dctime = 0;
	int activebit4 = 0;
	if(ec_init(ifname))
	{
		printf("start ethernet at %s\n",ifname);
		if ( ec_config_init(FALSE) > 0 )
		{
			ec_slavet *slave;
			printf("found %d slave on the bus\n",ec_slavecount);
			ec_slave[0].state = ec_dcsync0;
			ec_writestate(0);
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			slave = &ec_slave[1];
			slave->PO2SOconfig = pdo_config;
			ec_config_map(&IOmap);
			ec_configdc();
			ec_dcsync0(1,TRUE,2000000,5000);
			slavetop(0);
			if(ec_slave[0].state == EC_STATE_OPERATIONAL)
			{
				printf("all slave to op\n");
				tpdo = (TxPdo_t *)ec_slave[1].outputs;
				rpdo = (RxPdo_t *)ec_slave[1].inputs;
				while(run)
				{
					app++;
					ec_receive_processdata(EC_TIMEOUTRET);
					if(firstflag)
					{
						firstflag = 0;	
						tpdo->control = 0x00;
						tpdo->control = 0x40;
					}
					else if(((rpdo->status)&0x4f) == 0x40)
					{
						tpdo->control = 0x06;
						tpdo->mode = 1;
						tpdo->tposition = rpdo->cposition;
					}
					else if(((rpdo->status)&0x6f) == 0x21)
					{
						tpdo->control = 0x07;
						tpdo->mode = 1;
						tpdo->tposition = rpdo->cposition;
					}
					else if(((rpdo->status)&0x6f) == 0x23)
					{
						int32 adv = 8388608;//加减速
						tpdo->control = 0x0f;
						tpdo->mode = 1;
						tpdo->tposition = rpdo->cposition;
						ec_SDOwrite(1,0x6083,0x00,FALSE,4,&adv,EC_TIMEOUTRXM);
						ec_SDOwrite(1,0x6084,0x00,FALSE,4,&adv,EC_TIMEOUTRXM);
						tpdo->velocity = 838860;
						//tpdo->tposition = 214748364;
						tpdo->tposition = 0;
					}
					else if(((rpdo->status)&0x6f) == 0x27)
					{
						activebit4++;
						if(activebit4 < 1000)
						{
							if(((tpdo->control) & 0x10) == 0x10)
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
						tpdo->mode = 0;

					}

					ec_send_processdata();
					usleep(2000);// 周期大小
				}
				printf("cyclic task end\n");
				do{
					ec_slave[1].state = EC_STATE_SAFE_OP;
					ec_writestate(1);
				}while(ec_slave[1].state == EC_STATE_OPERATIONAL);
				do{
					ec_slave[1].state = EC_STATE_PRE_OP;
					ec_writestate(1);
				}while(ec_slave[1].state == EC_STATE_SAFE_OP);
				do{
					ec_slave[1].state = EC_STATE_INIT;
					ec_writestate(1);
				}while(ec_slave[1].state == EC_STATE_PRE_OP);
				ec_close();
			}
			else
			{
				printf("slave again to op\n");
			}
		}
		else
		{
			printf("no slave on the bus\n");
		}
	}
	else
	{
		printf("no ethernet card\n");
	}
}
int main(int argc, char *argv[])
{
	printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
	signal( SIGINT , endsignal );

	if (argc > 1)
	{      
		simpletest(argv[1]);
	}
	else
	{
		printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
	}   

	printf("End program\n");
	return (0);
}
//PDO配置
int pdo_config(uint16 i)
{
	uint16 value16 = 0;
	uint32 value32 = 0;

	value16 = 0;
	ec_SDOwrite(i,0x1C12,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);
	ec_SDOwrite(i,0x1C13,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);

	value16 = 0;
	ec_SDOwrite(i,0x1A00,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);
	value32 = 0x60410010;
	ec_SDOwrite(i,0x1A00,1,FALSE,sizeof(value32),&value32,EC_TIMEOUTRXM);
	value32 = 0x60640020;
	ec_SDOwrite(i,0x1A00,2,FALSE,sizeof(value32),&value32,EC_TIMEOUTRXM);
	value16 = 2;
	ec_SDOwrite(i,0x1A00,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);

	value16 = 0;
	ec_SDOwrite(i,0x1600,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);
	value32 = 0x60400010;
	ec_SDOwrite(i,0x1600,1,FALSE,sizeof(value32),&value32,EC_TIMEOUTRXM);
	value32 = 0x60600008;
	ec_SDOwrite(i,0x1600,2,FALSE,sizeof(value32),&value32,EC_TIMEOUTRXM);
	value32 = 0x607A0020;
	ec_SDOwrite(i,0x1600,3,FALSE,sizeof(value32),&value32,EC_TIMEOUTRXM);
	//value32 = 0x60FF0020;
	value32 = 0x60810020;
	ec_SDOwrite(i,0x1600,4,FALSE,sizeof(value32),&value32,EC_TIMEOUTRXM);
	value16 = 4;
	ec_SDOwrite(i,0x1600,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);

	value16 = 1;
	ec_SDOwrite(i,0x1C12,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);
	ec_SDOwrite(i,0x1C13,0,FALSE,sizeof(value16),&value16,EC_TIMEOUTRXM);
	return 0;

}
