
#include <soemservo.h> //#include <mutex>#include<math.h>

#pragma comment(lib, "ws2_32.lib")

// #include <Mmsystem.h>//#pragma comment(lib, "Winmm.lib")
// using namespace std;

#define EC_TIMEOUTMON 500
char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
volatile int rtcnt;
boolean inOP;
uint8 currentgroup = 0;

int state[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int stateCnt[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool bClosAllThread = false;
int moveType[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int moveStep[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int moveAbsStep[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double acc1Time[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double velTime[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double dec1Time[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double startPos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double endacc1Pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double startdec1Pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double startdec1Vel[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int dir[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double acc1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double dec1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double vel[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double maxVm[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double movePos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double lengAbs[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double lenAccDec[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double lenAcc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double disStartDec[8] = {0, 0, 0, 0, 0, 0, 0, 0};
Drive_Inputs *iptr1;
Drive_Outputs *optr1;
int checkCnt = 0;
int checkCnt1 = 0;
Drive_Inputs driveIn[8];
Drive_Outputs driveOut[8];
/* most basic RT thread for process data, just does IO transfer */ void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{ // IOmap[0]++;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    rtcnt++; /* do RT control stuff here */
    for (int i = 0; i < ec_slavecount; i++)
    {
        servoResetEnable(i);
        moveJog(i);
        moveAbs(i);
        if (inOP)
        {
            iptr1 = (Drive_Inputs *)ec_slave[i + 1].inputs;
            optr1 = (Drive_Outputs *)ec_slave[i + 1].outputs;
            driveIn[i] = *(Drive_Inputs *)iptr1;
            driveOut[i] = *(Drive_Outputs *)optr1; // printf("slave=%d;cw=%d;sw=%d;actpos=%d;actvel=%d;actTor=%d;tarPos=%d;checkCnt=%d;checkCnt1=%d;\n",i,optr1->Controlword,iptr1->Statusword,*((int32_t*)(ec_slave[i + 1].inputs+10)),iptr1->ActualVel,iptr1->ActualTor,optr1->TargetPos,checkCnt,checkCnt1);        }
        }
    }
    static int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
    {
        int wkc1;
        wkc1 = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        return wkc1;
    }
    static int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
    {
        int wkc1;
        wkc1 = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        return wkc1;
    }
    static int drive_write32(uint16 slave, uint16 index, uint8 subindex, int32 value)
    {
        int wkc1;
        wkc1 = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        return wkc1;
    }
    // 该函数用于设置PDO映射表int drive_setup(uint16 slave){   int wkc1 = 0;
    printf("Drive setup\n");
    wkc1 += drive_write16(slave, 0x1C12, 0, 0);
    wkc1 += drive_write16(slave, 0x1C13, 0, 0);
    wkc1 += drive_write16(slave, 0x1A00, 0, 0);
    wkc1 += drive_write32(slave, 0x1A00, 1, 0x60410010); // Statusword   wkc1 += drive_write32(slave, 0x1A00, 2, 0x60770010); // Torque actual value   wkc1 += drive_write32(slave, 0x1A00, 3, 0x60640020); // Position actual value   wkc1 += drive_write32(slave, 0x1A00, 4, 0x606C0020); // Velocity actual value   wkc1 += drive_write32(slave, 0x1A00, 5, 0x60610008); // Modes of operation display   wkc1 += drive_write32(slave, 0x1A00, 6, 0x00000008); // 2nd Pos   wkc1 += drive_write8(slave, 0x1A00, 0, 6);
    wkc1 += drive_write8(slave, 0x1600, 0, 0);
    wkc1 += drive_write32(slave, 0x1600, 1, 0x60400010); // Controlword   wkc1 += drive_write32(slave, 0x1600, 2, 0x60710010); // Target torque   wkc1 += drive_write32(slave, 0x1600, 3, 0x607A0020); // Target position   wkc1 += drive_write32(slave, 0x1600, 4, 0x60FF0020); // Target velocity   wkc1 += drive_write32(slave, 0x1600, 5, 0x60600008); // Modes of operation display   wkc1 += drive_write32(slave, 0x1600, 6, 0x00000008); // Torque offset   wkc1 += drive_write8(slave, 0x1600, 0, 6);
    wkc1 += drive_write16(slave, 0x1C12, 1, 0x1600);
    wkc1 += drive_write8(slave, 0x1C12, 0, 1);
    wkc1 += drive_write16(slave, 0x1C13, 1, 0x1A00);
    wkc1 += drive_write8(slave, 0x1C13, 0, 1);
    strncpy(ec_slave[slave].name, "Drive", EC_MAXNAME);
    if (wkc1 != 22)
    {
        printf("Drive %d setup failed\nwkc: %d\n", slave, wkc1);
        return -1;
    }
    else
        printf("Drive %d setup succeed.\n", slave);
    return 0;
}
int EL7031setup(uint16 slave)
{
    int retval;
    uint16 u16val;
    // map velocity    uint16 map_1c12[4] = {0x0003, 0x1601, 0x1602, 0x1604};    uint16 map_1c13[3] = {0x0002, 0x1a01, 0x1a03};
    retval = 0;
    // Set PDO mapping using Complete acc1ess    // Strange, writing CA works, reading CA doesn't    // This is a protocol error of the slave.    retval += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);    retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    // bug in EL7031 old firmware, Completeacc1ess for reading is not supported even if the slave says it is.    ec_slave[slave].CoEdetails &= ~ECT_COEDET_SDOCA;
    // set some motor parameters, just as example    u16val = 1200; // max motor current in mA//    retval += ec_SDOwrite(slave, 0x8010, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);    u16val = 150; // motor coil resistance in 0.01ohm//    retval += ec_SDOwrite(slave, 0x8010, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);
    // set other nescessary parameters as needed    // .....
    while (EcatError)
        printf("%s", ec_elist2string());
    printf("EL7031 slave %d set, retval = %d\n", slave, retval);
    return 1;
}
int AEPsetup(uint16 slave)
{
    int retval;
    uint8 u8val;
    uint16 u16val;
    retval = 0;
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1600;
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1a00;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u8val = 8;
    retval += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    // set some motor parameters, just as example    u16val = 1200; // max motor current in mA//    retval += ec_SDOwrite(slave, 0x8010, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);    u16val = 150; // motor coil resistance in 0.01ohm//    retval += ec_SDOwrite(slave, 0x8010, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTSAFE);
    // set other nescessary parameters as needed    // .....
    while (EcatError)
        printf("%s", ec_elist2string());
    printf("AEP slave %d set, retval = %d\n", slave, retval);
    return 1;
}
void simpletest(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk, slc;
    UINT mmResult;
    needlf = FALSE;
    inOP = FALSE;
    // Drive_Inputs *iptr;    Drive_Outputs *optr;
    printf("Starting simple test\n");
    /* initialise SOEM, bind socket to ifname */ if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname); /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);
            if ((ec_slavecount > 100))
            {
                for (slc = 1; slc <= ec_slavecount; slc++)
                { // beckhoff EL7031, using ec_slave[].name is not very reliable                 if((ec_slave[slc].eep_man == 0x00000002) && (ec_slave[slc].eep_id == 0x1b773052))                 {                     printf("Found %s at position %d\n", ec_slave[slc].name, slc);                     // link slave specific setup to preop->safeop hook                     ec_slave[slc].PO2SOconfig = &EL7031setup;                 }                 // Copley Controls EAP, using ec_slave[].name is not very reliable                 if((ec_slave[slc].eep_man == 0x000000ab) && (ec_slave[slc].eep_id == 0x00000380))                 {                     printf("Found %s at position %d\n", ec_slave[slc].name, slc);                     // link slave specific setup to preop->safeop hook                     ec_slave[slc].PO2SOconfig = &AEPsetup;                 }             }         }
                    if ((ec_slavecount >= 1))
                    {
                        for (slc = 1; slc <= ec_slavecount; slc++)
                        {
                            printf("Found %s at position %d\n", ec_slave[slc].name, slc); // link slave specific setup to preop->safeop hook                 //ec_slave[slc].PO2SOconfig =&drive_setup;                 ec_slave[slc].PO2SOconfig =&AEPsetup ;             }         }

                            ec_config_map(&IOmap);
                            ec_configdc();                                           // ec_dcsync0(1, TRUE, 4000000U, 20000U);
                            printf("Slaves mapped, state to SAFE_OP.\n");            /* wait for all slaves to reach SAFE_OP state */
                            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4); // ec_statecheck(1, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
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
                            ec_slave[0].state = EC_STATE_OPERATIONAL; // ec_slave[1].state = EC_STATE_OPERATIONAL;         /* send one valid process data to make outputs in slaves happy*/         ec_send_processdata();         wkc=ec_receive_processdata(EC_TIMEOUTRET);
                            /* start RT thread as periodic MM timer */ mmResult = timeSetEvent(1, 0, RTthread, 0, TIME_PERIODIC);
                            /* 对PDO进行初始化 */ for (i = 0; i < ec_slavecount; i++)
                            {
                                optr = (Drive_Outputs *)ec_slave[i + 1].outputs;
                                if (optr == NULL)
                                {
                                    printf("optr is NULL.\n");
                                }
                                optr->Controlword = 0;
                                optr->TargetPos = 0;
                                optr->ModeOp = 0;
                                optr->TargetTor = 0;
                                optr->TargetVel = 0; // optr->TorOff = 0;             //*((int32_t*)(ec_slave[i + 1].outputs+10))=0;         }
                                /* request OP state for all slaves */ ec_writestate(0);
                                chk = 200; /* wait for all slaves to reach OP state */
                                do
                                {
                                    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // ec_statecheck(1, EC_STATE_OPERATIONAL, 50000);            //wkc = ec_receive_processdata(EC_TIMEOUTRET);            //ec_send_processdata();         }         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));         if ((ec_slave[0].state == EC_STATE_OPERATIONAL || ec_slave[0].state == EC_STATE_OPERATIONAL)&&(wkc == expectedWKC) )         {            printf("Operational state reached for all slaves.\n");            wkc_count = 0;            inOP = TRUE;            printf("inOP = TRUE;\n");
                                    while (true)
                                    {
                                        if (bClosAllThread == true)
                                            break;
                                        osal_usleep(100000); //                iptr = (Drive_Inputs *)ec_slave[0 + 1].inputs;//                optr = (Drive_Outputs *)ec_slave[0 + 1].outputs;//                printf("cw=%d;sw=%d;actpos=%d;actvel=%d;tarPos=%d;\n",optr->Controlword,iptr->Statusword,iptr->ActualPos,iptr->ActualVel,optr->TargetPos);                if(bClosAllThread==true)                    break;            }

                                        /* cyclic loop, reads data from RT thread */ while (bClosAllThread == false && false) // for(i = 1; i <= 500; i++)            {                if(wkc >= expectedWKC)                {                    printf("Processdata cycle %4d, WKC %d , O:", rtcnt, wkc);
                                            for (j = 0; j < oloop; j++)
                                            {
                                                printf(" %2.2x", *(ec_slave[0].outputs + j));
                                            }
                                        printf(" I:");
                                        for (j = 0; j < iloop; j++)
                                        {
                                            printf(" %2.2x", *(ec_slave[0].inputs + j));
                                        }
                                        printf(" T:%lld\r", ec_DCtime);
                                        needlf = TRUE;
                                    }
                                    osal_usleep(15000);
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
                                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                                    }
                                }
                            }
                            /* stop RT thread */ timeKillEvent(mmResult);
                            printf("\nRequest init state for all slaves\n");
                            ec_slave[0].state = EC_STATE_INIT; /* request INIT state for all slaves */
                            ec_writestate(0);
                        }
                        else
                        {
                            printf("No slaves found!\n");
                        }
                        printf("End simple test, close socket\n"); /* stop SOEM, close socket */
                        ec_close();
                    }
                    else
                    {
                        printf("No socket connection on %s\nExcecute as root\n", ifname);
                    }
                }
                // DWORD WINAPI ecatcheck( LPVOID lpParam )OSAL_THREAD_FUNC ecatcheck(void *lpParam){    int slave;
                while (1)
                {
                    checkCnt1++;
                    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
                    {
                        checkCnt++;
                        if (needlf)
                        {
                            needlf = FALSE;
                            printf("\n");
                        } /* one ore more slaves are not responding */
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
                                else if (ec_slave[slave].state > 0)
                                {
                                    if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                                    {
                                        ec_slave[slave].islost = FALSE;
                                        printf("MESSAGE : slave %d reconfigured\n", slave);
                                    }
                                }
                                else if (!ec_slave[slave].islost)
                                { /* re-check state */
                                    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                                    if (!ec_slave[slave].state)
                                    {
                                        ec_slave[slave].islost = TRUE;
                                        printf("ERROR : slave %d lost\n", slave);
                                    }
                                }
                            }
                            if (ec_slave[slave].islost)
                            {
                                if (!ec_slave[slave].state)
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
                    osal_usleep(20000); // Sleep(10);        if(bClosAllThread==true)            break;    }
                    // return 0;}
                    char ifbuf[1024];
                    int csp_main(char *ifname)
                    {
                        ec_adaptert *adapter = NULL;
                        printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
                        int argc = 3; // char str[]={"\\Device\\NPF_{9F98CCFF-2634-4FF0-BD7E-0E2BC81473BB}"};   //char str[]={"\\Device\\NPF_{9361F0F7-476A-4E2A-811B-FCAB8A4D0613}"};
                        if (argc > 1)
                        { /* create thread to handle slave error handling in OP */
                            osal_thread_create(&thread1, 128000, &ecatcheck, (void *)&ctime);
                            strcpy(ifbuf, ifname); /* start cyclic part */
                            simpletest(ifbuf);
                        }
                        else
                        {
                            printf("Usage: simple_test ifname1\n"); /* Print the list */
                            printf("Available adapters\n");
                            adapter = ec_find_adapters();
                            while (adapter != NULL)
                            {
                                printf("Description : %s, Device to use for wpcap: %s\n", adapter->desc, adapter->name);
                                adapter = adapter->next;
                            }
                        }
                        printf("End program\n");
                        return (0);
                    }
                    void servoResetEnable(int i)
                    {
                        Drive_Inputs *iptr;
                        Drive_Outputs *optr;
                        switch (state[i])
                        {
                        case STATE_RESET: /* 对驱动器清除故障 */
                            if (i < ec_slavecount)
                            { // printf("%d slave: STATE_RESET.\n",i);            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;            optr->Controlword = 128;        }        state[i] = 0;        stateCnt[i]=0;        break;    case STATE_INIT /* 初始化驱动器 */:        if(i < ec_slavecount)        {            //printf("%d slave: STATE_INIT.\n",i);            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;            iptr = (Drive_Inputs *)ec_slave[i + 1].inputs;            optr->Controlword = 0;            optr->TargetPos = iptr->ActualPos;            //*((int32_t*)(ec_slave[i + 1].outputs+10))=*((int32_t*)(ec_slave[i + 1].inputs+10));            printf("%d slave: iptr->ActualPos=%d.\n",i,iptr->ActualPos);        }        stateCnt[i]++;        if(stateCnt[i]=500)        {            state[i] = STATE_PREREADY;            stateCnt[i]=0;        }        break;    case STATE_PREREADY:        if(i < ec_slavecount)        {            //printf("%d slave: STATE_PREREADY.\n",i);            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;            optr->Controlword = 6;        }        stateCnt[i]++;        if(stateCnt[i]=500)        {            state[i] = STATE_READY;            stateCnt[i]=0;        }        break;    case STATE_READY /* 系统转为准备使能状态 */:        if(i < ec_slavecount)        {            //printf("%d slave: TASTE_READY.\n",i);            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;            optr->Controlword = 7;            optr->ModeOp = 8;       }        stateCnt[i]++;        if(stateCnt[i]>=500)        {            state[i] = STATE_ENABLE;            stateCnt[i]=0;        }        break;    case STATE_ENABLE /* 驱动器使能 */:        if(i < ec_slavecount)        {            //printf("%d slave: STATE_ENABLE.\n",i);            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;            optr->Controlword = 15;       }        //state[i] =200;        break;    case STATE_DISABLE:        /* 使电机失能并在10个循环之后退出循环 */        if(i < ec_slavecount)        {            //printf("%d slave: STATE_DISABLE.\n",i);            optr = (Drive_Outputs *)ec_slave[i + 1].outputs;            optr->ModeOp = 0;            optr->TargetVel = 0;            optr->Controlword = 6;        }        break;    default:        break;    }}void set_bClosAllThread(bool bVal){    bClosAllThread=bVal;}
                                void get_driveInOut(int slave, Drive_Inputs *iptr, Drive_Outputs *optr)
                                { // int i=slave;    *optr = driveOut[slave];    *iptr = driveIn[slave];    //uint16_t cw=(uint16_t)(ec_slave[i + 1].outputs+0);    //uint16_t sw=(uint16_t)(ec_slave[i + 1].inputs+0);    //uint8 cw1=ec_slave[i + 1].outputs[0];    //printf("slave[%d].controlWord=%d ;statusWord=%d.\n",slave,optr->Controlword,iptr->Statusword);    //printf("slave[%d].cw=%d ;sw=%d; cw1=%d.\n",slave,cw,sw,cw1);}Drive_Inputs get_driveInput(int slave){
                                    return driveIn[slave];
                                }
                                Drive_Outputs get_driveOutput(int slave)
                                {
                                    return driveOut[slave];
                                }
                                void set_state(int slave, int stateVal) { state[slave] = stateVal; }
                                void moveJog(int slave)
                                {
                                    Drive_Inputs *iptr;
                                    Drive_Outputs *optr;
                                    double curVel = 0;
                                    double dis = 0;
                                    double curPos = 0;
                                    switch (moveStep[slave])
                                    {
                                    case 0:
                                        if (moveType[slave] == 1)
                                        {
                                            acc1Time[slave] = 0;
                                            velTime[slave] = 0;
                                            dec1Time[slave] = 0;
                                            iptr = (Drive_Inputs *)ec_slave[slave + 1].inputs;
                                            optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                            startPos[slave] = iptr->ActualPos;
                                            startPos[slave] = optr->TargetPos;
                                            endacc1Pos[slave] = 0;
                                            startdec1Pos[slave] = 0;
                                            startdec1Vel[slave] = 0;
                                            if (vel[slave] >= 0)
                                                dir[slave] = 1;
                                            else
                                                dir[slave] = -1;
                                            moveStep[slave] = 1;
                                        }
                                        break;
                                    case 1:
                                        acc1Time[slave] += 0.001;
                                        curVel = acc1[slave] * acc1Time[slave];
                                        if (curVel >= fabs(vel[slave]))
                                        {
                                            moveStep[slave] = 2;
                                            curVel = fabs(vel[slave]);
                                        }
                                        dis = acc1[slave] * acc1Time[slave] * acc1Time[slave] / 2;
                                        curPos = startPos[slave] + dis * dir[slave];
                                        optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                        optr->TargetPos = (int32_t)curPos;
                                        if (moveStep[slave] == 2)
                                        {
                                            endacc1Pos[slave] = curPos;
                                        }
                                        if (moveType[slave] != 1)
                                        {
                                            startdec1Pos[slave] = curPos;
                                            startdec1Vel[slave] = curVel;
                                            moveStep[slave] = 3;
                                        }
                                        break;
                                    case 2:
                                        velTime[slave] += 0.001;
                                        dis = fabs(vel[slave]) * velTime[slave];
                                        curPos = endacc1Pos[slave] + dis * dir[slave];
                                        optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                        optr->TargetPos = (int32_t)curPos;
                                        if (moveType[slave] != 1)
                                        {
                                            startdec1Pos[slave] = curPos;
                                            startdec1Vel[slave] = fabs(vel[slave]);
                                            moveStep[slave] = 3;
                                        }
                                        break;
                                    case 3:
                                        dec1Time[slave] += 0.001;
                                        curVel = startdec1Vel[slave] - dec1[slave] * dec1Time[slave];
                                        if (curVel < 4000)
                                        {
                                            if (curVel < 0)
                                                curVel = 0;
                                            moveStep[slave] = 0;
                                        }
                                        dis = startdec1Vel[slave] * dec1Time[slave] - dec1[slave] * dec1Time[slave] * dec1Time[slave] / 2;
                                        curPos = startdec1Pos[slave] + dis * dir[slave];
                                        optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                        optr->TargetPos = (int32_t)curPos;
                                    default:
                                        break;
                                    } //    if(moveStep[slave]>0)//    {//        printf("Jog:%d:acc1Time[slave]=%f;velTime[slave]=%f;dec1Time[slave]=%f;curPos=%f;curVel=%f;\n",moveStep[slave],acc1Time[slave],velTime[slave],dec1Time[slave],curPos,curVel);//    }}
                                    void moveAbs(int slave)
                                    {
                                        Drive_Inputs *iptr;
                                        Drive_Outputs *optr;
                                        double curVel = 0;
                                        double dis = 0;
                                        double curPos = 0;
                                        double leng = 0;
                                        double lenDec = 0;
                                        double s = 0;
                                        double dis2 = 0;
                                        switch (moveAbsStep[slave])
                                        {
                                        case 0:
                                            if (moveType[slave] == 2)
                                            {
                                                acc1Time[slave] = 0;
                                                velTime[slave] = 0;
                                                dec1Time[slave] = 0; // iptr = (Drive_Inputs *)ec_slave[slave + 1].inputs;            optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;            //startPos[slave]=iptr->ActualPos;            startPos[slave]=optr->TargetPos;            endacc1Pos[slave]=0;            startdec1Pos[slave]=0;            startdec1Vel[slave]=0;            leng=(movePos[slave]-startPos[slave]);            lengAbs[slave]=fabs(leng);            lenAcc[slave]=vel[slave]*vel[slave]/(2*acc1[slave]);            lenDec=vel[slave]*vel[slave]/(2*dec1[slave]);            lenAccDec[slave]=lenAcc[slave]+lenDec;            if(lenAccDec[slave]>lengAbs[slave])            {                s=dec1[slave]*lengAbs[slave]/(acc1[slave]+dec1[slave]); //vt^2=2*acc*s vt^2=2*dec*(lengAbs-s)=2*dec*lengAbs-2*dec*s   (acc+dec)*2s=2*dec*lengAbs                maxVm[slave]=sqrt(2*acc1[slave]*s);            }            else            {                maxVm[slave]=vel[slave];                disStartDec[slave]=lengAbs[slave]-lenDec;            }            if(leng>=0)                dir[slave]=1;            else                dir[slave]=-1;            moveAbsStep[slave]=1;        }        break;    case 1:        acc1Time[slave]+=0.001;        curVel=acc1[slave]*acc1Time[slave];        if(curVel>=vel[slave]&& lenAccDec[slave]<lengAbs[slave])        {            moveAbsStep[slave]=2;            curVel=vel[slave];        }        if(curVel>=maxVm[slave]&& lenAccDec[slave]>=lengAbs[slave])        {            moveAbsStep[slave]=3;            curVel=maxVm[slave];        }        dis=acc1[slave]*acc1Time[slave]*acc1Time[slave]/2;        curPos=startPos[slave]+dis*dir[slave];        optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;        optr->TargetPos=(int32_t)curPos;        if(moveAbsStep[slave]==2)        {            endacc1Pos[slave]=curPos;        }        if(moveAbsStep[slave]==3)        {            startdec1Pos[slave]=curPos;            startdec1Vel[slave]=curVel;
                                            }
                                            if (moveType[slave] != 2)
                                            {
                                                startdec1Pos[slave] = curPos;
                                                startdec1Vel[slave] = curVel;
                                                moveAbsStep[slave] = 5;
                                            }
                                            break;
                                        case 2:
                                            velTime[slave] += 0.001;
                                            dis = vel[slave] * velTime[slave];
                                            curPos = endacc1Pos[slave] + dis * dir[slave];
                                            dis2 = lenAcc[slave] + dis;
                                            optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                            optr->TargetPos = (int32_t)curPos;
                                            if (dis2 >= disStartDec[slave])
                                            {
                                                startdec1Pos[slave] = curPos;
                                                startdec1Vel[slave] = vel[slave];
                                                moveAbsStep[slave] = 3;
                                            }
                                            if (moveType[slave] != 2)
                                            {
                                                startdec1Pos[slave] = curPos;
                                                startdec1Vel[slave] = vel[slave];
                                                moveAbsStep[slave] = 5;
                                            }
                                            break;
                                        case 3:
                                            dec1Time[slave] += 0.001;
                                            curVel = startdec1Vel[slave] - dec1[slave] * dec1Time[slave];
                                            if (curVel <= 0)
                                            {
                                                if (curVel < 0)
                                                    curVel = 0;
                                                moveAbsStep[slave] = 4;
                                            }
                                            dis = startdec1Vel[slave] * dec1Time[slave] - dec1[slave] * dec1Time[slave] * dec1Time[slave] / 2;
                                            curPos = startdec1Pos[slave] + dis * dir[slave];
                                            if (fabs(curPos - startPos[slave]) >= lengAbs[slave])
                                            {
                                                curPos = movePos[slave];
                                                moveType[slave] = 0;
                                                moveAbsStep[slave] = 0;
                                            }
                                            optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                            optr->TargetPos = (int32_t)curPos;
                                            break;
                                        case 4: // curVel=4000;        iptr = (Drive_Inputs *)ec_slave[slave + 1].inputs;        curPos=iptr->ActualPos+10;        if(fabs(curPos-startPos[slave])>=lengAbs[slave])        {            curPos=movePos[slave];            moveType[slave]=0;            moveAbsStep[slave]=0;        }        optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;        optr->TargetPos=(int32_t)curPos;        break;    case 5:        dec1Time[slave]+=0.001;        curVel=startdec1Vel[slave]-dec1[slave]*dec1Time[slave];        if(curVel<4000)        {            if(curVel<0)                curVel=0;            moveType[slave]=0;            moveAbsStep[slave]=0;        }        dis=startdec1Vel[slave]*dec1Time[slave]-dec1[slave]*dec1Time[slave]*dec1Time[slave]/2;        curPos=startdec1Pos[slave]+dis*dir[slave];        optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;        optr->TargetPos=(int32_t)curPos;    default:        break;    }//    if(moveAbsStep[slave]>0)//    {//        printf("Abs:%d:acc1Time[slave]=%f;velTime[slave]=%f;dec1Time[slave]=%f;curPos=%f;curVel=%f;\n",moveAbsStep[slave],acc1Time[slave],velTime[slave],dec1Time[slave],curPos,curVel);//    }}
                                            int servoMoveJog(int slave, double vel1, double acc11, double dec11, int dir)
                                            {
                                                if (slave < ec_slavecount && moveType[slave] == 0 && moveStep[slave] == 0)
                                                {
                                                    if (dir == 1)
                                                    {
                                                        vel[slave] = vel1;
                                                        printf("servoMoveJog+ axis=%d;\n", slave);
                                                    }
                                                    else
                                                    {
                                                        vel[slave] = -vel1;
                                                        printf("servoMoveJog- axis=%d;\n", slave);
                                                    }
                                                    acc1[slave] = acc11;
                                                    dec1[slave] = dec11;
                                                    moveType[slave] = 1;
                                                    return 1;
                                                }
                                                else
                                                {
                                                    return -ec_slavecount;
                                                }
                                            }
                                            int servoMoveAbs(int slave, double pos, double vel1, double acc11, double dec11)
                                            {
                                                printf("servoMoveAbs axis=%d;pos=%f;\n", slave, pos);
                                                if (slave < ec_slavecount && moveType[slave] == 0 && moveStep[slave] == 0)
                                                {
                                                    movePos[slave] = pos;
                                                    vel[slave] = vel1;
                                                    acc1[slave] = acc11;
                                                    dec1[slave] = dec11;
                                                    moveType[slave] = 2;
                                                    return 1;
                                                }
                                                else
                                                {
                                                    return -ec_slavecount;
                                                }
                                            }
                                            int servoMoveRel(int slave, double dis, double vel1, double acc11, double dec11)
                                            {
                                                printf("servoMoveRel asix=%d;dis=%f;\n", slave, dis);
                                                if (slave < ec_slavecount && moveType[slave] == 0 && moveStep[slave] == 0)
                                                {
                                                    Drive_Outputs *optr;
                                                    optr = (Drive_Outputs *)ec_slave[slave + 1].outputs;
                                                    movePos[slave] = optr->TargetPos + dis;
                                                    vel[slave] = vel1;
                                                    acc1[slave] = acc11;
                                                    dec1[slave] = dec11;
                                                    moveType[slave] = 2;
                                                    return 1;
                                                }
                                                else
                                                {
                                                    return -ec_slavecount;
                                                }
                                            }
                                            int servoMoveStop(int slave)
                                            {
                                                printf("servoMoveStop axis=%d;\n", slave);
                                                if (slave < ec_slavecount)
                                                {
                                                    moveType[slave] = 0;
                                                    return 1;
                                                }
                                                else
                                                {
                                                    return -ec_slavecount;
                                                }
                                            }