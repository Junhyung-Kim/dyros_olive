#include "red_ec_master.h"
#include "sensoray826.h"
#include "controller.h"

#include <Eigen/Dense>
#include <stdio.h>
#include "omd/opto.h"
#include <unistd.h>

#define DOF 2
//15720448
#define RAD2CNT  2501987.527342825
#define CNT2RAD  1/RAD2CNT
#define CNT2EXTRAD 0.000011984225
#define EXTRAD2CNT 1/CNT2EXTRAD
#define CNT2NM  0.144 //1.44*100/1000 : motor_torque*gear_ratio/1000
#define NM2CNT  1/CNT2NM
#define Fts

using namespace EtherCAT_Elmo;
using namespace Eigen;

EthercatElmoBridge elmo;
ArmController _controller_;
//WalkingCtrl _WalkingCtrl;
sensoray826_dev _sensoray826_dev;

ElmoGoldDevice::elmo_gold_rx *rxPDO[DOF];
ElmoGoldDevice::elmo_gold_tx *txPDO[DOF];

VectorXd start_positionElmo(DOF);

VectorXd positionElmo(DOF);
VectorXd positionExtElmo(DOF);
VectorXd velocityElmo(DOF);
VectorXd torqueElmo(DOF);
VectorXd torqueDemandElmo(DOF);
const int extpositionmod[DOF] = {105489, 340054};
VectorXd positionmod(DOF);

double F_zzz;
double F_zzz_low;
bool first_torque_sensor = false;
bool first_torque_sensor_bias = false;

VectorXd positionDesiredElmo(DOF);
VectorXd velocityDesiredElmo(DOF);
VectorXd torqueDesiredElmo(DOF);

pthread_t thread1, thread2, thread3, thread4,thread5;

double torque_sensor_[3];
double torque_sensor_1[3];
double torque_sensor_prev[3];
Eigen::Vector3d torque_sensor_bias;
double jhtoff =0;
double F_zzz_prev;
int jhcnt2=0;

char *ifname;
volatile int wkc;

static struct termios initial_settings, new_settings;
static int peek_character = -1;

double _tChirp = 0.0;
double amplitudeChirp = 3.0;
double freqChirp = 0.0;
double torqueChirp = 0.0;

char key_ch;
bool key_stop = true;

//FTSensor
Response r;
unsigned int i;
SOCKET_HANDLE socketHandle;		/* Handle to UDP socket used to communicate with Ethernet DAQ. */


int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

void ethercatTask(void *ptr)
{
    char IOmap[4096];

    int wkc_count;
    boolean needlf = FALSE;
    boolean inOP = FALSE;
    int jhcnt=0;
    double jhqinit=0.0;
    double jhkp=400;  //14400    // low torque control PD 400,40
    double jhkv=40;   //240
    double jhqd=0;
    double jhqgoal=2.858;
    int jhmode=2; //1 : position mode, else : torque mode
    //int16_t jhtd=0;
    double jhtd = 0;

    double traj_qd=0;


    struct timespec current, begin, time;
    double elapsed = 0.0, elapsed_sum = 0.0, elapsed_avg = 0.0, elapsed_var = 0.0, prev = 0.0, now = 0.0, current_time = 0.0, begin_time = 0.0;
    double elapsed_time[10000] = {0.0};
    static int elapsed_cnt = 0, max_cnt=0, min_cnt=0;
    double elapsed_min = 210000000.0, elapsed_max= 0.0;
    double time_mem[10000]={0.0};

    bool reachedInitial[DOF] = {false};

    //ft?
  /*  for (i = 0; i < SAMPLE_COUNT; ++i) {
      if (i == SAMPLE_COUNT / 2) {
        SendCommand(&socketHandle, COMMAND_BIAS, BIASING_ON);
      }
      r = Receive(&socketHandle);
    //  ShowResponse(r);
    }
*/

    /*ofstream fout_positionElmo("/home/dyros/testbed_jjs/data/positionElmo.txt");
    ofstream fout_velocityElmo("/home/dyros/testbed_jjs/data/velocityElmo.txt");
    ofstream fout_velocityDesiredElmo("/home/dyros/testbed_jjs/data/velocityDesiredElmo.txt");
    ofstream fout_torqueElmo("/home/dyros/testbed_jjs/data/torqueElmo.txt");
    ofstream fout_torqueDesiredElmo("/home/dyros/testbed_jjs/data/torqueDesiredElmo.txt");
    ofstream fout_torqueSensor("/home/dyros/testbed_jjs/data/torqueSensor.txt", std::ofstream::out);*/
    ofstream taskdata("/home/dyros/testbed_jjs/data/taskdata.txt");
    ofstream jointdata("/home/dyros/testbed_jjs/data/jointdata.txt");
    ofstream torquedata("/home/dyros/testbed_jjs/data/torque.txt");
    ofstream momentum("/home/dyros/testbed_jjs/data/momentum.txt");
    ofstream vargain("/home/dyros/testbed_jjs/data/vargain.txt");

    struct sched_param schedp;
    memset( &schedp, 0, sizeof(schedp) );
    schedp.sched_priority = 49;
    if(sched_setscheduler(0, SCHED_FIFO, &schedp) == -1) {
        exit(EXIT_FAILURE);
    }

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);

        /* find and auto-config slaves */
        /* network discovery */
        if ( ec_config_init(FALSE) > 0 ) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("%d slaves found and configured.\n",ec_slavecount); // ec_slavecount -> slave num

            /** CompleteAccess disabled for Elmo driver */
            for(int slave=1; slave<=ec_slavecount; slave++)
            {
                printf("Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

            for(int slave=1; slave<=ec_slavecount; slave++)
            {
                uint16 map_1c12[2] = {0x0001, 0x1605};
                uint16 map_1c13[5] = {0x0004, 0x1a04, 0x1a11, 0x1a12, 0x1a1e};
                int os;
                os=sizeof(map_1c12);
                ec_SDOwrite(slave,0x1c12,0,TRUE,os,map_1c12,EC_TIMEOUTRXM);
                os=sizeof(map_1c13);
                ec_SDOwrite(slave,0x1c13,0, TRUE, os,map_1c13,EC_TIMEOUTRXM);
            }

            /** if CA disable => automapping works */
            ec_config_map(&IOmap);


            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            printf("Request operational state for all slaves\n");
            int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);

            int wait_cnt = 40;

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
            }
            while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                inOP = TRUE;

                /* cyclic loop */
                for(int slave=1; slave<=ec_slavecount; slave++)
                {
                    txPDO[slave-1] = (ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                    rxPDO[slave-1] = (ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                }

                struct timespec ts;
                int64 cycletime;

                cycletime = *(int*)ptr * 1000; /* cycletime in ns */
                _controller_.hz_ = 1000000000/cycletime;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                clock_gettime(CLOCK_MONOTONIC, &begin);
                prev = begin.tv_sec; prev += begin.tv_nsec/1000000000.0;

                while(!kbhit())
                //while(key_stop)
                {
                    /* wait to cycle start */
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

                    //KOJS 20190308 -----
                    _tChirp = now - prev;
                    freqChirp = _tChirp * 2.5;
                    torqueChirp = amplitudeChirp * sin(2.0*PI*_tChirp*freqChirp);
                    //KOJS 20190308 -----

                    /** PDO I/O refresh */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(250);

                    if(wkc >= expectedWKC)
                    {
                        for(int slave=1; slave<=ec_slavecount; slave++)
                        {
                            if(elmo.controlWordGenerate(rxPDO[slave-1]->statusWord, txPDO[slave-1]->controlWord))
                            {
                                reachedInitial[slave-1] = true;
                            }
                        }

                            for(int slave=1; slave<=ec_slavecount; slave++)
                            {
                                if(reachedInitial[slave-1])
                                {
                                  ///////////////ext position
                                    positionExtElmo(slave-1) = (((int32_t)ec_slave[slave].inputs[20]) +
                                        ((int32_t)ec_slave[slave].inputs[21] << 8) +
                                        ((int32_t)ec_slave[slave].inputs[22] << 16) +
                                        ((int32_t)ec_slave[slave].inputs[23] << 24) - extpositionmod[slave- 1]) * CNT2EXTRAD;
                                    _controller_.qext_(slave-1) = positionExtElmo(slave-1);

                                  ///////////////position
                                    positionElmo(slave-1) = (rxPDO[slave-1]->positionActualValue)*CNT2RAD;
                                    if(_controller_.initialize_joint == false){
                                      positionmod(slave-1) = positionElmo(slave-1) - positionExtElmo(slave-1);
                                      //cout << positionmod(0) <<" " << positionmod(1) << endl;
                                      if(slave == ec_slavecount)
                                      {
                                          _controller_.initialize_joint = true;
                                      }
                                    }
                                    _controller_.q_(slave-1) = positionElmo(slave-1) - positionmod(slave -1);

                                  ///////////////velocity
                                    velocityElmo(slave-1) =
                                            (((int32_t)ec_slave[slave].inputs[14]) +
                                            ((int32_t)ec_slave[slave].inputs[15] << 8) +
                                            ((int32_t)ec_slave[slave].inputs[16] << 16) +
                                            ((int32_t)ec_slave[slave].inputs[17] << 24)) * CNT2RAD;
                                    _controller_.qdot_(slave-1) = velocityElmo(slave-1);

                                  ////////////////torque
                                    torqueDemandElmo(slave-1) =
                                            (int16_t)((ec_slave[slave].inputs[18]) +
                                            (ec_slave[slave].inputs[19] << 8))*CNT2NM;
                                    torqueElmo(slave-1) = rxPDO[slave-1]->torqueActualValue*CNT2NM;
                                    _controller_.current_(slave-1) = torqueElmo(slave-1);

                                    double a = jhcnt;
                                    _controller_.play_time_ = a/_controller_.hz_;

                                    /////////////TORQUE SENSOR////////////
                                    _sensoray826_dev.analogOversample();

                                    for(int i=0; i<3; i++)
                                     {
                                        torque_sensor_[i] = _sensoray826_dev.adcVoltages[i]*2*-10;
                                     }

                                    if(first_torque_sensor_bias==false)
                                    {
                                      for(int i=0; i<3; i++)
                                      {
                                          torque_sensor_bias(i) = torque_sensor_[i];
                                      }
                                      first_torque_sensor_bias = true;
                                    }

                                    for(int i=0; i<3; i++)
                                    {
                                        torque_sensor_[i] =torque_sensor_[i] -torque_sensor_bias(i);
                                        _controller_.torque_(i) = torque_sensor_[i];
                                    }

                                    /////////////Controller////////////

                                    if(slave == ec_slavecount)
                                    {
                                       if(jhcnt ==0){
                                         for(int i =0 ; i<ec_slavecount; i++)
                                         {
                                             _controller_.q_init_desired_(i) = positionElmo(i);
                                         }
                                       }
                                        _controller_.compute();
                                        Eigen::VectorXd qd, td;
                                        qd.resize(ec_slavecount);
                                        td.resize(ec_slavecount);

                                        if(jhcnt == 0)
                                        {
                                          for(int i = 0; i<ec_slavecount; i++)
                                          {
                                             qd(i) = _controller_.q_init_desired_(i);
                                             td(i) = _controller_.torque_desired_(i);
                                          }
                                        }
                                        else
                                        {
                                          for(int i = 0; i<ec_slavecount; i++)
                                          {
                                             qd(i) = _controller_.q_desired_(i);
                                             td(i) = _controller_.torque_desired_(i);
                                          }
                                        }

                                        for(int i = 0; i<ec_slavecount; i++)
                                        {
                                           /////////////DAQ////////////
                                          if(jhmode==1)
                                          {
                                            txPDO[i]->modeOfOperation  = CyclicSynchronousPositionmode;
                                            txPDO[i]->targetPosition = (int32_t)(qd(i)*RAD2CNT);
                                          }
                                          else
                                          {
                                            txPDO[i]->modeOfOperation  = CyclicSynchronousTorquemode;
                                            txPDO[i]->targetTorque    = (int16_t)td(i)*NM2CNT;
                                          }
                                          /////////////MAXTORUQELIMIT////////////
                                          txPDO[i]->maxTorque       = (uint16)1000;
                                        }

                                        //errorcheck << _controller_.play_time_ << "\t" << _controller_.errorcheck2D(0) << "\t" << _controller_.errorcheck2D(1) << endl;
                                        jointdata << _controller_.play_time_
                                                  << "\t" << _controller_.q2D_desired_(0) << "\t" << _controller_.q2D_desired_(1) << "\t" << _controller_.q2D_(0) << "\t" << _controller_.q2D_(1) << "\t" << _controller_.q2Dext_(0) << "\t" << _controller_.q2Dext_(1)
                                                  << "\t" << _controller_.q2Ddot_desired_(0) << "\t" << _controller_.q2Ddot_desired_(1) << "\t" << _controller_.q2Ddot_f(0) << "\t" << _controller_.q2Ddot_f(1) << "\t" << _controller_.q2Ddotext_(0) << "\t" << _controller_.q2Ddotext_(1)
                                                  << "\t" << _controller_.q2Dddot_desired_(0) << "\t" << _controller_.q2Dddot_desired_(1) << "\t" << _controller_.q2Dddot_(0) << "\t" << _controller_.q2Dddot_(1) << "\t" << _controller_.q2Dddotext_(0) << "\t" << _controller_.q2Dddotext_(1)
                                                  << endl;
                                        momentum << _controller_.momentum_(0) << "\t" << _controller_.momentum_(1)
                                                 << "\t" << _controller_.momentum_est_(0) << "\t" << _controller_.momentum_est_(1)
                                                 << "\t" << _controller_.torque2D_dist_est(0) << "\t" << _controller_.torque2D_dist_est(1)
                                                 << "\t" << _controller_.torque2D_dist_est_mod(0) << "\t" << _controller_.torque2D_dist_est_mod(1)
                                                 << "\t" << _controller_.force_ext_est_(0) << "\t" << _controller_.force_ext_est_(1)
                                                 //<< "\t" << _controller_.tor_dist
                                                 << endl;
                                        torquedata << _controller_.torque_(0) << "\t" << _controller_.current_(0) << "\t" << _controller_.torque_desired_(0)
                                                   << "\t" << _controller_.torque2D_fric_est(0)
                                                   << endl;
                                        taskdata << _controller_.play_time_
                                                 << "\t" << _controller_.x_desired_(0) << "\t" << _controller_.x_desired_(1) << "\t" << _controller_.x_(0) << "\t" << _controller_.x_(1)
                                                 << "\t" << _controller_.x_dot_desired_(0) << "\t" << _controller_.x_dot_desired_(1) << "\t" << _controller_.x_dot_(0) << "\t" << _controller_.x_dot_(1)
                                                 << "\t" << _controller_.fstar_h(0) << "\t" << _controller_.fstar_h(1) << "\t" << _controller_.fstar_hf(0) << "\t" << _controller_.fstar_hf(1)
                                                 << "\t" << _controller_.fstar_l(0) << "\t" << _controller_.fstar_l(1) << "\t" << _controller_.fstar_lf(0) << "\t" << _controller_.fstar_lf(1) 
                                                 << endl;
                                        // vargain << _controller_.kp_mod(0) << "\t" << _controller_.kp_mod(1)
                                        //         << "\t" << _controller_.kv_mod(0) << "\t" << _controller_.kv_mod(1)
                                        //         << endl;

                                        jhcnt++;
                                    }
                                }
                            }

                        needlf = TRUE;
                    }
                    clock_gettime(CLOCK_MONOTONIC, &time);
                    now = time.tv_sec; now += time.tv_nsec/1000000000.0;
                    elapsed_time[elapsed_cnt] = now - prev;
                    prev = now;

                    elapsed_sum += elapsed_time[elapsed_cnt];
                    if(elapsed_min>elapsed_time[elapsed_cnt]) elapsed_min = elapsed_time[elapsed_cnt];
                    if(elapsed_max<elapsed_time[elapsed_cnt]) elapsed_max = elapsed_time[elapsed_cnt];

                    time_mem[elapsed_cnt] = (elapsed_time[elapsed_cnt] - (cycletime/1000000000.0)) * 1000;

                    if(++elapsed_cnt >= 100)
                    {
                        elapsed_avg = elapsed_sum/elapsed_cnt;
                        for(int i=0; i<elapsed_cnt; i++)
                        {
                            elapsed_var += (elapsed_time[i]-elapsed_avg)*(elapsed_time[i]-elapsed_avg);
                            if(elapsed_time[i]>elapsed_avg+0.00010) max_cnt++;
                            if(elapsed_time[i]<elapsed_avg-0.00010) min_cnt++;
                        }

                        elapsed_var = elapsed_var/elapsed_cnt;

                        max_cnt = 0;
                        min_cnt = 0;
                        elapsed_sum = 0;
                        elapsed_var = 0;
                        elapsed_cnt = 0;
                        elapsed_min=210000000.0;
                        elapsed_max=0.0;
                    }

                   add_timespec(&ts, cycletime);
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int slave = 1; slave<=ec_slavecount ; slave++)
                {
                    if(ec_slave[slave-1].state != EC_STATE_OPERATIONAL)
                    {
                        printf("EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               slave-1, ec_slave[slave-1].state, ec_slave[slave-1].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave-1].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            /** request INIT state for all slaves
             *  slave number = 0 -> write to all slaves
             */
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

void computeTorque(void *ptr)
{
  while(1)
  {

  }
}
/*
int computeFtsensor(void *ptr)
{

  struct sched_param schedp;
  memset( &schedp, 0, sizeof(schedp) );
  schedp.sched_priority = 49;
  if(sched_setscheduler(0, SCHED_FIFO, &schedp) == -1) {
      exit(EXIT_FAILURE);
  }

  for (i = 0; i < SAMPLE_COUNT; ++i) {
    if (i == SAMPLE_COUNT / 2) {
      SendCommand(&socketHandle, COMMAND_BIAS, BIASING_ON);
    }
    r = Receive(&socketHandle);
    ShowResponse(r);
  }

  while(1)
  {
    SendCommand(&socketHandle, COMMAND_START, 1);
    r = Receive(&socketHandle);
//    ShowResponse(r);
  }

  //F_zzz = r.fz;

}
*/
double lowPassFilter(double input, double prev, double ts, double tau)
{
    return (tau*prev + ts*input)/(tau+ts);
}

int main(int argc, char *argv[])
{

    ///// Torque sensor init /////
    _sensoray826_dev.open();
    _sensoray826_dev.analogSingleSamplePrepare(slotAttrs, 16);

    printf("Sensoray Board DAQ Setting Complete\n");

    ///// FT SENSOR /////
/*
    if (Connect(&socketHandle, "192.168.1.1", PORT) != 0) {
        fprintf(stderr, "Could not connect to device...");
        return -1;
    }

    SendCommand(&socketHandle, COMMAND_SPEED, SPEED);
    SendCommand(&socketHandle, COMMAND_FILTER, FILTER);
    SendCommand(&socketHandle, COMMAND_BIAS, BIASING_OFF);
    SendCommand(&socketHandle, COMMAND_START, SAMPLE_COUNT);

    printf("Ftsensor clear\n");*/
    printf("SOEM (Simple Open EtherCAT Master)\nRed EtherCat Master\n");

    if (argc > 2)
    {
        ifname = argv[1];
        int ctime = atoi(argv[2]);
        int ctime1 = 1000;

        /* create thread to handle slave error handling in OP */
        osal_thread_create( &thread1, NULL, (void *) &EthercatElmoBridge::ethercatCheck, NULL);

        /* create RT thread*/
     //   osal_thread_create_rt( &thread3, NULL, (void *) &computeTorque, (void *) &ctime);

        osal_thread_create_rt( &thread2, NULL, (void *) &ethercatTask, (void *) &ctime);

     //   osal_thread_create_rt( &thread4, NULL, (void *) &computeFtsensor, (void *) &ctime);

        /* start cyclic part */
    //    pthread_join( thread3, NULL);
        pthread_join( thread2, NULL);
    //    pthread_join( thread4, NULL);
    }
    else
    {
        printf("Usage: dyros_red_ethercat_master ifname1 \nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
}

