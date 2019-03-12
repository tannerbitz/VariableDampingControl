
/*
PositionControlApp
*/
//#include <chrono>
#include <sys/time.h>
#include <iostream>
#include <cmath>
//#include "engine.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h> // strstr
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;

#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"  //ip address set for KONI connection
#define SHM_SIZE 1024


FILE *NewDataFile(void);


int *MakeFloatSharedMemory(int HowBig);
float *MakeFloatSharedMemory2(int HowBig2);

int main (int argc, char** argv)
{
    //--------------------------FORWARD KINEMATICS-----------------------------
    //DH Parameters
    double elapsed=0;
    double vel_depend=0;
    double velocity_2=0;
    double w2=0;
    double velocity_old=0;
    double velocity_new=0;
    double velocity_mean=0;
    double error_delta=0;
    double checkval=0;
    double mag=0.02;
    double dur=250;
    int trig_stat=0;
    int outerdelay=1;
    int delay_gate=0;
    int met=0;
    double ftx;
    double fty;
    double ftx_un;
    double fty_un;
    double zerox=0;
    double zeroy=0;
    double x_disp;
    double y_disp;
    double ftx_0=0.0;
    double fty_0=0.0;
    int *data; //pointer to shared memory
    data=MakeFloatSharedMemory(2);
    float *data2; //pointer to shared memory
    data2=MakeFloatSharedMemory2(2);
    int c=0;
    int cc=0;
    double al=0.0005;
    double al2=0.1;
    int firstIt=0;
    int trigger = 1;
    double fty_freeze;
    double T06_freeze_x;
    double T06_freeze_y;
    double T06_freeze_z;
    double phi_euler_freeze;
    double theta_euler_freeze;
    double psi_euler_freeze;
    int w=0;
    int random_num;
    int ww =0;
    double ft[2];
    int steady=0;
    double FK_x=0;
    double FK_x0=0;
    double gate1=0.355;
    double gate2=-0.1;
    int count3=0;
    double delta_t =0.001;
    double stupid = 1000.0;
    int perturb_flag=0;
    int rr=1;
    int count_ran1=0;
    int count_ran2=0;
    int count_ran3=0;
    int count_ran4=0;
    int count_ran_null=0;
    int exit_ran=0;
    int perturb_count=0;
    /*
    double T01=0;
    double T02=0;
    double T03=0;
    double T04=0;
    double T05=0;
    double T06=0;*/
    double phi_euler=0;
    double theta_euler=0;
    double psi_euler=0;
    /*MatrixXd alpha(1,7); alpha << -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, 0;
    MatrixXd a(1,7); a << 0, 0, 0, 0, 0, 0, 0;
    MatrixXd d(1,7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
    MatrixXd theta(1,7); theta << 0, 0, 0, 0, 0, 0, 0;	*/

    MatrixXd alpha(1,7); alpha << M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, 0;
    MatrixXd a(1,7); a << 0, 0, 0, 0, 0, 0, 0;
    MatrixXd d(1,7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
    MatrixXd theta(1,7); theta << 0, 0, 0, 0, 0, 0, 0;

    MatrixXd x_0(6,1); x_0 << 0, 0, 0, 0, 0, 0;
    MatrixXd x_00(6,1); x_00 << 0, 0, 0, 0, 0, 0;
    MatrixXd x_incr(6,1); x_incr << 0, 0, 0, 0, 0, 0;
    MatrixXd x_incr_0(6,1); x_incr_0 << 0, 0, 0, 0, 0, 0;
    MatrixXd test_new(6,1); test_new << 0, 0, 0, 0, 0, 0;


    MatrixXd qc(6,1); qc << 0, 0, 0, 0, 0, 0;
    MatrixXd delta_q(6,1); delta_q << 0, 0, 0, 0, 0, 0;
    MatrixXd q_freeze(6,1); q_freeze << 0, 0, 0, 0, 0, 0;

    MatrixXd q_0(6,1); q_0 << 0, 0, 0, 0, 0, 0;
    MatrixXd t_0(6,1); t_0 << 0, 0, 0, 0, 0, 0;
    MatrixXd x_e(6,1); x_e << 0, 0, 0, 0, 0, 0;
    MatrixXd force(6,1); force << 0, 0, 0, 0, 0, 0;
    MatrixXd q_new(6,1); q_new << 0,0,0,0,0,0;
    MatrixXd q_delay(6,1); q_delay << 0,0,0,0,0,0;
    MatrixXd x_new(6,1); x_new << 0,0,0,0,0,0;
    MatrixXd q_old(6,1); q_old << 0,0,0,0,0,0;
    MatrixXd test0(6,6); test0 << 1,0,0,0,0,0,
                                0,1,0,0,0,0,
                                0,0,1,0,0,0,
                                0,0,0,2,0,0,
                                0,0,0,0,2,0,
                                0,0,0,0,0,2;

    //filter
    int sample=200;
    int sample2=100;
    MatrixXd tdata(6,sample); tdata.setZero(6,sample);
    //MatrixXd qdata(6,sample); qdata.setRow(0)=1;//this needs to be itialize as the starting joint angles



    MatrixXd qdata(6,sample); qdata.setZero(6,sample);//this needs to be itialize as the starting joint angles

    for(int i=0;i<sample;i++){
        qdata(0,i)=-1.779862; //UPRIGHT
        qdata(1,i)=0.821814;
        qdata(2,i)=-0.067855;
        qdata(3,i)=1.302481;
        qdata(4,i)=0.284275;
        qdata(5,i)=-1.118251;
    }





    //MatrixXd Zeros(6,1));
    MatrixXd t_sum(6,1); t_sum.setZero(6,1);
    MatrixXd q_sum(6,1); q_sum.setZero(6,1);
    MatrixXd t_ave(6,1); t_ave << 0, 0, 0, 0, 0, 0;
    MatrixXd q_ave(6,1); q_ave << 0, 0, 0, 0, 0, 0;
    MatrixXd torques_0(6,1); torques_0 << 0, 0, 0, 0, 0, 0;
    MatrixXd t_e(6,1); t_e << 0, 0, 0, 0, 0, 0;
    // MatrixXd test(2,2); test << 1, 2, 3, 4;

    // std::cout << test << std::endl;
    //-------------------------------------------------------------------------


    //timestamp
    struct timespec start2, finish2;
    struct timeval start, end;
    long mtime, seconds, useconds;
    gettimeofday (&start, NULL);




    /*
    float *data; //pointer to shared memory
    data=MakeFloatSharedMemory(7); //creates and initializes to 0 a shared memory float array*/
    //Engine *ep = engOpen(NULL);
    //engEvalString(ep,"Display");
    //engClose(ep);

    //
    // parse command line arguments
    if (argc < 2){
        printf(
            "\nKUKA LBR Position Control application\n\n"
            "\tCommand line arguments:\n"
            "\t1) filename of trajectory file"
            "\t2) remote hostname (optional)\n"
            "\t3) port ID (optional)\n"
        );
        return 1;
    }

    char* filename = argv[1]; //Requiered command line argument for the file name for trajectory file
    const char* hostname = (argc >= 3) ? argv[2] : DEFAULT_IP; //optional command line argument for ip address (default is for KONI)
    int port = (argc >= 4) ? atoi(argv[3]) : DEFAULT_PORTID; //optional comand line argument for port
    char path[4096];

    FILE *InputFile;
    FILE *OutputFile;
    FILE *fp;

    bool success = true;
    bool success2 = true;

    int count=0;
    int count2=0;
    int enough=0;
    int rows=0;
    int i=0;
    int j=0;
    //int j=0;


    int stime=0;
    float sampletime=0; //will be determined from FRI
    double MJoint[7]={0};//{-1.776415,1.025433,-0.064599,1.656986,0.290444,-0.971846,-0.223775}; //last measured joint position (not sure time when controller measured, be careful using this for feedback loop)
    double ETorque[7]={0}; //External torque :supposedly the torque applied to the robot (subtracts the torque due to the robot, ackording to Kuka)

    double MaxRadPerSec[7]={1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159}; //absolute max velocity (no load from KUKA manual for iiwa 800)
    //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
    double MaxRadPerStep[7]={0};//will be calculated
    double MaxJointLimitRad[7]={2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543};//Max joint limits in radians (can be changed to further restrict motion of robot)
    double MinJointLimitRad[7]={-2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543}; //Min joint limits in radians (can be changed to further restrict motion of robot)
    double FirstPositionDelta[7]={0.0175,0.0175,0.0175,0.0175,0.0175,0.0175,0.0175}; //maximum deviation from initial position in trajectory from start position in robot(radians)


    //Get value for the time step of the command cycle (used for determining max velocity)
    // sampletime=client.GetTimeStep();
    sampletime=0.001;
    fprintf(stdout,"Sample Time:%f seconds\n",sampletime);

    //calculate max step value
    for(i=0;i<7;i++){
        MaxRadPerStep[i]=sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
    }


    // create new joint position client
    PositionControlClient client;
    client.intvalues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);




    // create new udp connection
    UdpConnection connection;


    // pass connection and client to a new FRI client application
    ClientApplication app(connection, client);

    // connect client application to KUKA Sunrise controller
    app.connect(port, hostname);




    //create file for output
    OutputFile=NewDataFile();

    //open trajectory file
    InputFile = fopen(filename,"r");

    //get the first value in the file. This should be the number of rows(or the number of joint commands)
    success2=fscanf(InputFile, "%d", &rows);
    //cout<<rows<<endl;
    if(rows<4){
        fprintf(stdout,"Number of Rows is %d\n Are you sure the file is in correct format?\n First number should be number of rows.  \nThen each row is 7 joint values in radians.\n",rows);
        return 1;
    }
    if (!success2){
        fprintf(stdout,"Error in reading file!\n");
        return 1;
    }


    //read in first 7 joint positions (these should be in radians)
    success2=fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf", &client.NextJoint[0],&client.NextJoint[1],&client.NextJoint[2],&client.NextJoint[3],&client.NextJoint[4],&client.NextJoint[5],&client.NextJoint[6]);
    memcpy(client.LastJoint, client.NextJoint,7*sizeof(double));


    //fprintf(stdout,"success2\n");

    //cout<<client.NextJoint[0]<<endl;
    if (!success2){
        fprintf(stdout,"Error in reading file!\n");
        return 1;
    }


    //check to insure that the first joint command is not significantly different from the robot's initial position
    //for(j=0;j<2;j++)
    //{

    /*    memcpy(MJoint,client.GetMeasJoint(),sizeof(double)*7 );
    memcpy(ETorque,client.GetExtTor(),sizeof(double)*7 ); //gets the external torques at the robot joints (supposedly subtracts the torques caused by the robot)
    //}
    //print out to file for condition 0
    fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],MJoint[6],ETorque[0],ETorque[1],ETorque[2],ETorque[3],ETorque[4],ETorque[5],ETorque[6],stime);*/



    // fprintf(stdout, "%lf", &client.NextJoint[0]);

    /*,&client.NextJoint[1],&client.NextJoint[2],&client.NextJoint[3],&client.NextJoint[4],&client.NextJoint[5],&client.NextJoint[6]);
    fprintf(stdout, "%lf %lf %lf %lf %lf %lf %lf %d \n", &MJoint[0],&MJoint[1],&MJoint[2],&MJoint[3],&MJoint[4],&MJoint[5],&MJoint[6]);*/
    //-1.776415 1.025433 -0.064599 1.656986 0.290444 -0.971846 -0.223775

    /* MJoint[0]=-1.776415;
    MJoint[1]=1.025433;
    MJoint[2]=-0.064599;
    MJoint[3]=1.656986;
    MJoint[4]=0.290444;
    MJoint[5]=-0.971846;
    MJoint[6]=-0.223775; */


    /*
    for(i=0;i<7;i++)
    {
    if (fabs(MJoint[i]-client.NextJoint[i])>FirstPositionDelta[i])
    {
    fprintf(stdout,"ERROR!!! Intital Position of Robot too far from first command!!!\n");
    return 1;
}
}*/

// repeatedly call the step routine to receive and process FRI packets

//ADD MJoint AND CLIENT JOINT HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!
//{-1.776415,1.025433,-0.064599,1.656986,0.290444,-0.971846,-0.223775}

//----------------Initialize forward and inverse kinematic variables------------

//----------------------------------------------------------------------------
//---------------------------------- LOOP ------------------------------------
//----------------------------------------------------------------------------

while (!enough)
{
    clock_gettime(CLOCK_MONOTONIC, &start2);
    //---------------------------timestamp----------------------------------
    gettimeofday(&end, NULL);
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    //std::cout << mtime << std::endl;
    //----------------------------------------------------------------------


    success = app.step();//step through program

    if(client.KukaState==4)
    {
        count++; //count initialized at 0
        if (count==1)//first time inside
        {
            sampletime=client.GetTimeStep();
            //calculate max step value
            for(i=0;i<7;i++)
            {
                client.MaxRadPerStep[i]=sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
            }

        }
        memcpy(MJoint,client.GetMeasJoint(),sizeof(double)*7 ); //gets the most recent measured joint value
        memcpy(ETorque,client.GetExtTor(),sizeof(double)*7 );//gets the external torques at the robot joints (supposedly subtracts the torques caused by the robot)

        //---------------------------FORWARD KINEMATICS----------------------------------
        theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];


        //std::cout << theta(0,1) << std::endl;
        /*	theta(1,1)=MJoint[0];
        theta(1,2)=MJoint[1];
        theta(1,3)=MJoint[2];
        theta(1,4)=MJoint[3];
        theta(1,5)=MJoint[4];
        theta(1,6)=MJoint[5];
        theta(1,7)=MJoint[6];*/
        MatrixXd A1(4,4); A1 << cos(theta(0,0)), -sin(theta(0,0))*cos(alpha(0,0)), sin(theta(0,0))*sin(alpha(0,0)), a(0,0)*cos(theta(0,0)),
                                sin(theta(0,0)), cos(theta(0,0))*cos(alpha(0,0)), -cos(theta(0,0))*sin(alpha(0,0)), a(0,0)*sin(theta(0,0)),
                                0, sin(alpha(0,0)), cos(alpha(0,0)), d(0,0),
                                0, 0, 0, 1;
        MatrixXd A2(4,4); A2 << cos(theta(0,1)), -sin(theta(0,1))*cos(alpha(0,1)), sin(theta(0,1))*sin(alpha(0,1)), a(0,1)*cos(theta(0,1)),
                                sin(theta(0,1)), cos(theta(0,1))*cos(alpha(0,1)), -cos(theta(0,1))*sin(alpha(0,1)), a(0,1)*sin(theta(0,1)),
                                0, sin(alpha(0,1)), cos(alpha(0,1)), d(0,1),
                                0, 0, 0, 1;
        MatrixXd A3(4,4); A3 << cos(theta(0,2)), -sin(theta(0,2))*cos(alpha(0,2)), sin(theta(0,2))*sin(alpha(0,2)), a(0,2)*cos(theta(0,2)),
                                sin(theta(0,2)), cos(theta(0,2))*cos(alpha(0,2)), -cos(theta(0,2))*sin(alpha(0,2)), a(0,2)*sin(theta(0,2)),
                                0, sin(alpha(0,2)), cos(alpha(0,2)), d(0,2),
                                0, 0, 0, 1;
        MatrixXd A4(4,4); A4 << cos(theta(0,3)), -sin(theta(0,3))*cos(alpha(0,3)), sin(theta(0,3))*sin(alpha(0,3)), a(0,3)*cos(theta(0,3)),
                                sin(theta(0,3)), cos(theta(0,3))*cos(alpha(0,3)), -cos(theta(0,3))*sin(alpha(0,3)), a(0,3)*sin(theta(0,3)),
                                0, sin(alpha(0,3)), cos(alpha(0,3)), d(0,3),
                                0, 0, 0, 1;
        MatrixXd A5(4,4); A5 << cos(theta(0,4)), -sin(theta(0,4))*cos(alpha(0,4)), sin(theta(0,4))*sin(alpha(0,4)), a(0,4)*cos(theta(0,4)),
                                sin(theta(0,4)), cos(theta(0,4))*cos(alpha(0,4)), -cos(theta(0,4))*sin(alpha(0,4)), a(0,4)*sin(theta(0,4)),
                                0, sin(alpha(0,4)), cos(alpha(0,4)), d(0,4),
                                0, 0, 0, 1;
        MatrixXd A6(4,4); A6 << cos(theta(0,5)), -sin(theta(0,5))*cos(alpha(0,5)), sin(theta(0,5))*sin(alpha(0,5)), a(0,5)*cos(theta(0,5)),
                                sin(theta(0,5)), cos(theta(0,5))*cos(alpha(0,5)), -cos(theta(0,5))*sin(alpha(0,5)), a(0,5)*sin(theta(0,5)),
                                0, sin(alpha(0,5)), cos(alpha(0,5)), d(0,5),
                                0, 0, 0, 1;
        /*MatrixXd A7(4,4); A7 << cos(theta(0,6)), -sin(theta(0,6))*cos(alpha(0,6)), sin(theta(0,6))*sin(alpha(0,6)), a(0,6)*cos(theta(0,6)),
        sin(theta(0,6)), cos(theta(0,6))*cos(alpha(0,6)), -cos(theta(0,6))*sin(alpha(0,6)), a(0,6)*sin(theta(0,6)),
        0, sin(alpha(0,6)), cos(alpha(0,6)), d(0,6),
        0, 0, 0, 1;
        */
        //MatrixXd AT(4,4); AT << A1*A2*A3*A4*A5*A6*A7;


        MatrixXd T01(4,4); T01 << A1;
        MatrixXd T02(4,4); T02 << A1*A2;
        MatrixXd T03(4,4); T03 << A1*A2*A3;
        MatrixXd T04(4,4); T04 << A1*A2*A3*A4;
        MatrixXd T05(4,4); T05 << A1*A2*A3*A4*A5;
        MatrixXd T06(4,4); T06 << A1*A2*A3*A4*A5*A6;




        FK_x=T06(2,3);



        x_disp=T06(0,3);
        data2[0]=x_disp;
        y_disp=T06(2,3);
        data2[1]=y_disp;



        //---------------------------INVERSE KINEMATICS------------------------------------

        phi_euler=atan2(T06(1,2),T06(0,2));
        theta_euler=atan2(sqrt(pow(T06(1,2),2)+pow(T06(0,2),2)),T06(2,2));
        psi_euler=atan2(T06(2,1),-T06(2,0));

        //std::cout << theta_euler << std::endl;


        MatrixXd z0(3,1); z0 << 0,0,1;
        MatrixXd z1(3,1); z1 << T01(0,2),T01(1,2),T01(2,2);
        MatrixXd z2(3,1); z2 << T02(0,2),T02(1,2),T02(2,2);
        MatrixXd z3(3,1); z3 << T03(0,2),T03(1,2),T03(2,2);
        MatrixXd z4(3,1); z4 << T04(0,2),T04(1,2),T04(2,2);
        MatrixXd z5(3,1); z5 << T05(0,2),T05(1,2),T05(2,2);
        MatrixXd z6(3,1); z6 << T06(0,2),T06(1,2),T06(2,2);

        MatrixXd p0(3,1); p0 << 0,0,0;
        MatrixXd p1(3,1); p1 << T01(0,3),T01(1,3),T01(2,3);
        MatrixXd p2(3,1); p2 << T02(0,3),T02(1,3),T02(2,3);
        MatrixXd p3(3,1); p3 << T03(0,3),T03(1,3),T03(2,3);
        MatrixXd p4(3,1); p4 << T04(0,3),T04(1,3),T04(2,3);
        MatrixXd p5(3,1); p5 << T05(0,3),T05(1,3),T05(2,3);
        MatrixXd p6(3,1); p6 << T06(0,3),T06(1,3),T06(2,3);

        //jacobian

        MatrixXd J1(6,1); J1 << z0(1,0)*(p6(2,0)-p0(2,0))-z0(2,0)*(p6(1,0)-p0(1,0)),
                                -z0(0,0)*(p6(2,0)-p0(2,0))+z0(2,0)*(p6(0,0)-p0(0,0)),
                                z0(0,0)*(p6(1,0)-p0(1,0))-z0(1,0)*(p6(0,0)-p0(0,0)),
                                z0(0,0),z0(1,0),z0(2,0);
        MatrixXd J2(6,1); J2 << z1(1,0)*(p6(2,0)-p1(2,0))-z1(2,0)*(p6(1,0)-p1(1,0)),
                                -z1(0,0)*(p6(2,0)-p1(2,0))+z1(2,0)*(p6(0,0)-p1(0,0)),
                                z1(0,0)*(p6(1,0)-p1(1,0))-z1(1,0)*(p6(0,0)-p1(0,0)),
                                z1(0,0),z1(1,0),z1(2,0);
        MatrixXd J3(6,1); J3 << z2(1,0)*(p6(2,0)-p2(2,0))-z2(2,0)*(p6(1,0)-p2(1,0)),
                                -z2(0,0)*(p6(2,0)-p2(2,0))+z2(2,0)*(p6(0,0)-p2(0,0)),
                                z2(0,0)*(p6(1,0)-p2(1,0))-z2(1,0)*(p6(0,0)-p2(0,0)),
                                z2(0,0),z2(1,0),z2(2,0);
        MatrixXd J4(6,1); J4 << z3(1,0)*(p6(2,0)-p3(2,0))-z3(2,0)*(p6(1,0)-p3(1,0)),
                                -z3(0,0)*(p6(2,0)-p3(2,0))+z3(2,0)*(p6(0,0)-p3(0,0)),
                                z3(0,0)*(p6(1,0)-p3(1,0))-z3(1,0)*(p6(0,0)-p3(0,0)),
                                z3(0,0),z3(1,0),z3(2,0);
        MatrixXd J5(6,1); J5 << z4(1,0)*(p6(2,0)-p4(2,0))-z4(2,0)*(p6(1,0)-p4(1,0)),
                                -z4(0,0)*(p6(2,0)-p4(2,0))+z4(2,0)*(p6(0,0)-p4(0,0)),
                                z4(0,0)*(p6(1,0)-p4(1,0))-z4(1,0)*(p6(0,0)-p4(0,0)),
                                z4(0,0),z4(1,0),z4(2,0);
        MatrixXd J6(6,1); J6 << z5(1,0)*(p6(2,0)-p5(2,0))-z5(2,0)*(p6(1,0)-p5(1,0)),
                                -z5(0,0)*(p6(2,0)-p5(2,0))+z5(2,0)*(p6(0,0)-p5(0,0)),
                                z5(0,0)*(p6(1,0)-p5(1,0))-z5(1,0)*(p6(0,0)-p5(0,0)),
                                z5(0,0),z5(1,0),z5(2,0);

        MatrixXd Jg(6,6); Jg << J1, J2, J3, J4, J5, J6;

        MatrixXd Tphi(6,6); Tphi << 1,0,0,0,0,0,
                                    0,1,0,0,0,0,
                                    0,0,1,0,0,0,
                                    0,0,0,0,-sin(phi_euler),cos(phi_euler)*sin(theta_euler),
                                    0,0,0,0,cos(phi_euler),sin(phi_euler)*sin(theta_euler),
                                    0,0,0,1,0,cos(theta_euler);

        MatrixXd Ja(6,6); Ja << Tphi.inverse()*Jg;
        //std::cout << Ja << std::endl;std::cout << ftx << std::endl;
        //std::cout << (Ja.transpose()).inverse() << std::endl;


        //here is where you would add your new point x due to a perturbation
        //MatrixXd x(6,1); x << x_0(0),x_0(1)+0.0001,x_0(2),x_0(3),x_0(4),x_0(5);



        //----------------------------------IMPEDANCE CONTROLLER--------------------------

        //----------------------------------Initialize variables-------------------------

        MatrixXd stiffness(6,6); stiffness << 10000,0,0,0,0,0, //toward varun desk
                                            0,10000000,0,0,0,0, //up
                                            0,0,0,0,0,0, //out toward workshop
                                            0,0,0,1000000,0,0,
                                            0,0,0,0,1000000,0,
                                            0,0,0,0,0,1000000;
        MatrixXd damping(6,6); damping << 100000,0,0,0,0,0,
                                            0,100000,0,0,0,0,
                                            0,0,500,0,0,0,
                                            0,0,0,500,0,0,
                                            0,0,0,0,500,0,
                                            0,0,0,0,0,500;
        MatrixXd inertia(6,6); inertia << 1,0,0,0,0,0,
                                            0,1,0,0,0,0,
                                            0,0,1,0,0,0,
                                            0,0,0,100,0,0,
                                            0,0,0,0,100,0,
                                            0,0,0,0,0,100;



        //MatrixXd q(6,1); q << (Ja.inverse()*(x-x_0)+q_0);
        //std::cout << q << std::endl;
        //std::cout << "----" << std::endl;





        MatrixXd torques(6,1); torques << ETorque[0], ETorque[1], ETorque[2], ETorque[3], ETorque[4], ETorque[5];
        //------------------------torque filter----------------------------------
        /* for(int i=sample;i-->1;)
        {
        tdata(0,i)=tdata(0,i-1);
        tdata(1,i)=tdata(1,i-1);
        tdata(2,i)=tdata(2,i-1);
        tdata(3,i)=tdata(3,i-1);
        tdata(4,i)=tdata(4,i-1);
        tdata(5,i)=tdata(5,i-1);

        qdata(0,i)=qdata(0,i-1);
        qdata(1,i)=qdata(1,i-1);
        qdata(2,i)=qdata(2,i-1);
        qdata(3,i)=qdata(3,i-1);
        qdata(4,i)=qdata(4,i-1);
        qdata(5,i)=qdata(5,i-1);
    }
    tdata(0,0)=ETorque[0];
    tdata(1,0)=ETorque[1];
    tdata(2,0)=ETorque[2];
    tdata(3,0)=ETorque[3];
    tdata(4,0)=ETorque[4];
    tdata(5,0)=ETorque[5];

    qdata(0,0)=MJoint[0];
    qdata(1,0)=MJoint[1];
    qdata(2,0)=MJoint[2];
    qdata(3,0)=MJoint[3];
    qdata(4,0)=MJoint[4];
    qdata(5,0)=MJoint[5];
    for(int i=0;i<6;i++)
    {
    t_sum(i,0)=0;
    q_sum(i,0)=0;
    for(int j=0;j<sample;j++)
    {
    t_sum(i,0)=tdata(i,j)+t_sum(i,0);
}
for(int j=0;j<sample2;j++)
{
q_sum(i,0)=qdata(i,j)+q_sum(i,0);
}
t_ave(i,0)=t_sum(i,0)/sample;
q_ave(i,0)=q_sum(i,0)/sample2;*/


t_e << al*torques+(1-al)*torques_0;
torques_0 << t_e;




//t_ave(i,0)=(tdata(i,0)+tdata(i,1)+tdata(i,2)+tdata(i,3)+tdata(i,4)+tdata(i,5)+tdata(i,6)+tdata(i,7)+tdata(i,8)+tdata(i,9))/10;

//}
//std::cout << q_ave << std::endl;
//std::cout << "----" << std::endl;

/*for(int i=10;i-->1;)
{

tdata(0,i)=tdata(0,i-1);
tdata(1,i)=tdata(1,i-1);
tdata(2,i)=tdata(2,i-1);
tdata(3,i)=tdata(3,i-1);
tdata(4,i)=tdata(4,i-1);
tdata(5,i)=tdata(5,i-1);
}
tdata(0,0)=ETorque[0];
tdata(1,0)=ETorque[1];
tdata(2,0)=ETorque[2];
tdata(3,0)=ETorque[3];
tdata(4,0)=ETorque[4];
tdata(5,0)=ETorque[5];
for(int i=0;i<6;i++)
{
t_ave(i,0)=(tdata(i,0)+tdata(i,1)+tdata(i,2)+tdata(i,3)+tdata(i,4)+tdata(i,5)+tdata(i,6)+tdata(i,7)+tdata(i,8)+tdata(i,9))/10;

}
std::cout << t_ave << std::endl;*/
//------------------------------------------------------------------------------
//std::cout << tdata << std::endl;



//MatrixXd q_0(6,1); q_0 << q;
//MatrixXd q_1(6,1); q_1 << q;
//MatrixXd J_0(6,6); J_0 << Ja;
//MatrixXd J_1(6,6); J_1 << Ja;




//	MatrixXd joints_new(6,1); joints_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*((Ja.transpose()).inverse()*torques+(2*inertia+damping)*Ja_1*q_1-inertia*J_0*q_0-stiffness*equilibrium_point);


if (firstIt == 0)//first time inside
{
    firstIt= 1;
    std::cout << firstIt << std::endl;
    //MatrixXd x(6,1); x << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;
    x_e << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;
    //x_0 << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;
    //x_00 << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;
    //q_0 << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5];
    //t_0 << ETorque[0], ETorque[1], ETorque[2], ETorque[3], ETorque[4], ETorque[5];
    t_0 <<-0.0313991,-0.414115,-0.00613078,-0.450773,0.265899,-0.0418674;
}


//std::cout << x_e << std::endl;
//MatrixXd q_new(6,1); q_new << Ja.inverse()*(inertia+damping-stiffness).inverse()*((Ja.transpose()).inverse()*(t_ave)+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_ave;

//MatrixXd q_new(6,1); q_new << Ja.inverse()*((inertia+damping-stiffness).inverse()*((Ja.transpose()).inverse()*(t_ave)-inertia*(x_00)+damping*(x_0)-stiffness*(x_e))-x_0)+q_ave;
//t_ave << 0,0,0,0,0,0;
//t_e << 0,0,0,0,0,0;


//force << (Ja.transpose()).inverse()*(t_e);
//force << force(0,0),force(1,0),force(2,0),0,0,0;
//force << 10*sin(M_PI*w/1000),0,0,0,0,0;


velocity_new=(x_00(2)-x_0(2))*1000.0*0.25;
velocity_mean=al2*velocity_new+(1-al2)*velocity_old;
velocity_old=velocity_new;


ftx=(double)data[0]/1000000-zerox; //toward varun desk
ftx_un=(double)data[0]/1000000-zerox;
//std::cout << ftx_un << std::endl;

fty=(double)data[1]/1000000-zeroy;
fty_un=(double)data[1]/1000000-zeroy;
//std::cout << data[0] << std::endl;

ftx=  al*ftx+(1-al)*ftx_0;
ftx_0 = ftx;
fty=  al*fty+(1-al)*fty_0;
fty_0 = fty;
//force << 3.0*sin(M_PI*ww/500),0,3.0*sin(M_PI*ww/500),0,0,0;
ww=ww+1;
force << ftx,0,fty,0,0,0;
steady=steady+1;
//std::cout << ftx << std::endl;

perturb_flag=0;

//ftx=0;
//fty=0;

//std::cout << data[0] << std::endl;
//ftx=data[0]/1000000;
//fty=data[1];
//w=0;
//std::cout << ftx << std::endl;


//---------------------------------------

//FILE *fp;


//fp = popen("ethercat upload --type=int32 0x6000 03", "r");
//   if (fp == NULL) {
//     printf("Failed to run command\n" );
//     exit(1);
//   }

q_0 << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5];
//fgets(path, sizeof(path), fp);
//getline(path, sizeof(path), fp);
//printf("%s", path);


//std::cout << 2 << std::endl;
//}

//system("ethercat upload --type=int32 0x6000 03");
//pclose(fp);
//--------------------------GATES-------------------------------------------------
//if(c==1)
//{
//count3=count3+1;
//if(count3==100) //100 ms perturbation
//{
//c=0; //disables gates
// }
//force << 0,0,0,0,0,0;
//std::cout << AT(0,3) << std::endl;

delay_gate=delay_gate+1;
//}

// 	if ((FK_x > gate1) && (FK_x0 < gate1) && (cc == 0) && (delay_gate > 3000))//first time inside
//  //if ((FK_x < gate1) && (FK_x0 > gate1) && (cc == 0) && (delay_gate > 3000))
//  {
// 	   cc=1;
// 	   exit_ran=0;
// 	   //system("amarok beep.wav");
// 	   //std::cout << cc << std::endl;
// 	   //force << 0,0,0,0,0,0;
//         }
if ((FK_x < gate1) && (FK_x0 > gate1) && (cc == 0) && (delay_gate > 2000))//first time inside
{
    cc=1;
    exit_ran=0;
    //std::cout << cc << std::endl;
    // force << 0,0,0,0,0,0;
}
// 	if ((FK_x >= gate2) && (FK_x0 < gate2) && (cc == 0) )//first time inside
//         {
// 	   cc=1;
// 	   std::cout << cc << std::endl;
// 	   	   //force << 0,0,0,0,0,0;
//         }
// 	if ((FK_x <= gate2) && (FK_x0 > gate2) && (cc == 0) )//first time inside
//         {
// 	   cc=1;
// 	   std::cout << cc << std::endl;
// 	   	  // force << 0,0,0,0,0,0;
//         }

FK_x0=FK_x;
//std::cout << FK_x << std::endl;
//---------------------------------PERTURBATION---------------------------------
//c=0;
//std::cout << M_PI << std::endl;
if(cc == 1)
{
    if(trigger == 1)
    {
        trigger = 0;
        fty_freeze = fty;
        T06_freeze_x=T06(0,3);
        T06_freeze_y=T06(1,3);
        T06_freeze_z=T06(2,3);
        phi_euler_freeze=phi_euler;
        theta_euler_freeze=theta_euler;
        psi_euler_freeze=psi_euler;
        q_freeze << q_new;
        //random_num=rand() % 6 + 1;


        while(exit_ran==0)
        {
            random_num=rand() % 4 + 1;
            //   random_num=2;
            if(random_num==1 && count_ran1<10)
            {
                perturb_flag=1;
                count_ran1=count_ran1+1;
                std::cout << "case1 = "<< std::endl;
                std::cout << count_ran1 << std::endl;
                velocity_2=velocity_mean;
                std::cout << velocity_2 << std::endl;
                exit_ran=1;
                perturb_count=perturb_count+1;
            }
            if(random_num==2 && count_ran2<10)
            {
                perturb_flag=2;
                count_ran2=count_ran2+1;
                std::cout << "case2 = "<< std::endl;
                std::cout << count_ran2 << std::endl;
                velocity_2=velocity_mean;
                exit_ran=1;
                perturb_count=perturb_count+1;

            }std::cout << velocity_2 << std::endl;
            if(random_num==3 && count_ran3<10)
            {
                perturb_flag=3;
                count_ran3=count_ran3+1;
                std::cout << "case3 = "<< std::endl;
                std::cout << count_ran3 << std::endl;
                velocity_2=velocity_mean;
                exit_ran=1;
                perturb_count=perturb_count+1;
            }
            if(random_num==4 && count_ran4<10)
            {
                perturb_flag=4;
                count_ran4=count_ran4+1;
                std::cout << "case4 = "<< std::endl;
                std::cout << count_ran4 << std::endl;
                velocity_2=velocity_mean;
                exit_ran=1;
                perturb_count=perturb_count+1;

            }





            if(random_num==5 && count_ran_null<10 || random_num==6 && count_ran_null<40 || random_num==7 && count_ran_null<40 || random_num==8  && count_ran_null<40)
            {
                perturb_flag=5;
                count_ran_null=count_ran_null+1;
                std::cout << "case null = "<< std::endl;
                std::cout << count_ran_null << std::endl;
                exit_ran=1;
                perturb_count=perturb_count+1;
            }
            std::cout << "Perturbation number = "<< std::endl;
            std::cout << perturb_count << std::endl;
        }






        //random_num=3;
        // std::cout << fty_freeze << std::endl;
    }

    // std::cout << w << std::endl;
    //std::cout << w << std::endl;
    // x_incr << T06_freeze_x+0.02*sin(w*M_PI/400-M_PI/2),T06_freeze_y,T06_freeze_z,phi_euler_freeze,theta_euler_freeze,psi_euler_freeze;
    if(random_num==1)
    {

        //std::cout << w << std::endl;
        //x_incr << 0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
        if (rr==1){
            //std::cout << "case rr=4" << std::endl;
            x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=2;
                w=1;
            }
        }

        if (rr==2){
            vel_depend=velocity_2*w/dur;
            // 	   if (abs(vel_depend) >> 0.06){
            // 	     vel_depend=0;
            // 	   }

            //ftx=  al*ftx+(1-al)*ftx_0;
            //ftx_0 = ftx;
            //x_incr << 0,0,mag*((0.5)+(0.5*sin((w*M_PI/dur)-M_PI/2))),0,0,0;
            x_incr << 0,0,-vel_depend+mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0; //
            //x_incr << 0,0,mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0; //
            //  x_incr << 0,0,-vel_depend,0,0,0; //
            //std::cout << x_incr(2) << std::endl;
            //std::cout << mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0))+ << std::endl;
            //x_incr << 0,0,mag*(w/dur),0,0,0;
            perturb_flag=7;
            // std::cout << x_incr(2) << std::endl;
            checkval=x_incr(2)-x_incr_0(2);
            //std::cout << x_incr(2) << std::endl;
            if (w==dur){
                rr=3;
                w=1;
            }
        }

        if (rr==3){
            //std::cout << "case rr=4" << std::endl;
            x_incr << 0,0,-velocity_2+mag*(10.0*pow(1,3.0)-15.0*pow(1,4.0)+6.0*pow(1,5.0))-velocity_2*w/dur,0,0,0;
            //x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=4;
                w=1;
            }
        }


        if (rr==4){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << 0,0,mag-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0;
            //x_incr << 0,0,0,0,0,0;
            //x_incr << 0,0,mag-mag*(w/dur),0,0,0;
            if (w==1){
                rr=5;
                w=1;
                //trig_stat=1;
            }

        }

        if (rr==5){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << 0,0,0,0,0,0;
            //x_incr << 0,0,mag-mag*(w/dur),0,0,0;
            if (w==1){
                //rr=4;
                //w=0;
                trig_stat=1;
            }

        }




        //std::cout << x_incr(0) << std::endl;
        //x_incr << 0,0,0,0,0,0;
        //q_new << Ja.inverse()*(x_incr)+q_freeze;//+(q_old-q_0)*0.3;
        qc << Ja.inverse()*(x_incr-x_incr_0);//+(q_old-q_0)*0.3;
        //std::cout << Ja.inverse() << std::endl;
        delta_q << delta_q+qc;

        q_new << q_freeze+delta_q;
        x_incr_0 << x_incr;
    }
    if(random_num==2)
    {
        //std::cout << w << std::endl;
        //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
        //x_incr << -0.02*(10.0*pow(w/200.0,3.0)-15.0*pow(w/200.0,4.0)+6.0*pow(w/200.0,5.0)),0,0,0,0,0;
        //std::cout << x_incr(0) << std::endl;
        //x_incr << 0,0,0,0,0,0;
        //q_new << Ja.inverse()*(x_incr)+q_freeze;//+(q_old-q_0)*0.3;


        if (rr==1){
            //std::cout << "case rr=4" << std::endl;
            x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=2;
                w=1;
            }
        }

        if (rr==2){
            //x_incr << 0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            vel_depend=velocity_2*w/dur;
            //std::cout << velocity_2 << std::endl;
            //std::cout << w/dur << std::endl;
            //std::cout << vel_depend << std::endl;
            //std::cout << "test" << std::endl;
            // x_incr << 0,0,-vel_depend-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0;
            x_incr << 0,0,-vel_depend-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0;
            //x_incr << 0,0,-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0;
            //x_incr << 0,0,-mag*((0.5)+(0.5*sin((w*M_PI/dur)-M_PI/2))),0,0,0;


            //x_incr << 0,0,-mag*(w/dur),0,0,0;
            perturb_flag=7;
            //std::cout << "case rr=1" << std::endl;
            if (w==dur){
                rr=3;
                w=1;
            }
        }

        if (rr==3){
            //std::cout << "case rr=4" << std::endl;
            x_incr << 0,0,-velocity_2-mag*(10.0*pow(1,3.0)-15.0*pow(1,4.0)+6.0*pow(1,5.0))-velocity_2*w/dur,0,0,0;
            //x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=4;
                w=1;
            }
        }

        if (rr==4){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << 0,0,-(mag-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0))),0,0,0;
            //  x_incr << 0,0,0,0,0,0;
            //x_incr << 0,0,-(mag-mag*(w/dur)),0,0,0;
            if (w==1){
                rr=5;
                w=1;
                //trig_stat=1;
            }
        }

        if (rr==5){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << 0,0,0,0,0,0;
            //x_incr << 0,0,mag-mag*(w/dur),0,0,0;
            if (w==1){
                //rr=4;
                //w=0;
                trig_stat=1;
            }

        }





        qc << Ja.inverse()*(x_incr-x_incr_0);//+(q_old-q_0)*0.3;
        delta_q << delta_q+qc;
        //std::cout << delta_q << std::endl;
        q_new << q_freeze+delta_q;
        x_incr_0 << x_incr;
    }

    if(random_num==3)
    {

        //std::cout << w << std::endl;
        //x_incr << 0,0,0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0;
        // x_incr << 0,0,0.02*(10.0*pow(w/200.0,3.0)-15.0*pow(w/200.0,4.0)+6.0*pow(w/200.0,5.0)),0,0,0;
        //std::cout << x_incr(0) << std::endl;
        //x_incr << 0,0,0,0,0,0;
        //q_new << Ja.inverse()*(x_incr)+q_freeze;//+(q_old-q_0)*0.3;


        if (rr==1){
            //std::cout << "case rr=4" << std::endl;
            x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=2;
                w=1;
            }
        }

        if (rr==2){
            vel_depend=velocity_2*w/dur;
            //x_incr << 0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,-vel_depend,0,0,0;
            //  x_incr << mag*((0.5)+(0.5*sin((w*M_PI/dur)-M_PI/2))),0,0,0,0,0;


            //x_incr << mag*(w/dur),0,0,0,0,0;
            perturb_flag=7;
            //std::cout << "case rr=1" << std::endl;
            if (w==dur){
                rr=3;
                w=1;
            }
        }

        if (rr==3){
            //std::cout << "case rr=4" << std::endl;
            x_incr << mag*(10.0*pow(1,3.0)-15.0*pow(1,4.0)+6.0*pow(1,5.0)),0,-velocity_2-velocity_2*w/dur,0,0,0;
            //  x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=4;
                w=1;
            }
        }

        if (rr==4){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << mag-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,0,0,0,0;
            //  x_incr << 0,0,0,0,0,0;
            //x_incr << mag-mag*(w/dur),0,0,0,0,0;
            if (w==1){
                rr=5;
                w=1;
                //trig_stat=1;
            }
        }

        if (rr==5){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << 0,0,0,0,0,0;
            //x_incr << 0,0,mag-mag*(w/dur),0,0,0;
            if (w==1){
                //rr=4;
                //w=0;
                trig_stat=1;
            }

        }






        qc << Ja.inverse()*(x_incr-x_incr_0);//+(q_old-q_0)*0.3;
        delta_q << delta_q+qc;
        //std::cout << delta_q << std::endl;
        q_new << q_freeze+delta_q;
        x_incr_0 << x_incr;
    }
    if(random_num==4)
    {
        //std::cout << w << std::endl;
        //x_incr << 0,0,-0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0;
        //x_incr << 0,0,-0.02*(10.0*pow(w/200.0,3.0)-15.0*pow(w/200.0,4.0)+6.0*pow(w/200.0,5.0)),0,0,0;
        //std::cout << x_incr(2) << std::endl;
        //x_incr << 0,0,0,0,0,0;
        //q_new << Ja.inverse()*(x_incr)+q_freeze;//+(q_old-q_0)*0.3;



        if (rr==1){
            //std::cout << "case rr=4" << std::endl;
            x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=2;
                w=1;
            }
        }

        if (rr==2){
            vel_depend=velocity_2*w/dur;
            //x_incr << 0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            // x_incr << -mag*((0.5)+(0.5*sin((w*M_PI/dur)-M_PI/2))),0,0,0,0,0;
            x_incr << -mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0)),0,-vel_depend,0,0,0;
            //x_incr << -mag*(w/dur),0,0,0,0,0;
            perturb_flag=7;
            //std::cout << "case rr=1" << std::endl;
            if (w==dur){
                rr=3;
                w=1;
            }
        }

        if (rr==3){
            //std::cout << "case rr=4" << std::endl;
            x_incr << -mag*(10.0*pow(1,3.0)-15.0*pow(1,4.0)+6.0*pow(1,5.0)),0,-velocity_2-velocity_2*w/dur,0,0,0;
            //      x_incr << 0,0,0,0,0,0;
            if (w==1){
                rr=4;
                w=1;
            }
        }

        if (rr==4){
            // std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << -(mag-mag*(10.0*pow(w/dur,3.0)-15.0*pow(w/dur,4.0)+6.0*pow(w/dur,5.0))),0,0,0,0,0;
            //  x_incr << 0,0,0,0,0,0;
            //x_incr << -(mag-mag*(w/dur)),0,0,0,0,0;
            if (w==1){
                rr=5;
                w=1;
                //trig_stat=1;
            }
        }

        if (rr==5){
            //std::cout << "case rr=3" << std::endl;
            //x_incr << -0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0,0,0;
            x_incr << 0,0,0,0,0,0;
            //x_incr << 0,0,mag-mag*(w/dur),0,0,0;
            if (w==1){
                //rr=4;
                //w=0;
                trig_stat=1;
            }

        }




        qc << Ja.inverse()*(x_incr-x_incr_0);//+(q_old-q_0)*0.3;
        delta_q << delta_q+qc;
        //std::cout << delta_q << std::endl;
        q_new << q_freeze+delta_q;
        x_incr_0 << x_incr;
    }








    if(random_num==5 || random_num==6 || random_num==7 || random_num==8)
    {
        x_00 << x_0;
        x_0 << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;
        q_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;//+(q_old-q_0)*0.3;
        trig_stat=1;
    }















    //     if(random_num==3)
    //   {
    //    //std::cout << w << std::endl;
    //     x_incr << 0,0,0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0;
    //    //std::cout << x_incr(0) << std::endl;
    //     //x_incr << 0,0,0,0,0,0;
    //   //q_new << Ja.inverse()*(x_incr)+q_freeze;//+(q_old-q_0)*0.3;
    //   qc << Ja.inverse()*(x_incr-x_incr_0);//+(q_old-q_0)*0.3;
    //   delta_q << delta_q+qc;
    //   //std::cout << delta_q << std::endl;
    //   q_new << q_freeze+delta_q;
    //   x_incr_0 << x_incr;
    //   }
    //     if(random_num==4)
    //   {
    //    //std::cout << w << std::endl;
    //     x_incr << 0,0,-0.02*((0.5)+(0.5*sin((w*M_PI/200)-M_PI/2))),0,0,0;
    //    //std::cout << x_incr(0) << std::endl;
    //     //x_incr << 0,0,0,0,0,0;
    //   //q_new << Ja.inverse()*(x_incr)+q_freeze;//+(q_old-q_0)*0.3;
    //   qc << Ja.inverse()*(x_incr-x_incr_0);//+(q_old-q_0)*0.3;
    //   delta_q << delta_q+qc;
    //   //std::cout << delta_q << std::endl;
    //   q_new << q_freeze+delta_q;
    //   x_incr_0 << x_incr;
    //   }






    w=w+1;
    w2=w;
    //force << ftx,0,fty_freeze+1*sin(M_PI*w/50),0,0,0; //BEFORE UNCOMMENTING< CHANGE DAMPING Y TO 10000, STIFFNESS Y to 500
    //force << ftx,0,fty,0,0,0;
}
if(trig_stat==1)
{
    //system("canberra-gtk-play -f beep.wav");
    x_incr_0 << 0,0,0,0,0,0;
    delta_q << 0,0,0,0,0,0;
    cc=0;
    w=0;
    trigger = 1;
    delay_gate=0;
    rr=1;
    trig_stat=0;
    //force_divider=0;
    //std::cout << "test" << std::endl;
}


if(steady<2000)
{
    delay_gate=0;
    ww=0;
    force << 0,0,0,0,0,0;
    zerox=  100*al*(double)data[0]/1000000+(1-100*al)*zerox;
    zeroy=  100*al*(double)data[1]/1000000+(1-100*al)*zeroy;

    x_00 << x_0;
    x_0 << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;

    x_new << (inertia/0.00001+damping/0.001+stiffness).inverse()*(force+inertia*(x_0-x_00)/0.00001+stiffness*(x_e-x_0))+x_0;
    //std::cout << x_new << std::endl;
}
//std::cout << "zeroy" << std::endl;
//std::cout << zeroy << std::endl;
//-------------------------------------------------------------------------------
//force << 0,0,0,0,0,0;
met=met+1;
if(met==2000)
{
    met=0;
    //std::cout << '\a' << std::endl;
}
//---------------------------------------
//MatrixXd q_new(6,1); q_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;

//	MatrixXd q_new(6,1); q_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;

// 	MatrixXd q_new(6,1); q_new << Ja.inverse()*(inertia/(0.00001*stupid)+damping/0.001+stiffness).inverse()*(force+inertia/(0.00001*stupid)*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;
//

if(steady>1200)
{
    //q_0 << q_new;
    //   stupid=1.0;
    // //  q_new(6,1); q_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;
    // // //std::cout << "---" << std::endl;
}
//std::cout << "ww" << std::endl;
// 	std::cout << q_new << std::endl;
// 	std::cout << "---" << std::endl;
// 	std::cout << test_new << std::endl;
// 	std::cout << "---" << std::endl;


//std::cout << T06(1,3) << std::endl;
//std::cout << T06(1,3) << std::endl;


// 	steady=steady+1;
// 	if(steady<2000)
// 	{
// 	  q_0 << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5];
// 	}

//MatrixXd q_new(6,1); q_new << q_0;
//std::cout << x_0 << std::endl;
//std::cout << "----" << std::endl;

//torques-

//MatrixXd joints_new(6,1); joints_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*((Ja.transpose()).inverse()*torques+(2*inertia+damping)*J_1*q_1-inertia*J_0*q_0-stiffness*equilibrium_point)*180/M_PI;
//std::cout << joints_new << std::endl;







stime=client.GetTimeStamp();  //get a nanosecond timestamp
//std::cout << stime << std::endl;


//-------------------------Shared Memory---------------------------
/*data=MakeFloatSharedMemory(7); //creates and initializes to 0 a shared memory float array
data[0]=MJoint[0];
data[1]=MJoint[1];
data[2]=MJoint[2];
data[3]=MJoint[3];
data[4]=MJoint[4];
data[5]=MJoint[5];
data[6]=MJoint[6];
//std::cout << data[3] << std::endl;*/

//-----------------------------------------------------------------

//output data to file
// fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],MJoint[6],ETorque[0],ETorque[1],ETorque[2],ETorque[3],ETorque[4],ETorque[5],ETorque[6],stime);

//error_delta=T06_freeze_x-T06(0,3);
//std::cout << error_delta << std::endl;

//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],MJoint[6],q_new(0),q_new(1),q_new(2),q_new(3),q_new(4),q_new(5),-0.7854,perturb_flag);
fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf\n",
                    count,
                    MJoint[0],MJoint[1],MJoint[2],
                    MJoint[3],MJoint[4],MJoint[5],
                    ftx_un,
                    fty_un,
                    perturb_flag,
                    q_new(0),q_new(1),q_new(2),
                    q_new(3),q_new(4),q_new(5));
//printf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],ftx_un,fty_un,perturb_flag,q_new(0),q_new(1),q_new(2),q_new(3),q_new(4),q_new(5),elapsed);
//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],ftx_un,fty_un,perturb_flag);
//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,q_new(0),q_new(1),q_new(2),q_new(3),q_new(4),q_new(5),ftx_un,fty_un,perturb_flag);







//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],MJoint[6],q_new(3),x_new(0),x_new(1),x_new(2),T06(0,3),T06(1,3),T06(2,3),stime);
//std::cout << count << std::endl;


if (c==1)
{
    count2++;

    if (count2==200 || !success) //was "rows" not 400
    {
        //enough=1;
        c=0;
        count2=0;
        std::cout << c << std::endl;
    }
    else
    {
        //move comanded joint to last joint
        memcpy(client.LastJoint, client.NextJoint,7*sizeof(double));
        //read in next joint command


        success2 = fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf",
                                    &client.NextJoint[0],
                                    &client.NextJoint[1],
                                    &client.NextJoint[2],
                                    &client.NextJoint[3],
                                    &client.NextJoint[4],
                                    &client.NextJoint[5],
                                    &client.NextJoint[6]);
        //std::cout << client.NextJoint[0] << std::endl;

        success2 = fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf",
                                    &MJoint[0],
                                    &MJoint[1],
                                    &MJoint[2],
                                    &MJoint[3],
                                    &MJoint[4],
                                    &MJoint[5],
                                    &MJoint[6]);

        if (!success2)
        {
            fprintf(stdout,"Error in reading file!\n");
            fclose(OutputFile);
            return 1;
        }

    }
}
else
{

    //q_0 << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5);
    x_00 << x_0;
    x_0 << T06(0,3),T06(1,3),T06(2,3),phi_euler,theta_euler,psi_euler;









    x_new << (inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+x_0;


    //std::cout << x_new(2) << std::endl;
    if(trigger==1) //prevent new joints commands from being sent during position perturbation
    {
        if(0.21<x_new(2) & x_new(2)<.51)
        {
            q_new << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;//+(q_old-q_0)*0.3;
            // std::cout << q_new << std::endl;
        }
    }























    q_old << q_new;

    q_delay << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;
    if(outerdelay > 9)
    {

        //q_delay << Ja.inverse()*(inertia+damping+stiffness).inverse()*(force+inertia*(x_0-x_00)+stiffness*(x_e-x_0))+q_0;
        outerdelay=0;

    }
    outerdelay=outerdelay+1;
    // std::cout << x_0 << std::endl;
    // std::cout << "---" << std::endl;
    //std::cout << (q_old-q_0)*0.75 << std::endl;



    //



    //q_old << q_new;
    //std::cout << x_new(2) << std::endl;
    //std::cout << "-actual-" << std::endl;
    //std::cout << q_new << std::endl;
    //std::cout << "--"<< std::endl;
    //std::cout << x_new << std::endl;
    //std::cout << "--"<< std::endl;
    /*client.NextJoint[0]=-1.779862; //UPRIGHT
    client.NextJoint[1]=0.821814;
    client.NextJoint[2]=-0.067855;
    client.NextJoint[3]=1.302481;
    client.NextJoint[4]=0.284275;
    client.NextJoint[5]=-1.118251;
    client.NextJoint[6]=-0.178413;*/

    client.NextJoint[0]=q_new(0);
    client.NextJoint[1]=q_new(1);
    client.NextJoint[2]=q_new(2);
    client.NextJoint[3]=q_new(3);
    client.NextJoint[4]=q_new(4);
    client.NextJoint[5]=q_new(5);
    client.NextJoint[6]=-0.7854;
    //client.NextJoint[6]=0;
    /*
    client.NextJoint[0]=0;
    client.NextJoint[1]=q_new(1);
    client.NextJoint[2]=0;
    client.NextJoint[3]=q_new(3);
    client.NextJoint[4]=-1.5708;
    client.NextJoint[5]=q_new(5);
    //client.NextJoint[6]=-0.958709;
    client.NextJoint[6]=0;*/


    /*client.NextJoint[0]=0.785399; //forward
    client.NextJoint[1]=-0.863358;
    client.NextJoint[2]=0.000000;
    client.NextJoint[3]=-1.787378;
    client.NextJoint[4]=0.000000;
    client.NextJoint[5]=-0.924020; 625
    client.NextJoint[6]=0.785398;*/
}

}

// clock_gettime(CLOCK_MONOTONIC, &finish2);
//  elapsed = (finish2.tv_sec - start2.tv_sec);
//  elapsed = (finish2.tv_nsec - start2.tv_nsec) / 1000000000.0;




//printf("%f\n", elapsed);
}//end of while (not enough) loop

fclose(OutputFile);
fprintf(stdout,"File closed.\n\n\n");
// disconnect from controller

fprintf(stdout,"Shhh.. I'm sleeping!\n");
usleep(10000000);//microseconds //wait for close on other side (10s)
app.disconnect();

gettimeofday (&start, NULL);

return 1;
}



////////////////////////////////////////////////////////////////
FILE *NewDataFile(void) //this may be specific to linux OS
{
    FILE *fp;
    time_t rawtime;
    struct tm *timeinfo;
    char namer[40];

    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (namer,40,"Output%Y-%j-%H_%M_%S.txt",timeinfo);//creates a file name that has year-julian day-hour min second (unique for each run, no chance of recording over previous data)
    fp = fopen(namer,"w");//open output file
    return fp;
}

/////////////////////////////SHARED MEMORY///////////////////////////////// 8>7

//    /////////////////////////////SHARED MEMORY///////////////////////////////// 8>7

int *MakeFloatSharedMemory(int HowBig){
    key_t key;
    int shmid;
    int *dataShared;

    dataShared = (int *) malloc(HowBig*sizeof(int));
    /* make the key */
    if ( (key=ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile",'R')) == -1 ){
        perror("ftok-->");
        exit(1);
    }

    if ((shmid = shmget(key, SHM_SIZE, 0666 | IPC_CREAT)) == -1){
        perror("shmget");
        exit(1);
    }

    dataShared = (int *) shmat(shmid, (void *)0, 0);

    if (dataShared == (int *) (-1)){
        perror("shmat");
        exit(1);
    }

    for(int i=0;i<HowBig;i++){
        dataShared[i]=0.0;
    }

    return dataShared;
}

//    /////////////////////////////SHARED MEMORY -DISPLAY///////////////////////////////// 8>7

float *MakeFloatSharedMemory2(int HowBig2)
{
    key_t key2;
    int shmid2;
    float *dataShared2;

    dataShared2 = (float *) malloc(HowBig2*sizeof(float));
    /* make the key */
    if ( (key2=ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile2",'R')) == -1 )
    {
        perror("ftok-->");
        exit(1);
    }

    if ((shmid2 = shmget(key2, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
    {
        perror("shmget");
        exit(1);
    }

    dataShared2 = (float *) shmat(shmid2, (void *)0, 0);

    if (dataShared2 == (float *) (-1))
    {
        perror("shmat");
        exit(1);
    }

    for(int i=0;i<HowBig2;i++)
    {
        dataShared2[i]=0.0;
    }

    return dataShared2;
}
