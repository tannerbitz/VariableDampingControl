
/*
  PositionControlApp


*/
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h> // strstr
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>


using namespace KUKA::FRI;




#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"  //ip address set for KONI connection

FILE *NewDataFile(void);

int main (int argc, char** argv)
{
   // parse command line arguments
    if (argc < 2)
        {
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
   

    FILE *InputFile;
    FILE *OutputFile;

    bool success = true;
    bool success2 = true;

    int count=0;
    int enough=0;
    int rows=0;
    int i=0;
    int cycle=2000;
   
    int stime=0;
    float sampletime=0; //will be determined from FRI
    double MJoint[7]={0}; //last measured joint position (not sure time when controller measured, be careful using this for feedback loop)
    double ETorque[7]={0}; //External torque :supposedly the torque applied to the robot (subtracts the torque due to the robot, ackording to Kuka)

    double MaxRadPerSec[7]={1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159}; //absolute max velocity (no load from KUKA manual for iiwa 800)
    //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
    double MaxRadPerStep[7]={0};//will be calculated
    double MaxJointLimitRad[7]={2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543};//Max joint limits in radians (can be changed to further restrict motion of robot)
    double MinJointLimitRad[7]={-2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543}; //Min joint limits in radians (can be changed to further restrict motion of robot)
    double FirstPositionDelta[7]={0.0175,0.0175,0.0175,0.0175,0.0175,0.0175,0.0175}; //maximum deviation from initial position in trajectory from start position in robot(radians)
   

    //Get value for the time step of the command cycle (used for determining max velocity)
   // sampletime=client.GetTimeStep();
    sampletime=0.005;
    fprintf(stdout,"Sample Time:%f seconds\n",sampletime);

    //calculate max step value
    for(i=0;i<7;i++)
    {
        MaxRadPerStep[i]=sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
    }
 

   // create new joint position client
      PositionControlClient client;
   /*
client.intvalues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);
*/



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

    if(rows<4)
    {
        fprintf(stdout,"Number of Rows is %d\n Are you sure the file is in correct format?\n First number should be number of rows.  \nThen each row is 7 joint values in radians.\n",rows);
        return 1;
    }
    if (!success2)
    {
        fprintf(stdout,"Error in reading file!\n");
        return 1;
    }
  

    //read in first 7 joint positions (these should be in radians)
    success2=fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf", &client.NextJoint[0],&client.NextJoint[1],&client.NextJoint[2],&client.NextJoint[3],&client.NextJoint[4],&client.NextJoint[5],&client.NextJoint[6]);
    if (!success2)
    {
        fprintf(stdout,"Error in reading file!\n");
        return 1;
    }


    //check to insure that the first joint command is not significantly different from the robot's initial position
     memcpy(MJoint,client.GetMeasJoint(),sizeof(double)*7 );
     memcpy(ETorque,client.GetExtTor(),sizeof(double)*7 ); //gets the external torques at the robot joints (supposedly subtracts the torques caused by the robot)
    //print out to file for condition 0
    fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],MJoint[6],ETorque[0],ETorque[1],ETorque[2],ETorque[3],ETorque[4],ETorque[5],ETorque[6],stime);


    for(i=0;i<7;i++)
    {
        /*if (fabs(MJoint[i]-client.NextJoint[i])>FirstPositionDelta[i])
        {
            fprintf(stdout,"ERROR!!! Intital Position of Robot too far from first command!!!\n");
            return 1;
        }*/
    }

    // repeatedly call the step routine to receive and process FRI packets


    while (!enough)
    {


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
        stime=client.GetTimeStamp();  //get a nanosecond timestamp

    //output data to file
        fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",count,MJoint[0],MJoint[1],MJoint[2],MJoint[3],MJoint[4],MJoint[5],MJoint[6],ETorque[0],ETorque[1],ETorque[2],ETorque[3],ETorque[4],ETorque[5],ETorque[6],stime);
        if (count==cycle || !success) //was count==rows
        {

            enough=1;
        }
        else
        {

            //move comanded joint to last joint
            memcpy(client.LastJoint, client.NextJoint,7*sizeof(double));
            //read in next joint command
            success2 = fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf", &client.NextJoint[0],&client.NextJoint[1],&client.NextJoint[2],&client.NextJoint[3],&client.NextJoint[4],&client.NextJoint[5],&client.NextJoint[6]);
//std::cout << client.NextJoint[5] << std::endl;
            if (!success2)
            {
                fprintf(stdout,"Error in reading file!\n");
                fclose(OutputFile);
                return 1;
            }

        }
    }

    }//end of while (not enough) loop

    fclose(OutputFile);
    fprintf(stdout,"File closed.\n\n\n");
   // disconnect from controller

   fprintf(stdout,"Shhh.. I'm sleeping!\n");
   usleep(10000000);//microseconds //wait for close on other side
   app.disconnect();
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

   ////////////////////////////////////////////////////////////// 8>7
