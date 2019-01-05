
/**


*/
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "PositionControlClient.h"
#include "friLBRState.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
PositionControlClient::PositionControlClient() //constructor
{
}

//******************************************************************************
PositionControlClient::~PositionControlClient() //destructor
{
}

//******************************************************************************
void PositionControlClient::intvalues(double MRPStep[7], double MaxJLRad[7], double MinJLRad[7])
{

    memcpy(MaxRadPerStep,MRPStep,7*sizeof(double));
    memcpy(MaxJointLimitRad,MaxJLRad,7*sizeof(double));
    memcpy(MinJointLimitRad,MinJLRad,7*sizeof(double));


    fprintf(stdout,"Position Control Client initialized:\n");
    fprintf(stdout,"MaxRadPerStep:%lf %lf %lf %lf %lf %lf %lf\n",MaxRadPerStep[0],MaxRadPerStep[1],MaxRadPerStep[2],MaxRadPerStep[3],MaxRadPerStep[4],MaxRadPerStep[5],MaxRadPerStep[6]);
    fprintf(stdout,"MaxJointLimitRad:%lf %lf %lf %lf %lf %lf %lf\n",MaxJointLimitRad[0],MaxJointLimitRad[1],MaxJointLimitRad[2],MaxJointLimitRad[3],MaxJointLimitRad[4],MaxJointLimitRad[5],MaxJointLimitRad[6]);
    fprintf(stdout,"MinJointLimitRad:%lf %lf %lf %lf %lf %lf %lf\n",MinJointLimitRad[0],MinJointLimitRad[1],MinJointLimitRad[2],MinJointLimitRad[3],MinJointLimitRad[4],MinJointLimitRad[5],MinJointLimitRad[6]);


}

//******************************************************************************
void PositionControlClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // set flag to know when state is ready for command

   KukaState=newState;

}
//******************************************************************************
void PositionControlClient::command()
{

    int Errors=0;
    for(int i=0; i<7; i++)
    {
        if(MinJointLimitRad[i]>NextJoint[i])
        {
            //fprintf(stdout,"minimum joint limit error.Joint:%d\tmin:%lf\tnext:%lf\n",i,MinJointLimitRad[i],NextJoint[i]);

            Errors=2;
        }


        if(MaxJointLimitRad[i]<NextJoint[i])
        {
         //fprintf(stdout,"maximum joint limit error.Joint:%d\n",i);
            Errors=2;
        }

        if(MaxRadPerStep[i]<(fabs(LastJoint[i]-NextJoint[i])))
        {
           //fprintf(stdout,"maximum joint velocity error.Joint:%d\n",i);
            Errors=1;
        }
    }


   /* if(Errors!=0)
    {
        memcpy(NextJoint,LastJoint,7*sizeof(double));
        fprintf(stdout,"ERROR!\neRROR!\nErROR!\nERrOR!\nERRoR!\nERROr!\n");
        fprintf(stdout,"Error in Position Control Clinet:Type:%d\n",Errors);

    }*/


    robotCommand().setJointPosition(NextJoint);


}


//*********************************************************************************
float PositionControlClient::GetTimeStep()
{
    return robotState().getSampleTime();
}
//*********************************************************************************

//*********************************************************************************
double* PositionControlClient::GetMeasJoint()
{
     memcpy(MeasuredJoint,robotState().getMeasuredJointPosition(),sizeof(double)*7 );
     return MeasuredJoint;
}




//*********************************************************************************
double* PositionControlClient::GetExtTor()
{
     memcpy(ExternalTorque,robotState().getExternalTorque(),sizeof(double)*7 );
     return ExternalTorque;
}

//*********************************************************************************
double* PositionControlClient::GetComJoint()
{
    memcpy(CommandedJoint,robotState().getCommandedJointPosition() ,sizeof(double)*7 );
    return CommandedJoint;

}

//*********************************************************************************
int PositionControlClient::GetTimeStamp()
{

    SecTime=robotState().getTimestampNanoSec();
    return SecTime;
}


