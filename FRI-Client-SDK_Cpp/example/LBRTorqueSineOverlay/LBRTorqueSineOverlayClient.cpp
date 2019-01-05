// version: 1.7
/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2015 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  



*/
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "LBRTorqueSineOverlayClient.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRTorqueSineOverlayClient::LBRTorqueSineOverlayClient(unsigned int jointMask, 
      double freqHz, double torqueAmplitude) 
   :_jointMask(jointMask)
   , _freqHz(freqHz)
   , _torqueAmpl(torqueAmplitude)
   , _phi(0.0)
   , _stepWidth(0.0)
{
   printf("LBRTorqueSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (Nm): %f\n",
         jointMask, freqHz, torqueAmplitude);
   
   for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
}

//******************************************************************************
LBRTorqueSineOverlayClient::~LBRTorqueSineOverlayClient()
{
}
      
//******************************************************************************
void LBRTorqueSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
         _phi = 0.0;
         _stepWidth = 2 * M_PI * _freqHz * robotState().getSampleTime();
         break;
      }
      default:
      {
         break;
      }
   }
}
//******************************************************************************

void LBRTorqueSineOverlayClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();
   
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(_torques);
   }
}
//******************************************************************************
void LBRTorqueSineOverlayClient::command()
{
    // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();
    
    // Check for correct ClientCommandMode.
    if (robotState().getClientCommandMode() == TORQUE)
    { 
       // calculate  offset
        double offset = _torqueAmpl * sin(_phi);
       
       _phi += _stepWidth;
       if (_phi >= 2 * M_PI) _phi -= 2 * M_PI;      

       for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
       {
          if (_jointMask & (1<<i))
          {
              _torques[i] = offset;
          }
       }
       // Set superposed joint torques.
       robotCommand().setTorque(_torques);
    }
}
