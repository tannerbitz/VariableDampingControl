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
Copyright (C)  2014 
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
#include "LBRJointSineOverlayClient.h"
#include "friLBRState.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRJointSineOverlayClient::LBRJointSineOverlayClient(unsigned int jointMask, 
      double freqHz, double amplRad, double filterCoeff) 
   : _jointMask(jointMask)
   , _freqHz(freqHz)
   , _amplRad(amplRad)
   , _filterCoeff(filterCoeff)
   , _offset(0.0)
   , _phi(0.0)
   , _stepWidth(0.0)
{
   printf("LBRJointSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (rad): %f\n"
         "\tfilterCoeff: %f\n",
         jointMask, freqHz, amplRad, filterCoeff);
}

//******************************************************************************
LBRJointSineOverlayClient::~LBRJointSineOverlayClient()
{
}
      
//******************************************************************************
void LBRJointSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         _offset = 0.0;
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
void LBRJointSineOverlayClient::command()
{
   // calculate new offset
   double newOffset = _amplRad * sin(_phi);
   _offset = _offset * _filterCoeff + newOffset * (1.0 - _filterCoeff);
   _phi += _stepWidth;
   if (_phi >= 2 * M_PI) _phi -= 2 * M_PI;      
   // add offset to ipo joint position for all masked joints
   double jointPos[LBRState::NUMBER_OF_JOINTS];
   memcpy(jointPos, robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
   for (int i=0; i< LBRState::NUMBER_OF_JOINTS; i++)
   {
      if (_jointMask & (1<<i))
      {
         jointPos[i] += _offset;
      }
   }
   robotCommand().setJointPosition(jointPos);
}
