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
Copyright (C)  2014-2015 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  



*/
#include "friLBRState.h"
#include "friLBRCommand.h"
#include "FRIMessages.pb.h"
#include "pb_frimessages_callbacks.h"

using namespace KUKA::FRI;

//******************************************************************************
void LBRCommand::setJointPosition(const double* values)
{
   _message->has_commandData = true;
   _message->commandData.has_jointPosition = true;
   tRepeatedDoubleArguments *dest =
            (tRepeatedDoubleArguments*)_message->commandData.jointPosition.value.arg;
   memcpy(dest->value, values, LBRState::NUMBER_OF_JOINTS * sizeof(double));
}

//******************************************************************************
void LBRCommand::setWrench(const double* wrench)
{
   _message->has_commandData = true;
   _message->commandData.has_cartesianWrenchFeedForward = true;

   double *dest = _message->commandData.cartesianWrenchFeedForward.element;
   memcpy(dest, wrench, 6 * sizeof(double));
}
//******************************************************************************
void LBRCommand::setTorque(const double* torques)
{
   _message->has_commandData = true;
   _message->commandData.has_jointTorque= true;

   tRepeatedDoubleArguments *dest =
              (tRepeatedDoubleArguments*)_message->commandData.jointTorque.value.arg;
   memcpy(dest->value, torques, LBRState::NUMBER_OF_JOINTS * sizeof(double));
}
