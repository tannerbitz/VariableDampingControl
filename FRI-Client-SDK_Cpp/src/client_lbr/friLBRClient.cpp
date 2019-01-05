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
#include <stdio.h>
#include "friLBRClient.h"
#include "friClientData.h"

using namespace KUKA::FRI;

//******************************************************************************
LBRClient::LBRClient()
{
   
}

//******************************************************************************
LBRClient::~LBRClient()
{
   
}

//******************************************************************************
void LBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   // TODO: String converter function for states
   printf("LBRiiwaClient state changed from %d to %d\n", oldState, newState);
}

//******************************************************************************
void LBRClient::monitor()
{
   robotCommand().setJointPosition(robotState().getCommandedJointPosition());
}

//******************************************************************************
void LBRClient::waitForCommand()
{
   robotCommand().setJointPosition(robotState().getIpoJointPosition());
}

//******************************************************************************
void LBRClient::command()
{
   robotCommand().setJointPosition(robotState().getIpoJointPosition());
}

//******************************************************************************
ClientData* LBRClient::createData()
{
   ClientData* data = new ClientData(_robotState.NUMBER_OF_JOINTS);
   
   // link monitoring and command message to wrappers
   _robotState._message = &data->monitoringMsg;
   _robotCommand._message = &data->commandMsg;
   
   // set specific message IDs
   data->expectedMonitorMsgID = _robotState.LBRMONITORMESSAGEID;
   data->commandMsg.header.messageIdentifier = _robotCommand.LBRCOMMANDMESSAGEID;
   
   return data;
}
