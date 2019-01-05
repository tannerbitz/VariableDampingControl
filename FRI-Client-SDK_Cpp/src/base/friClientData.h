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
#ifndef _KUKA_FRI_CLIENT_DATA_H
#define _KUKA_FRI_CLIENT_DATA_H

#include "FRIMessages.pb.h"
#include "friMonitoringMessageDecoder.h"
#include "friCommandMessageEncoder.h"


namespace KUKA
{
namespace FRI
{

   struct ClientData
   {
      char receiveBuffer[FRI_MONITOR_MSG_MAX_SIZE];//!< monitoring message receive buffer
      char sendBuffer[FRI_COMMAND_MSG_MAX_SIZE];   //!< command message send buffer

      FRIMonitoringMessage monitoringMsg;          //!< monitoring message struct
      FRICommandMessage commandMsg;                //!< command message struct
      
      MonitoringMessageDecoder decoder;            //!< monitoring message decoder
      CommandMessageEncoder encoder;               //!< command message encoder
      
      ESessionState lastState;                     //!< last FRI state
      uint32_t sequenceCounter;                    //!< sequence counter for command messages
      uint32_t lastSendCounter;                    //!< steps since last send command
      uint32_t expectedMonitorMsgID;               //!< expected ID for received monitoring messages     

      ClientData(int numDofs) 
      : decoder(&monitoringMsg, numDofs), 
        encoder(&commandMsg, numDofs),
        lastState(IDLE),
        sequenceCounter(0),
        lastSendCounter(0),
        expectedMonitorMsgID(0)
      {}
      
      void resetCommandMessage()
      {
         commandMsg.commandData.has_jointPosition = false;
         commandMsg.commandData.has_cartesianWrenchFeedForward = false;
         commandMsg.commandData.has_jointTorque = false;
         commandMsg.has_commandData = false;     
      }
      
   };
   
}
}


#endif // _KUKA_FRI_CLIENT_DATA_H
