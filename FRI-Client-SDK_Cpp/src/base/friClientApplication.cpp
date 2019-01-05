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
#include "friClientApplication.h"
#include "friClientIf.h"
#include "friConnectionIf.h"
#include "friClientData.h"
#include "FRIMessages.pb.h"

using namespace KUKA::FRI;

//******************************************************************************
ClientApplication::ClientApplication(IConnection& connection, IClient& client)
   : _connection(connection), _client(client), _data(NULL)
{
   _data = _client.createData();
}

//******************************************************************************
ClientApplication::~ClientApplication()
{
   disconnect();
   delete _data;
}

//******************************************************************************
bool ClientApplication::connect(int port, const char *remoteHost)
{
   if (_connection.isOpen()) 
   {
      printf("Warning: client application already connected!\n");
      return true;
   }
   
   return _connection.open(port, remoteHost);
}
   
//******************************************************************************
void ClientApplication::disconnect()
{
   if (_connection.isOpen()) _connection.close();
}

//******************************************************************************
bool ClientApplication::step()
{
   if (!_connection.isOpen())
   {
      printf("Error: client application is not connected!\n");
      return false;
   }
   
   // **************************************************************************
   // Receive and decode new monitoring message
   // **************************************************************************
   int size = _connection.receive(_data->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);
   
   if (size <= 0)
   {  // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
      printf("Error: failed while trying to receive monitoring message!\n");
      return false;
   }
   
   if (!_data->decoder.decode(_data->receiveBuffer, size))
   {
      return false;
   }
   
   // check message type (so that our wrappers match)
   if (_data->expectedMonitorMsgID != _data->monitoringMsg.header.messageIdentifier)
   {
      printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            (int)_data->monitoringMsg.header.messageIdentifier,
            (int)_data->expectedMonitorMsgID);
      return false;
   }   
   
   // **************************************************************************
   // callbacks
   // **************************************************************************
   
   // reset commmand message before callbacks
   _data->resetCommandMessage();
   
   ESessionState currentState = (ESessionState)_data->monitoringMsg.connectionInfo.sessionState;
   
   if (_data->lastState != currentState)
   {
      _client.onStateChange(_data->lastState, currentState);
      _data->lastState = currentState;
   }
   
   switch (currentState)
   {
      case MONITORING_WAIT:
      case MONITORING_READY:
         _client.monitor();
         break;
      case COMMANDING_WAIT:
         _client.waitForCommand();
         break;
      case COMMANDING_ACTIVE:
         _client.command();
         break;
      case IDLE:
      default:
         return true; // nothing to send back
   }
   
   // **************************************************************************
   // Encode and send command message
   // **************************************************************************
   
   _data->lastSendCounter++;
   // check if its time to send an answer
   if (_data->lastSendCounter >= _data->monitoringMsg.connectionInfo.receiveMultiplier)
   {
      _data->lastSendCounter = 0;
      
      // set sequence counters
      _data->commandMsg.header.sequenceCounter = _data->sequenceCounter++;
      _data->commandMsg.header.reflectedSequenceCounter = 
            _data->monitoringMsg.header.sequenceCounter;
      
      if (!_data->encoder.encode(_data->sendBuffer, size))
      {
         return false;
      }
      
      if (!_connection.send(_data->sendBuffer, size))
      {
         printf("Error: failed while trying to send command message!\n");
         return false;
      }
   }
   
   return true;
}

