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
#include "friMonitoringMessageDecoder.h"
#include "pb_decode.h"


using namespace KUKA::FRI;

//******************************************************************************
MonitoringMessageDecoder::MonitoringMessageDecoder(FRIMonitoringMessage* pMessage, int num)
   : m_nNum(num), m_pMessage(pMessage)
{
   initMessage();
}

//******************************************************************************
MonitoringMessageDecoder::~MonitoringMessageDecoder()
{

}

//******************************************************************************
void MonitoringMessageDecoder::initMessage()
{
   // set initial data
   // it is assumed that no robot information and monitoring data is avaible and therefore the 
   // optional fields are initialized with false
   m_pMessage->has_robotInfo = false;
   m_pMessage->has_monitorData = false;
   m_pMessage->has_connectionInfo = true;
   m_pMessage->has_ipoData = false;
   m_pMessage->has_endOfMessageData = false;
   
   m_pMessage->header.messageIdentifier = 0;
   m_pMessage->header.reflectedSequenceCounter = 0;
   m_pMessage->header.sequenceCounter = 0;

   m_pMessage->connectionInfo.sessionState = FRISessionState_IDLE;
   m_pMessage->connectionInfo.quality = FRIConnectionQuality_POOR;

   // allocate and map memory for protobuf repeated structures
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.measuredJointPosition.value,
         &m_tSendContainer.m_AxQMsrLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.measuredTorque.value,
         &m_tSendContainer.m_AxTauMsrLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.commandedJointPosition.value,
         &m_tSendContainer.m_AxQCmdT1mLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.commandedTorque.value,
         &m_tSendContainer.m_AxTauCmdLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.externalTorque.value,
         &m_tSendContainer.m_AxTauExtMsrLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE,m_nNum,
         &m_pMessage->ipoData.jointPosition.value,
         &m_tSendContainer.m_AxQCmdIPO);

   map_repeatedInt(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->robotInfo.driveState,
         &m_tSendContainer.m_AxDriveStateLocal);
   
}

//******************************************************************************
bool MonitoringMessageDecoder::decode(char* buffer, int size)
{
    pb_istream_t stream = pb_istream_from_buffer((uint8_t*)buffer, size);

    bool status = pb_decode(&stream, FRIMonitoringMessage_fields, m_pMessage);
    if (!status)
    {
        printf("!!decoding error: %s!!\n", PB_GET_ERROR(&stream));
    }

    return status;
}
