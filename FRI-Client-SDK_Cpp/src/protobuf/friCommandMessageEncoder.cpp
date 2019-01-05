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
#include "friCommandMessageEncoder.h"
#include "pb_encode.h"

using namespace KUKA::FRI;

//******************************************************************************
CommandMessageEncoder::CommandMessageEncoder(FRICommandMessage* pMessage, int num)
   : m_nNum(num), m_pMessage(pMessage)
{
   initMessage();
}

//******************************************************************************
CommandMessageEncoder::~CommandMessageEncoder()
{

}

//******************************************************************************
void CommandMessageEncoder::initMessage()
{
   m_pMessage->has_commandData = false;
   m_pMessage->has_endOfMessageData = false;
   m_pMessage->commandData.has_jointPosition = false;
   m_pMessage->commandData.has_cartesianWrenchFeedForward = false;
   m_pMessage->commandData.has_jointTorque = false;

   m_pMessage->header.messageIdentifier = 0;
   // init with 0. Necessary for creating the correct reflected sequence count in the monitoring msg
   m_pMessage->header.sequenceCounter = 0;
   m_pMessage->header.reflectedSequenceCounter = 0;

   // allocate and map memory for protobuf repeated structures
   map_repeatedDouble(FRI_MANAGER_NANOPB_ENCODE, m_nNum, 
         &m_pMessage->commandData.jointPosition.value,
         &m_tRecvContainer.jointPosition);
   map_repeatedDouble(FRI_MANAGER_NANOPB_ENCODE, m_nNum, 
         &m_pMessage->commandData.jointTorque.value,
         &m_tRecvContainer.jointTorque);
   // nanopb encoding needs to know how many elements the static array contains
   // a Cartesian wrench feed forward vector always contains 6 elements
   m_pMessage->commandData.cartesianWrenchFeedForward.element_count = 6;
}

//******************************************************************************
bool CommandMessageEncoder::encode(char* buffer, int& size)
{
    // generate stream for encoding
    pb_ostream_t stream = pb_ostream_from_buffer((uint8_t*)buffer, FRI_COMMAND_MSG_MAX_SIZE);
    // encode monitoring Message to stream
    bool status = pb_encode(&stream, FRICommandMessage_fields, m_pMessage);
    size = stream.bytes_written;
    if (!status)
    {
        printf("!!encoding error: %s!!\n", PB_GET_ERROR(&stream));
    }
    return status;
}
