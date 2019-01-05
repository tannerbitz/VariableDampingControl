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
#ifndef _KUKA_FRI_LBR_CLIENT_H
#define _KUKA_FRI_LBR_CLIENT_H

#include "friClientIf.h"
#include "friLBRState.h"
#include "friLBRCommand.h"

namespace KUKA
{
namespace FRI
{

   /**
    * \brief Implementation of the IClient interface for the KUKA LBR (leightweight) robots.
    * 
    * Provides access to the current LBR state and the possibility to send new 
    * commands to the LBR.  
    */
   class LBRClient : public IClient
   {
   
   public:
      
      /** \brief Constructor. */
      LBRClient();
      
      /** \brief Destructor. */
      ~LBRClient();
         
      /** 
       * \brief Callback that is called whenever the FRI session state changes.
       * 
       * @param oldState previous FRI session state
       * @param newState current FRI session state
       */
      virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
      /**
       * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
       */
      virtual void monitor();
   
      /**
       * \brief Callback for the FRI session state 'Commanding Wait'.
       */
      virtual void waitForCommand();
   
      /**
       * \brief Callback for the FRI session state 'Commanding'.
       */
      virtual void command();
   
      /**
       * \brief Provide read access to the current robot state.
       * 
       * @return Reference to the LBRState instance
       */
      const LBRState& robotState() const { return _robotState; }
      
      /**
       * \brief Provide write access to the robot commands.
       * 
       * @return Reference to the LBRCommand instance
       */
      LBRCommand& robotCommand() { return _robotCommand; }
      
   private:
      
      LBRState _robotState;      //!< wrapper class for the FRI monitoring message
      LBRCommand _robotCommand;  //!< wrapper class for the FRI command message
      
      /**
       * \brief Method to create and initialize the client data structure (used internally).
       * 
       * @return newly allocated client data structure
       */
      virtual ClientData* createData();
      
   };
   
}
}


#endif // _KUKA_FRI_LBR_CLIENT_H
