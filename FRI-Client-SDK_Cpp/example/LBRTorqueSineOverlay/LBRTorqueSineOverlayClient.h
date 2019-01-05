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
#ifndef _KUKA_FRI_LBR_TORQUE_SINE_OVERLAY_CLIENT_H
#define _KUKA_FRI_LBR_TORQUE_SINE_OVERLAY_CLIENT_H

#include "friLBRClient.h"

using namespace KUKA::FRI;

/**
 * \brief Test client that superposes joint torques with sine waves.
 */
class LBRTorqueSineOverlayClient : public LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    * 
    * @param jointMask Bit mask that encodes the joint indices to be overlaid by sine waves
    * @param freqHz Sine frequency in Hertz
    * @param torqueAmplitude Sine amplitude in Nm
    */
   LBRTorqueSineOverlayClient(unsigned int jointMask, double freqHz, 
         double torqueAmplitude);
   
   /** 
    * \brief Destructor.
    */
   ~LBRTorqueSineOverlayClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
   /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    */
   virtual void waitForCommand();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
      
private:
      
   int _jointMask;         //!< bit mask encoding of overlay joints
   double _freqHz;         //!< sine frequency (Hertz)
   double _torqueAmpl;     //!< sine amplitude (Nm)
   double _phi;            //!< phase of sine wave
   double _stepWidth;      //!< stepwidth for sine 
   double _torques[LBRState::NUMBER_OF_JOINTS]; //!< commanded superposed torques
   
};

#endif // _KUKA_FRI_LBR_TORQUE_SINE_OVERLAY_CLIENT_H
