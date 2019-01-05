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
#ifndef _KUKA_FRI_LBR_WRENCH_SINE_OVERLAY_CLIENT_H
#define _KUKA_FRI_LBR_WRENCH_SINE_OVERLAY_CLIENT_H

#include "friLBRClient.h"

using namespace KUKA::FRI;

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class LBRWrenchSineOverlayClient : public LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    * 
    * @param freqHzX Sine frequency in hertz of force in X-direction
    * @param freqHzY Sine frequency in hertz of force in Y-direction
    * @param amplRadX Sine amplitude in radians of force in X-direction
    * @param amplRadY Sine amplitude in radians of force in Y-direction
    */
   LBRWrenchSineOverlayClient(double freqHzX, double freqHzY, 
         double amplRadX, double amplRadY);
   
   /** 
    * \brief Destructor.
    */
   ~LBRWrenchSineOverlayClient();
   
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
   static const int CART_VECTOR_DIM = 6; //!< number of elements in a Cartesian vector
   
   double _freqHzX;        //!< sine frequency x-direction (Hertz)
   double _freqHzY;        //!< sine frequency y-direction (Hertz)
   double _amplRadX;       //!< sine amplitude x-direction (radians)
   double _amplRadY;       //!< sine amplitude y-direction (radians)
   double _wrench[CART_VECTOR_DIM];      //!< commanded wrench
   double _stepWidthX;     //!< stepwidth for sine in x-direction
   double _stepWidthY;     //!< stepwidth for sine in y-direction
   double _phiX;             //!< current phase for sine in x-direction
   double _phiY;             //!< current phase for sine in y-direction
   
};

#endif // _KUKA_FRI_LBR_WRENCH_SINE_OVERLAY_CLIENT_H
