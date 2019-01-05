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
#ifndef _KUKA_FRI_LBR_COMMAND_H
#define _KUKA_FRI_LBR_COMMAND_H


// forward declarations
typedef struct _FRICommandMessage FRICommandMessage;


namespace KUKA
{
namespace FRI
{
   
   /**
    * \brief Wrapper class for the FRI command message for a KUKA LBR (leightweight) robot.
    */
   class LBRCommand
   {
      friend class LBRClient;
   
   public:
            
      /**
       * \brief Set the joint positions for the current interpolation step.
       * 
       * This method is only effective when the client is in a commanding state.
       * @param values Array with the new joint positions (in rad)
       */
      void setJointPosition(const double* values);
      
      /**
       * \brief Set the applied wrench vector of the current interpolation step.
       * 
       * The wrench vector consists of:
       * [F_x, F_y, F_z, tau_A, tau_B, tau_C]
       * 
       * F ... forces (in N) applied along the Cartesian axes of the 
       * currently used motion center.
       * tau ... torques (in Nm) applied along the orientation angles 
       * (Euler angles A, B, C) of the currently used motion center.
       *  
       * This method is only effective when the client is in a commanding state.
       * The ControlMode of the robot has to be Cartesian impedance control mode. The
       * Client Command Mode has to be wrench.
       * 
       * @param wrench Applied Cartesian wrench vector.
       */
      void setWrench(const double* wrench);

      /**
       * \brief Set the applied joint torques for the current interpolation step.
       * 
       * This method is only effective when the client is in a commanding state.
       * The ControlMode of the robot has to be joint impedance control mode. The
       * Client Command Mode has to be torque.
       * 
       * @param torques Array with the applied torque values (in Nm)
       */
      void setTorque(const double* torques);
      
   protected:
            
      static const int LBRCOMMANDMESSAGEID = 0x34001; //!< type identifier for the FRI command message corresponding to a KUKA LBR robot 
      FRICommandMessage* _message;                    //!< FRI command message (protobuf struct)
      
   };

}
}


#endif // _KUKA_FRI_LBR_COMMAND_H
