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
#ifndef _KUKA_FRI_UDP_CONNECTION_H
#define _KUKA_FRI_UDP_CONNECTION_H

#include <stdlib.h>

#ifdef _MSC_VER
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#ifdef VXWORKS // VxWorks Kernel
#include <sockLib.h>
#endif

#include "friConnectionIf.h"


namespace KUKA
{
namespace FRI
{

   /**
    * \brief This class implements the IConnection interface using UDP sockets.
    */
   class UdpConnection : public IConnection
   {
   
   public:
   
      /** \brief Constructor. */
      UdpConnection();
   
      /** \brief Destructor. */
      ~UdpConnection();
   
      /**
       * \brief Open a connection to the KUKA Sunrise controller.
       * 
       * @param port The port ID
       * @param remoteHost The address of the remote host
       * @return True if connection was established
       */
      virtual bool open(int port, const char *remoteHost = NULL);
   
      /**
       * \brief Close a connection to the KUKA Sunrise controller.
       */
      virtual void close();
      
      /**
       * \brief Checks whether a connection to the KUKA Sunrise controller is established.
       * 
       * @return True if connection is established
       */
      virtual bool isOpen() const;
   
      /**
       * \brief Receive a new FRI monitoring message from the KUKA Sunrise controller. 
       * 
       * This method blocks until a new message arrives.
       * @param buffer Pointer to the allocated buffer that will hold the FRI message
       * @param maxSize Size in bytes of the allocated buffer
       * @return Number of bytes received (0 when connection was terminated, negative in case of errors)
       */
      virtual int receive(char *buffer, int maxSize);
   
      /**
       * \brief Send a new FRI command message to the KUKA Sunrise controller.
       * 
       * @param buffer Pointer to the buffer holding the FRI message
       * @param size Size in bytes of the message to be send
       * @return True if successful
       */
      virtual bool send(const char* buffer, int size);
   
   private:
   
      int _udpSock;                          //!< UDP socket handle
      struct sockaddr_in _controllerAddr;    //!< the controller's socket address
   
   };
   
}
}


#endif // _KUKA_FRI_UDP_CONNECTION_H
