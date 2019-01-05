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
#include <string.h>
#include <stdio.h>
#ifndef WIN32
#include <unistd.h>
#endif
#include "friUdpConnection.h"

#ifdef WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib") 
#endif


using namespace KUKA::FRI;

//******************************************************************************
UdpConnection::UdpConnection() : _udpSock(-1)
{
#ifdef WIN32
   WSADATA WSAData;
   WSAStartup(MAKEWORD(2,0), &WSAData);
#endif
}
   
//******************************************************************************
UdpConnection::~UdpConnection()
{
   close();
#ifdef WIN32
   WSACleanup();
#endif
}
   
//******************************************************************************
bool UdpConnection::open(int port, const char *remoteHost)
{
   struct sockaddr_in servAddr;
   memset(&servAddr, 0, sizeof(servAddr));
   memset(&_controllerAddr, 0, sizeof(_controllerAddr));

   // socket creation
   _udpSock = socket(PF_INET, SOCK_DGRAM, 0);
   if (_udpSock < 0)
   {
      printf("opening socket failed!\n");
      return false;
   }
   // bind local server port
   servAddr.sin_family = AF_INET;
   servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
   servAddr.sin_port = htons(port);

   if (bind(_udpSock, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
   {
      printf("binding port number %d failed!\n", port);
      close();
      return false;
   }
   // if the remote host is specified, preinitialize the socket properly
   _controllerAddr.sin_family = AF_INET;
   _controllerAddr.sin_port = htons(port);
   if (remoteHost)
   {
      printf("preinitialized remote host to %s\n", remoteHost);
      _controllerAddr.sin_addr.s_addr = inet_addr(remoteHost);
   }
   return true;
}
   
//******************************************************************************
void UdpConnection::close()
{
   if (isOpen())
   {
#ifdef WIN32
      closesocket(_udpSock);
#else
      ::close(_udpSock);
#endif
   }
   _udpSock = -1;
}

//******************************************************************************
bool UdpConnection::isOpen() const
{
   return (_udpSock >= 0);
}
   
//******************************************************************************
int UdpConnection::receive(char *buffer, int maxSize)
{
   if (isOpen())
   {
      /** HAVE_SOCKLEN_T
       Yes - unbelievable: There are differences in standard calling parameters (types) to recvfrom 
       Windows winsock, VxWorks and QNX use int
       newer Posix (most Linuxes) use socklen_t
       */
#ifdef HAVE_SOCKLEN_T
      socklen_t sockAddrSize;
#else
      int sockAddrSize;
#endif
      
      sockAddrSize = sizeof(struct sockaddr_in);
      
      return recvfrom(_udpSock, buffer, maxSize, 0, (struct sockaddr *)&_controllerAddr, &sockAddrSize);
   }
   return -1;
}
   
//******************************************************************************
bool UdpConnection::send(const char* buffer, int size)
{
   if ((isOpen()) && (ntohs(_controllerAddr.sin_port) != 0))
   {
      int sent = sendto(_udpSock, const_cast<char*>(buffer), size, 0, (struct sockaddr *)&_controllerAddr, sizeof(_controllerAddr));
      if (sent == size)
      {
         return true;
      }
   }
   return false;
}
