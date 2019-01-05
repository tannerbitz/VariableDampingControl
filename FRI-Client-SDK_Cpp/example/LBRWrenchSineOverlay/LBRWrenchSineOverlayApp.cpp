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
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // strstr
#include "LBRWrenchSineOverlayClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

using namespace KUKA::FRI;


#define DEFAULT_PORTID 30200
#define DEFAULT_FREQUENCY 0.25
#define DEFAULT_AMPLITUDE 5.0



int main (int argc, char** argv)
{
   // parse command line arguments
   if (argc > 1)
   {
	   if ( strstr (argv[1],"help") != NULL)
	   {
	      printf(
	            "\nKUKA LBR wrench sine overlay test application\n\n"
	            "\tCommand line arguments:\n"
	            "\t1) remote hostname (optional)\n"
	            "\t2) port ID (optional)\n"
	            "\t3) sine frequency in Hertz (for Fx) (optional)\n"
	            "\t4) sine amplitude in radians (for Fx) (optional)\n"
	            "\t5) sine frequency in Hertz (for Fy) (optional)\n"
	            "\t6) sine amplitude in radians (for Fy) (optional)\n"
	      );
	      return 1;
	   }
   }
   char* hostname = (argc >= 2) ? argv[1] : NULL;
   int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORTID;
   double frequencyX = (argc >= 4) ? atof(argv[3]) : DEFAULT_FREQUENCY;
   double amplitudeX = (argc >= 5) ? atof(argv[4]) : DEFAULT_AMPLITUDE;
   double frequencyY = (argc >= 6) ? atof(argv[5]) : DEFAULT_FREQUENCY;
   double amplitudeY = (argc >= 7) ? atof(argv[6]) : DEFAULT_AMPLITUDE;
   
   printf("Enter LBRWrenchSineOverlay Client Application\n");

   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /**************************************************************************/
   
   // create new sine overlay client
   LBRWrenchSineOverlayClient client(frequencyX, frequencyY, amplitudeX, amplitudeY);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Configuration                                                         */
   /*                                                                         */
   /***************************************************************************/

   // create new udp connection
   UdpConnection connection;


   // pass connection and client to a new FRI client application
   ClientApplication app(connection, client);
   
   // connect client application to KUKA Sunrise controller
   app.connect(port, hostname);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Execution mainloop                                                    */
   /*                                                                         */
   /***************************************************************************/

   // repeatedly call the step routine to receive and process FRI packets
   bool success = true;
   while (success)
   {
      success = app.step();
   }

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/

   // disconnect from controller
   app.disconnect();
   
   printf("Exit LBRWrenchSineOverlay Client Application\n");
   
   return 1;
}
