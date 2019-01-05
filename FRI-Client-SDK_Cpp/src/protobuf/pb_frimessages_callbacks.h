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
#ifndef _pb_frimessages_callbacks_H
#define _pb_frimessages_callbacks_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pb.h"


/** container for repeated double elements */
typedef struct repeatedDoubleArguments {
   size_t size;
   size_t max_size;
   double* value;
} tRepeatedDoubleArguments;

/** container for repeated integer elements */
typedef struct repeatedIntArguments {
   size_t size;
   size_t max_size;
   int64_t* value;
} tRepeatedIntArguments;

/** enumeration for direction (encoding/decoding) */
typedef enum DIRECTION {
   FRI_MANAGER_NANOPB_DECODE = 0, //!< Argument um eine 
   FRI_MANAGER_NANOPB_ENCODE = 1  //!< 
} eNanopbCallbackDirection;


bool encode_repeatedDouble(pb_ostream_t *stream, const pb_field_t *field,
      void * const *arg);

bool decode_repeatedDouble(pb_istream_t *stream, const pb_field_t *field,
      void **arg);

bool encode_repeatedInt(pb_ostream_t *stream, const pb_field_t *field,
      void * const *arg);

bool decode_repeatedInt(pb_istream_t *stream, const pb_field_t *field,
      void **arg);

void map_repeatedDouble(eNanopbCallbackDirection dir, int numDOF,
      pb_callback_t *values, tRepeatedDoubleArguments *arg);

void map_repeatedInt(eNanopbCallbackDirection dir, int numDOF,
      pb_callback_t *values, tRepeatedIntArguments *arg);

void init_repeatedDouble(tRepeatedDoubleArguments *arg);

void init_repeatedInt(tRepeatedIntArguments *arg);

void free_repeatedDouble(tRepeatedDoubleArguments *arg);

void free_repeatedInt(tRepeatedIntArguments *arg);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
