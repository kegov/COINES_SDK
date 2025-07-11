/**
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  @file   bridge_decoder.c
 *  @brief  This module defines decoder APIs to be used by the api layer
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/

/*********************************************************************/
/* global variables */
/*********************************************************************/
FILE *log_file = NULL; // File pointer for logging
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;

/*! Buffer to hold the formatted date-time string */
static char datetime_str[64];

/*! Holds the current time as a time_t value */
static time_t system_time;

/*! Pointer to a tm structure holding the decomposed time */
static struct tm *tm_local;
/*********************************************************************/
/* extern variables */
/*********************************************************************/

/*********************************************************************/
/* static function declarations */
/*********************************************************************/


/*********************************************************************/
/* static functions */
/*********************************************************************/

/*********************************************************************/
/* functions */
/*********************************************************************/
void log_start(void)
{
    (void)time(&system_time);
    tm_local = localtime(&system_time);
    
    /* Format the current date and time as a string ("YYYY-MM-DD HH-MM-SS") */
    (void)strftime(datetime_str, sizeof(datetime_str),  LOG_PATH "CB_log_%Y-%m-%d_%H-%M-%S.txt", tm_local);
    
    // Open the log file with the generated file name
    log_file = fopen(datetime_str, "w");
    if (log_file == NULL)
    {
        printf("Error opening log file: %s\n", datetime_str);
        exit(EXIT_FAILURE);
    }
}
/**
 * @brief This function prints a byte array in hexadecimal format separated by hyphens.
 *
 * @param[in] label  Label to print before the hex message.
 * @param[in] bytes  Pointer to the byte array.
 * @param[in] length Length of the byte array.
 */
void log_message(const char *label, const uint8_t *bytes, size_t length)
{
    pthread_mutex_lock(&log_mutex);
    fprintf(log_file, "%s", label);
    for (size_t i = 0; i < length; i++)
    {
        if (i > 0)
        {
            fprintf(log_file, "-");
        }
        fprintf(log_file, "%02X", bytes[i]);
    }
    fprintf(log_file, "\n");
    fflush(log_file);
    pthread_mutex_unlock(&log_mutex);
}

void log_stop(void)
{
    fclose(log_file);
}

/*********************************************************************/
/* DLL entry point */
/*********************************************************************/
BOOL APIENTRY DllMain(HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
        log_start();
        break;
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
        break;
    case DLL_PROCESS_DETACH:
        log_stop();
        break;
    }
    return TRUE;
}