/**
 *   @file  sensor_mgmt.c
 *
 *   @brief
 *      The file implements the sensor management. Sensors can be started
 *      or stopped by either the GPIO Switch or through the CLI.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/utils/cli/cli.h>

/* Demo Include Files */
#include "mmw.h"
#include "data_path.h"

/**************************************************************************
 ******************** Sensor Mgmt Local Definitions ***********************
 **************************************************************************/

/**
 * @brief   Sensor Start Event
 */
#define MMWDEMO_CLI_SENSORSTART_EVT     Event_Id_00

/**
 * @brief   Sensor Stop Event
 */
#define MMWDEMO_CLI_SENSORSTOP_EVT      Event_Id_01

/**
 * @brief   Sensor Start Event without reconfig
 */
#define MMWDEMO_CLI_FRAMESTART_EVT      Event_Id_02

/**
 * @brief   Sensor start completed event
 */
#define MMWDEMO_START_COMPLETED_EVT      Event_Id_03

/**
 * @brief   Sensor start completed event
 */
#define MMWDEMO_STOP_COMPLETED_EVT      Event_Id_04

/**
 * @brief   Sensor start failed event
 */
#define MMWDEMO_START_FAILED_EVT        Event_Id_05



/**************************************************************************
 ******************** Sensor Mgmt Local Structures ************************
 **************************************************************************/

/**
 * @brief
 *  Millimeter Wave Demo Session Management MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo session management module
 */
typedef struct  MmwDemo_SensorMgmtMCB_t
{
    /**
     * @brief   Sensor Management Task Handle:
     */
    Task_Handle     sensorMgmtTaskHandle;

    /**
     * @brief   Sensor Management Event Handle:
     */
    Event_Handle    sensorMgmtEventHandle;

    /**
     * @brief   Sensor Management Event Notify Handle:
     */
    Event_Handle    eventHandleNotify;

    /**
     * @brief   Tracks the status of the sensor
     */
    bool            isSensorOpened;

    /**
     * @brief   Tracks the status of the sensor
     */
    bool            isSensorStarted;

    /**
     * @brief   Tracks the number of sensor start
     */
    uint32_t        sensorStartCount;

    /**
     * @brief   Tracks the number of sensor sop
     */
    uint32_t        sensorStopCount;

}MmwDemo_SensorMgmtMCB;

/**************************************************************************
 ************************ Sensor Mgmt Globals *****************************
 **************************************************************************/

/**
 * @brief   Global variable which is used to keep relevant information used
 * by the Session Management module.
 */
MmwDemo_SensorMgmtMCB  gMmwSessionMgmtMCB;


/**************************************************************************
 ************************ Sensor Mgmt Functions ***************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Callback function invoked when the GPIO switch is pressed.
 *      This is invoked from interrupt context.
 *
 *  @param[in]  index
 *      GPIO index configured as input
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_switchPressFxn(unsigned int index)
{
    /* Was the sensor started? */
    if (gMmwSessionMgmtMCB.isSensorStarted == true)
    {
        /* YES: We need to stop the sensor now */
        MmwDemo_notifySensorStop();
    }
    else
    {
        /* NO: We need to start the sensor now. */
        MmwDemo_notifySensorStart(true);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the sensor management task which is used to start & stop
 *      the sensor. Sensors can be controlled via the CLI or the switch
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_sensorMgmtTask (UArg arg0, UArg arg1)
{
    int32_t        errCode;
    UInt           event;
    int32_t        stopStatus;

    /* GPIO Input: Configure pin N7 as GPIO_1 input */
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN7_PADAC, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN7_PADAC, SOC_XWR14XX_PINN7_PADAC_GPIO_1);

    /* GPIO Output: Configure pin N13 as GPIO_2 output */
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN13_PADAZ, SOC_XWR14XX_PINN13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the SW1 switch on the EVM connected to GPIO_1
     * - This is used as an input
     * - Enable interrupt to be notified on a switch press
     **********************************************************************/
    GPIO_setConfig (SOC_XWR14XX_GPIO_1, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING | GPIO_CFG_IN_INT_LOW);
    GPIO_setCallback (SOC_XWR14XX_GPIO_1, MmwDemo_switchPressFxn);
    GPIO_enableInt (SOC_XWR14XX_GPIO_1);

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (SOC_XWR14XX_GPIO_2, GPIO_CFG_OUTPUT);

    /* Start the Task: */
    while (1)
    {
        /* Wait for a sensor event */
        event = Event_pend(gMmwSessionMgmtMCB.sensorMgmtEventHandle, Event_Id_NONE,
                           MMWDEMO_CLI_SENSORSTART_EVT | MMWDEMO_CLI_SENSORSTOP_EVT | MMWDEMO_CLI_FRAMESTART_EVT,
                           BIOS_WAIT_FOREVER);

        /************************************************************************
         * Sensor Start Event:
         ************************************************************************/
        if (event & MMWDEMO_CLI_SENSORSTART_EVT)
        {
            /*  Open mmWave module, this is only done once */
            if (gMmwSessionMgmtMCB.isSensorOpened == false)
            {
                /* Get the open configuration: */
                CLI_getMMWaveExtensionOpenConfig (&gMmwMCB.cfg.openCfg);

                /* Setup the calibration frequency: */
                gMmwMCB.cfg.openCfg.freqLimitLow  = 760U;
                gMmwMCB.cfg.openCfg.freqLimitHigh = 810U;

                /* Open the mmWave module: */
                if (MMWave_open (gMmwMCB.ctrlHandle, &gMmwMCB.cfg.openCfg, &errCode) < 0)
                {
                    System_printf ("Error: mmWave Open failed %d\n", errCode);
                    /* Post event that start failed */
                    Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_FAILED_EVT);
                    continue;
                }

                /* mmWave module has been opened. */
                gMmwSessionMgmtMCB.isSensorOpened = true;
            }
            else
            {
                /* openCfg related configurations like chCfg, lowPowerMode, adcCfg
                 * are only used on the first sensor start. If they are different
                 * on a subsequent sensor start, then generate a fatal error
                 * so the user does not think that the new (changed) configuration
                 * takes effect, the board needs to be reboot for the new
                 * configuration to be applied.
                 */
                MMWave_OpenCfg openCfg;

                CLI_getMMWaveExtensionOpenConfig (&openCfg);

                /* initialize to same as in "if" part where open is done
                 * to allow memory compare of structures to be used.
                 * Note that even if structures may have holes, the memory
                 * compare is o.k because CLI always stores the configurations
                 * in the same global CLI structure which is copied over to the
                 * one supplied by the application through the
                 * CLI_getMMWaveExtensionOpenConfig API. Not using memcmp will
                 * require individual field comparisons which is probably
                 * more code size and cumbersome.
                 */
                openCfg.freqLimitLow  = 760U;
                openCfg.freqLimitHigh = 810U;

                /* Compare openCfg */
                if(memcmp((void *)&gMmwMCB.cfg.openCfg, (void *)&openCfg,
                                  sizeof(MMWave_OpenCfg)) != 0)
                {
                    /* Post event that start failed */
                    Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_FAILED_EVT);

                    MmwDemo_debugAssert(0);
                }
            }
            
            if (gMmwSessionMgmtMCB.isSensorStarted == false)
            {
                gMmwSessionMgmtMCB.sensorStartCount++;

                /* Get the configuration from the CLI mmWave Extension */
                CLI_getMMWaveExtensionConfig (&gMmwMCB.cfg.ctrlCfg);

                /* Configure the mmWave module: */
                if (MMWave_config (gMmwMCB.ctrlHandle, &gMmwMCB.cfg.ctrlCfg, &errCode) < 0)
                {
                    /* Error: Report the error */
                    System_printf ("Error: mmWave Control failed [Error code %d]\n", errCode);

                    /* Post event that start failed */
                    Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_FAILED_EVT);
                }
                else
                {
                    /* Setup the data path: */
                    MmwDemo_dataPathConfig ();
                    if (MmwDemo_dataPathStart()==0)
                    {
                        /* The sensor has been started successfully. Switch on the LED */
                        GPIO_write (SOC_XWR14XX_GPIO_2, 1U);

                        /* Sensor has been started successfully: */
                        gMmwSessionMgmtMCB.isSensorStarted = true;

                        /* Post event that start is done */
                        Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_COMPLETED_EVT);
                    }
                    else
                    {
                        /* Post event that start failed */
                        Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_FAILED_EVT);
                    }
                }
            }
            else
            {
                /* Ignore duplicate command */
                Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_COMPLETED_EVT);
            }
            
        }

        /************************************************************************
         * Sensor Start Event:
         ************************************************************************/
        if (event & MMWDEMO_CLI_FRAMESTART_EVT)
        {

            if (gMmwSessionMgmtMCB.isSensorStarted == false)
            {
                gMmwSessionMgmtMCB.sensorStartCount++;
                if (MmwDemo_dataPathStart()==0)
                {
                    /* The sensor has been started successfully. Switch on the LED */
                    GPIO_write (SOC_XWR14XX_GPIO_2, 1U);

                    /* Sensor has been started successfully: */
                    gMmwSessionMgmtMCB.isSensorStarted = true;

                    /* Post event that start is done */
                    Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_COMPLETED_EVT);
                }
                else
                {
                    /* Post event that start failed */
                    Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_FAILED_EVT);
                }
            }
            else
            {
                /* Ignore duplicate command */
                Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_START_COMPLETED_EVT);
            }
        }

        /************************************************************************
         * Sensor Stop Event:
         ************************************************************************/
        if (event & MMWDEMO_CLI_SENSORSTOP_EVT)
        {
            if (gMmwSessionMgmtMCB.isSensorStarted == true)
            {
                /* Stop the mmWave module: */
                stopStatus = 0;
                if (MMWave_stop (gMmwMCB.ctrlHandle, &errCode) < 0)
                {
                    MMWave_ErrorLevel   errorLevel;
                    int16_t             mmWaveErrorCode;
                    int16_t             subsysErrorCode;

                    /* Error/Warning: Unable to stop the mmWave module */
                    MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
                    if (errorLevel == MMWave_ErrorLevel_ERROR)
                    {
                        /* Error: Display the error message: */
                        System_printf ("mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                                        mmWaveErrorCode, subsysErrorCode);

                        /* Set the status of the stop. */
                        stopStatus = -1;
                    }
                    else
                    {
                        /* Warning: This is treated as a successful stop. */
                    }
                }
                if (stopStatus == 0)
                {
                    /* The sensor has been stopped successfully. Switch off the LED */
                    GPIO_write (SOC_XWR14XX_GPIO_2, 0U);

                    /* Sensor has been stopped successfully: */
                    gMmwSessionMgmtMCB.isSensorStarted = false;

                    gMmwSessionMgmtMCB.sensorStopCount++;
                    /* print for user */
                    System_printf("Sensor has been stopped: start:%d stop%d\n",
                                    gMmwSessionMgmtMCB.sensorStartCount,gMmwSessionMgmtMCB.sensorStopCount);
                }
            }
            /* Post event that stop is done */
            Event_post(gMmwSessionMgmtMCB.eventHandleNotify, MMWDEMO_STOP_COMPLETED_EVT);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_notifySensorStart(bool doReconfig)
{
    if (doReconfig) {
        Event_post (gMmwSessionMgmtMCB.sensorMgmtEventHandle, MMWDEMO_CLI_SENSORSTART_EVT);
    }
    else {
        Event_post (gMmwSessionMgmtMCB.sensorMgmtEventHandle, MMWDEMO_CLI_FRAMESTART_EVT);
    }
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_notifySensorStop(void)
{
    Event_post (gMmwSessionMgmtMCB.sensorMgmtEventHandle, MMWDEMO_CLI_SENSORSTOP_EVT);
}


/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to
 *      pend for start complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
int32_t MmwDemo_waitSensorStartComplete(void)
{
    UInt          event;
    int32_t       retVal;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gMmwSessionMgmtMCB.eventHandleNotify,
                          Event_Id_NONE,
                          MMWDEMO_START_COMPLETED_EVT|MMWDEMO_START_FAILED_EVT,
                          BIOS_WAIT_FOREVER);

    /************************************************************************
     * DSS event:: START notification
     ************************************************************************/
    if(event & MMWDEMO_START_COMPLETED_EVT)
    {
        /* nothing much to do here */
        retVal=0;
    }
    else if(event & MMWDEMO_START_FAILED_EVT)
    {
        /* nothing much to do here */
        retVal = -1;
    }
    else 
    {
        /* we should block forever till we get the event. If the desired event
           didnt happen, then throw an assert */
        retVal = -1;
        MmwDemo_debugAssert(0);
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to
 *      pend for stop complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_waitSensorStopComplete(void)
{
    UInt          event;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gMmwSessionMgmtMCB.eventHandleNotify,
                          Event_Id_NONE,
                          MMWDEMO_STOP_COMPLETED_EVT,
                          BIOS_WAIT_FOREVER);

    /************************************************************************
     * STOP notification
     ************************************************************************/
    if(event & MMWDEMO_STOP_COMPLETED_EVT)
    {
        /* do nothing */
    }
    else {
        /* we should block forever till we get the event. If the desired event
           didnt happen, then throw an assert */
        MmwDemo_debugAssert(0);
    }
}


/**
 *  @b Description
 *  @n
 *      This is the function which is used to initialize and setup the
 *      sensor management module.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_sensorMgmtInit (void)
{
    Error_Block     eb;
    Task_Params     taskParams;

    /* Initialize the session management module: */
    memset ((void *)&gMmwSessionMgmtMCB, 0, sizeof(MmwDemo_SensorMgmtMCB));

    /* Setup the event handle used by the sensor management task: */
    Error_init(&eb);
    gMmwSessionMgmtMCB.sensorMgmtEventHandle = Event_create(NULL, &eb);
    if (gMmwSessionMgmtMCB.sensorMgmtEventHandle == NULL)
    {
        MmwDemo_debugAssert(0);
        return -1;
    }

    Error_init(&eb);
    gMmwSessionMgmtMCB.eventHandleNotify = Event_create(NULL, &eb);
    if (gMmwSessionMgmtMCB.eventHandleNotify == NULL)
    {
        MmwDemo_debugAssert(0);
        return -1;
    }


    /* Sensor is not started */
    gMmwSessionMgmtMCB.isSensorStarted = false;

    /* Launch the Session Management Task: */
    Task_Params_init(&taskParams);
    gMmwSessionMgmtMCB.sensorMgmtTaskHandle = Task_create(MmwDemo_sensorMgmtTask, &taskParams, NULL);
    if (gMmwSessionMgmtMCB.sensorMgmtTaskHandle == NULL)
    {
        MmwDemo_debugAssert(0);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the function which is used to deinitialize and shutdown the
 *      sensor management module
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_sensorMgmtDeinit (void)
{
    /* Delete the Task: */
    if (gMmwSessionMgmtMCB.sensorMgmtTaskHandle != NULL)
    {
        Task_delete (&gMmwSessionMgmtMCB.sensorMgmtTaskHandle);
    }

    /* Delete the Event: */
    if (gMmwSessionMgmtMCB.sensorMgmtEventHandle != NULL)
    {
        Event_delete (&gMmwSessionMgmtMCB.sensorMgmtEventHandle);
    }

    if (gMmwSessionMgmtMCB.eventHandleNotify != NULL)
    {
        Event_delete (&gMmwSessionMgmtMCB.eventHandleNotify);
    }
    return 0;
}

