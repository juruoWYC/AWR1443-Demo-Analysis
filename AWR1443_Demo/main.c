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
#include <ti/sysbios/utils/Load.h>


/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/demo/io_interface/mmw_output.h>

#include "config_edma_util.h"
#include "config_hwa_util.h"
#include "post_processing.h"

/* Demo Include Files */
#include "mmw.h"
#include "data_path.h"
#include <ti/demo/io_interface/mmw_config.h>
#include <ti/demo/utils/rx_ch_bias_measure.h>

extern mmwHwaBuf_t gMmwHwaMemBuf[MMW_HWA_NUM_MEM_BUFS];

extern uint32_t log2Approx(uint32_t x);

/*! L3 RAM buffer */
uint8_t gMmwL3[SOC_XWR14XX_MSS_L3RAM_SIZE];               //8位整形数组，长度为L3RAM_SIZE
#pragma DATA_SECTION(gMmwL3, ".l3ram");                                //?

/*! L3 heap for convenience of partitioning L3 RAM */
MmwDemoMemPool_t gMmwL3heap =                                  //结构体，分别为8位指针，32位整形，32位整形
{
    &gMmwL3[0],
    SOC_XWR14XX_MSS_L3RAM_SIZE,
    0                                                    //indx = 0
};

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMCB;             //结构体，存储毫米波演示相关信息


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void MmwDemo_CLIInit (void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);            //关联debug

void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathConfig(void);
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj);

void MmwDemo_getAngleBinsAtPeak(uint32_t numObj,
                                     MmwDemo_detectedObj *objOut,           //范围指数，多普勒指数，峰值，角度x，y，z
                                     uint16_t *pAngleBins);

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,          //通过UART传输检测数据
                                    MmwDemo_DataPathObj *obj);

void MmwDemo_initTask(UArg arg0, UArg arg1);
void MmwDemo_dataPathTask(UArg arg0, UArg arg1);
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)     //expression为0时执行
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Get a handle for ADCBuf.
 */
void MmwDemo_ADCBufOpen(MmwDemo_DataPathObj *obj)                  //ADC缓冲器
{
    ADCBuf_Params       ADCBufparams;
    /*****************************************************************************
     * Start ADCBUF driver:
     *****************************************************************************/
    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThreshold = 1;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
    obj->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (obj->adcbufHandle == NULL)
    {
        //System_printf("Error: Unable to open the ADCBUF driver\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: ADCBUF Instance(0) %p has been opened successfully\n", obj->adcbufHandle);
}



/**
 *  @b 描述
 *  @n
 *      配置ADC缓冲器，返回Rx天线数量
 */
int32_t MmwDemo_ADCBufConfig(MmwDemo_DataPathObj *dataPathObj)
{

    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    uint8_t             channel;
    int32_t             retVal = 0;                    //返回值
    uint8_t             numBytePerSample = 0;
    MmwDemo_ADCBufCfg*  ptrAdcbufCfg;
    uint32_t            chirpThreshold;               //啁啾入口
    uint32_t            rxChanMask = 0xF;

    ptrAdcbufCfg = &dataPathObj->cliCfg->adcBufCfg;

    /*****************************************************************************
     * Disable all ADCBuf channels
     *****************************************************************************/
    if ((retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       //System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       MmwDemo_debugAssert (0);
       goto exit;
    }

    /* Calculate the DMA transfer parameters */
    if (ptrAdcbufCfg->adcFmt == 0)
    {
        /* Complex dataFormat has 4 bytes */
        numBytePerSample =  4;
    }
    else
    {
        /* Real dataFormat has 2 bytes */
        numBytePerSample =  2;
    }

    /* Configure ADC buffer data format */
    dataFormat.adcOutFormat       = ptrAdcbufCfg->adcFmt;
    dataFormat.sampleInterleave   = ptrAdcbufCfg->iqSwapSel;
    dataFormat.channelInterleave  = ptrAdcbufCfg->chInterleave;

    /* Debug Message: */
    /*System_printf("Debug: Start ADCBuf driver dataFormat=%d, sampleSwap=%d, interleave=%d, chirpThreshold=%d\n",
                   dataFormat.adcOutFormat, dataFormat.sampleInterleave, dataFormat.channelInterleave,
                   ptrAdcbufCfg->chirpThreshold);*/

    retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

    memset((void*)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    /* 启用通道配置中指示的Rx通道 */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(gMmwMCB.cfg.openCfg.chCfg.rxChannelEn & (0x1<<channel))
        {
            /* 填充接收通道配置: */
            rxChanConf.channel = channel;
            retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                MmwDemo_debugAssert (0);
                goto exit;
            }
            rxChanConf.offset  += dataPathObj->numAdcSamples * numBytePerSample;
        }
    }

    chirpThreshold = ptrAdcbufCfg->chirpThreshold;

    /* 设置啁啾阈值: */
    retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        MmwDemo_debugAssert (0);
    }

exit:
    return retVal;
}


/**
 *  @b 描述
 *  @n
 *      解析Profile，Chirp和Frame配置并提取处理链配置所需的参数
 */
bool MmwDemo_parseProfileAndChirpConfig(MmwDemo_DataPathObj *dataPathObj)
{
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    int32_t     errCode;
    uint32_t    profileLoopIdx, chirpLoopIdx;
    bool        foundValidProfile = false;
    uint16_t    channelTxEn = gMmwMCB.cfg.openCfg.chCfg.txChannelEn;
    uint8_t     channel;
    uint8_t     numRxAntennas = 0;
    uint8_t     rxAntOrder [SYS_COMMON_NUM_RX_CHANNEL];
    uint8_t     txAntOrder [SYS_COMMON_NUM_TX_ANTENNAS];
    int32_t     i;
    int32_t     txIdx, rxIdx;

    /* Find number of enabled channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        rxAntOrder[channel] = 0;
        if(gMmwMCB.cfg.openCfg.chCfg.rxChannelEn & (0x1<<channel))
        {
            rxAntOrder[numRxAntennas] = channel;
            /* Track the number of receive channels: */
            numRxAntennas++;
        }
    }
    dataPathObj->numRxAntennas = numRxAntennas;

    /* read frameCfg chirp start/stop*/
    frameChirpStartIdx = gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpStartIdx;
    frameChirpEndIdx = gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpEndIdx;
    frameTotalChirps = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    for (profileLoopIdx=0;
        ((profileLoopIdx<MMWAVE_MAX_PROFILE)&&(foundValidProfile==false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        bool        validProfileHasElevation=false;
        bool        validProfileHasOneTxPerChirp=false;
        uint16_t    validProfileTxEn = 0;
        uint16_t    validChirpTxEnBits[32]={0};
        MMWave_ProfileHandle profileHandle;

        profileHandle = gMmwMCB.cfg.ctrlCfg.u.frameCfg.profileHandle[profileLoopIdx];
        if (profileHandle == NULL)
            continue; /* skip this profile */

        /* get numChirps for this profile; skip error checking */
        MMWave_getNumChirps(profileHandle,&mmWaveNumChirps,&errCode);
        /* loop for chirps and find if it has valid chirps for the frame
           looping around for all chirps in a profile, in case
           there are duplicate chirps
         */
        for (chirpLoopIdx=1;chirpLoopIdx<=mmWaveNumChirps;chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            /* get handle and read ChirpCfg */
            if (MMWave_getChirpHandle(profileHandle,chirpLoopIdx,&chirpHandle,&errCode)==0)
            {
                rlChirpCfg_t chirpCfg;
                if (MMWave_getChirpCfg(chirpHandle,&chirpCfg,&errCode)==0)
                {
                    uint16_t chirpTxEn = chirpCfg.txEnable;
                    /* do chirps fall in range and has valid antenna enabled */
                    if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                        (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
                        ((chirpTxEn & channelTxEn) > 0))
                    {
                        uint16_t idx = 0;
                        for (idx=(chirpCfg.chirpStartIdx-frameChirpStartIdx);idx<=(chirpCfg.chirpEndIdx-frameChirpStartIdx);idx++)
                        {
                            validChirpTxEnBits[idx] = chirpTxEn;
                            foundValidProfile = true;
                        }

                    }
                }
            }
        }
        /* now loop through unique chirps and check if we found all of the ones
           needed for the frame and then determine the azimuth/elevation antenna
           configuration
         */
        if (foundValidProfile) {
            int16_t nonElevFirstChirpIdx = -1;
            for (chirpLoopIdx=0;chirpLoopIdx<frameTotalChirps;chirpLoopIdx++)
            {
                bool validChirpHasElevation=false;
                bool validChirpHasOneTxPerChirp=false;
                uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
                if (chirpTxEn == 0) {
                    /* this profile doesnt have all the needed chirps */
                    foundValidProfile = false;
                    break;
                }
                /* check if this is an elevation TX chirp */
                validChirpHasElevation = (chirpTxEn==0x2);
                validProfileHasElevation |= validChirpHasElevation;
                /* if not, then check the MIMO config */
                if (!validChirpHasElevation)
                {
                    validChirpHasOneTxPerChirp = ((chirpTxEn==0x1) || (chirpTxEn==0x4));
                    /* if this is the first chirp without elevation, record the chirp's
                       MIMO config as profile's MIMO config. We dont handle intermix
                       at this point */
                    if (nonElevFirstChirpIdx==-1) {
                        validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                        nonElevFirstChirpIdx = chirpLoopIdx;
                    }
                    /* check the chirp's MIMO config against Profile's MIMO config */
                    if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
                    {
                        /* this profile doesnt have all chirps with same MIMO config */
                        foundValidProfile = false;
                        break;
                    }
                }
                /* save the antennas actually enabled in this profile */
                validProfileTxEn |= chirpTxEn;
            }
        }

        /* found valid chirps for the frame; mark this profile valid */
        if (foundValidProfile==true) {
            rlProfileCfg_t  profileCfg;
            uint32_t        numTxAntAzim = 0;
            uint32_t        numTxAntElev = 0;

            dataPathObj->validProfileIdx = profileLoopIdx;
            dataPathObj->numTxAntennas = 0;
            if (validProfileHasElevation)
            {
                numTxAntElev = 1;
            }
            if (!validProfileHasOneTxPerChirp)
            {
                numTxAntAzim=1;
            }
            else
            {
                if (validProfileTxEn & 0x1)
                {
                    numTxAntAzim++;
                }
                if (validProfileTxEn & 0x4)
                {
                    numTxAntAzim++;
                }
            }
            /*System_printf("Azimuth Tx: %d (MIMO:%d), Elev Tx:%d\n",
                            numTxAntAzim,validProfileHasMIMO,numTxAntElev);*/
            dataPathObj->numTxAntennas = numTxAntAzim + numTxAntElev;
            dataPathObj->numVirtualAntAzim = numTxAntAzim * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntElev = numTxAntElev * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntennas = dataPathObj->numVirtualAntAzim + dataPathObj->numVirtualAntElev;

            /* Copy the Rx channel compensation coefficients from common area to data path structure */
            if (validProfileHasOneTxPerChirp)
            {
                for (i = 0; i < dataPathObj->numTxAntennas; i++)
                {
                    txAntOrder[i] = log2Approx(validChirpTxEnBits[i]);
                }
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] =
                                dataPathObj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[txAntOrder[txIdx]*SYS_COMMON_NUM_RX_CHANNEL + rxAntOrder[rxIdx]];

                    }

                }
            }
            else
            {
                cmplx16ImRe_t one;
                one.imag = 0;
                one.real = 0x7fff;
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] = one;
                    }

                }
            }

            /* Get the profile configuration: */
            if (MMWave_getProfileCfg(profileHandle,&profileCfg, &errCode) < 0)
            {
                MmwDemo_debugAssert(0);
                return false;
            }

#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                System_printf("Frequency slope must be positive\n");
                MmwDemo_debugAssert(0);
            }
#endif

            dataPathObj->numAdcSamples = profileCfg.numAdcSamples;
            dataPathObj->numRangeBins = MmwDemo_pow2roundup(dataPathObj->numAdcSamples);
            dataPathObj->numChirpsPerFrame = frameTotalChirps *
                                              gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numLoops;

            dataPathObj->numAngleBins = MMW_NUM_ANGLE_BINS;
            dataPathObj->numDopplerBins = dataPathObj->numChirpsPerFrame/dataPathObj->numTxAntennas;
            dataPathObj->numRangeBinsPerTransfer = MMW_NUM_RANGE_BINS_PER_TRANSFER;
            dataPathObj->rangeResolution = MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC * profileCfg.digOutSampleRate * 1e3 /
                    (2 * profileCfg.freqSlopeConst * ((3.6*1e3*900)/(1U << 26)) * 1e12 * dataPathObj->numRangeBins);

            dataPathObj->xyzOutputQFormat    = (uint32_t) ceil(log10(16./fabs(dataPathObj->rangeResolution))/log10(2));
            dataPathObj->dataPathMode = DATA_PATH_WITH_ADCBUF;
            dataPathObj->frameStartIntCounter = 0;
            dataPathObj->interFrameProcToken = 0;
        }
    }
    return foundValidProfile;
}

void MmwDemo_measurementResultOutput(MmwDemo_DataPathObj *obj)
{
    /* 通过CLI发送收到的DSS校准信息 */
    CLI_write ("compRangeBiasAndRxChanPhase");
    CLI_write (" %.7f", obj->cliCommonCfg->compRxChanCfg.rangeBias);
    int32_t i;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        CLI_write (" %.5f", (float) obj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[i].real/32768.);
        CLI_write (" %.5f", (float) obj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[i].imag/32768.);
    }
    CLI_write ("\n");

}

/** @概述 通过UART传送检测数据
*
*    下列数据被传输:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and icluding the number of TLV items
*    TLV Items:
*    2. If detectedObjects flag is set, pbjOut structure containing range,
*       Doppler, and X,Y,Z location for detected objects,
*       size = sizeof(objOut_t) * number of detected objects
*    3. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    4. If noiseProfile flag is set,  noiseProfile,
*       size = number of range bins * sizeof(uint16_t)
*    7. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    8. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*    9. If statsInfo flag is set, the stats information
*   @param[in] uartHandle   UART driver handle
*   @param[in] obj          Pointer data path object MmwDemo_DataPathObj
*/

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                    MmwDemo_DataPathObj *obj)
{
    MmwDemo_output_message_header header;
    MmwDemo_GuiMonSel   *pGuiMonSel;
    uint32_t tlvIdx = 0;
    uint32_t i;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];

    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];

    /*获取GUI监视器数据 */
    pGuiMonSel = &gMmwMCB.cliCfg.guiMonSel;

    /*清楚信息帧头*/
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
    /* 帧头: */
    header.platform = 0xA1443;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.numDetectedObj = obj->numObjOut;
    header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    if (pGuiMonSel->detectedObjects && (obj->numObjOut > 0))
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(MmwDemo_detectedObj) * obj->numObjOut +
                            sizeof(MmwDemo_output_message_dataObjDescr);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->logMagRange)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * obj->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->noiseProfile)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * obj->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP;
        tl[tlvIdx].length = obj->numRangeBins * obj->numVirtualAntAzim * sizeof(uint32_t);
        packetLen += sizeof(MmwDemo_output_message_tl) +  tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeDopplerHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        tl[tlvIdx].length = obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->statsInfo)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    /* 整理数据包长度为MMWDEMO_OUTPUT_MSG_SEGMENT_LEN倍 */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles =  Pmu_getCount(0);
    header.frameNumber = obj->frameStartIntCounter;


    UART_writePolling (uartHandle,
                       (uint8_t*)&header,
                       sizeof(MmwDemo_output_message_header));

    tlvIdx = 0;
    /* 发送检测对象 */
    if ((pGuiMonSel->detectedObjects == 1) && (obj->numObjOut > 0))
    {
        MmwDemo_output_message_dataObjDescr descr;

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        /* 发送对象描述符 */
        descr.numDetetedObj = (uint16_t) obj->numObjOut;
        descr.xyzQFormat = (uint16_t) obj->xyzOutputQFormat;
        UART_writePolling (uartHandle, (uint8_t*)&descr, sizeof(MmwDemo_output_message_dataObjDescr));

        /*发送对象数组 */
        UART_writePolling (uartHandle, (uint8_t*)obj->objOut, sizeof(MmwDemo_detectedObj) * obj->numObjOut);
        tlvIdx++;
    }

    /* 发送范围配置文件 */
    if (pGuiMonSel->logMagRange)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        for(i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t*)&obj->rangeDopplerLogMagMatrix[i*obj->numDopplerBins],
                    sizeof(uint16_t));
        }
        tlvIdx++;
    }

    /* 发送噪音配置文件 */
    if (pGuiMonSel->noiseProfile)
    {
        uint32_t maxDopIdx = obj->numDopplerBins/2 -1;
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        for(i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t*)&obj->rangeDopplerLogMagMatrix[i*obj->numDopplerBins + maxDopIdx],
                    sizeof(uint16_t));
        }
        tlvIdx++;
    }

    /* 发送静态方位热图的数据 */
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        uint32_t skip = obj->numChirpsPerFrame * obj->numRxAntennas;
        uint32_t copySize = obj->numVirtualAntAzim * sizeof(uint32_t);

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        for (i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t *)  &obj->radarCube[i * skip],
                    copySize);
        }
        tlvIdx++;
    }

    /* 发送范围/多普勒热图的数据 */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        UART_writePolling (uartHandle,
                (uint8_t*)obj->rangeDopplerLogMagMatrix,
                tl[tlvIdx].length);
        tlvIdx++;
    }

    /* 发送统计信息 */
    if (pGuiMonSel->statsInfo == 1)
    {
        MmwDemo_output_message_stats stats;
        stats.interChirpProcessingMargin = 0; /* 不可用 */
        stats.interFrameProcessingMargin = (uint32_t) (obj->timingInfo.interFrameProcessingEndMargin/R4F_CLOCK_MHZ); /* In micro seconds */
        stats.interFrameProcessingTime = (uint32_t) (obj->timingInfo.interFrameProcCycles/R4F_CLOCK_MHZ); /* In micro seconds */
        stats.transmitOutputTime = (uint32_t) (obj->timingInfo.transmitOutputCycles/R4F_CLOCK_MHZ); /* In micro seconds */
        stats.activeFrameCPULoad = obj->timingInfo.activeFrameCPULoad;
        stats.interFrameCPULoad = obj->timingInfo.interFrameCPULoad;

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        UART_writePolling (uartHandle,
                           (uint8_t*)&stats,
                           tl[tlvIdx].length);
        tlvIdx++;
    }

    /* 发送填充字节 */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }
}

/**
 *  @b 描述
 *  @n
 *      这个函数用来引发雷达前端产生啁啾。
 *
 *  @返回值
 *      不可用.
 */
int32_t MmwDemo_dataPathStart (void)
{
    MMWave_CalibrationCfg   calibrationCfg;           //校准
    int32_t                 errCode = 0;
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;

    dataPathObj->frameStartIntCounter = 0;
    dataPathObj->interFrameProcToken = 0;

    /* 初始化校准配置: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* 配置校准数据: */
    calibrationCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* 启动mmWave模块: 配置已成功应用. */
    if (MMWave_start (gMmwMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        /*错误: 无法启动mmWave控制 */
        //System_printf ("Error: mmWave Control Start failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
    }
    return errCode;
}

/**
 *  @b 描述
 *  @n
 *      这个函数被用来基于啁啾数据配置data path
 *      该函数执行之后，当ADC缓冲器开始收集与啁啾一致的样例时，data path过程将准备执行。
 *
 *  @返回值
 *      不可用.
 */
void MmwDemo_dataPathConfig (void)
{
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;

    /* Configure ADCBuf Config and get the valid number of RX antennas
       do this first as we need the numRxAntennas in MmwDemo_parseProfileAndChirpConfig
       to get the Virtual Antennas */
    /* Parse the profile and chirp configs and get the valid number of TX Antennas */
    if (MmwDemo_parseProfileAndChirpConfig(dataPathObj) == true)
    {

        if (MmwDemo_ADCBufConfig(dataPathObj) < 0)
        {
            //System_printf("Error: ADCBuf config failed \n");
            MmwDemo_debugAssert (0);
        }

        /* Now we are ready to allocate and config the data buffers */
        MmwDemo_dataPathCfgBuffers(dataPathObj, &gMmwL3heap);
        /* Configure one-time EDMA and HWA parameters */
        MmwDemo_dataPathConfigCommon(dataPathObj);
        /* Config HWA for 1D processing and keep it ready for immediate processingh
           as soon as Front End starts generating chirps */
        MmwDemo_config1D_HWA(dataPathObj);
        MmwDemo_dataPathTrigger1D(dataPathObj);
    }
    else
    {
        /* 没有找到有效profile - assert! */
        MmwDemo_debugAssert(0);
    }
    return;
}

/**
 *  @b 描述
 *  @n
 *  这个函数在初始化时，被@ref MmwDemo_initTask调用。
 *  它初始化这些驱动: ADCBUF, HWA, EDMA, 并且被@ref MmwDemo_dataPathTask用来传输信号。
 *
 *  @返回值
 *      不可用.
 */
void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj)                         //ADC,HWA,EDMA
{
    MmwDemo_dataPathObjInit(obj, &gMmwMCB.cliCfg, &gMmwMCB.cliCommonCfg);

    /* 初始化 ADCBUF */
    ADCBuf_init();

    /* 初始化 HWA */
    MmwDemo_hwaInit(obj);

    /* 初始化 EDMA */
    MmwDemo_edmaInit(obj);
}

void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj)                 //打开
{
    /*****************************************************************************
     * 启动 HWA, EDMA 和 ADCBUF drivers:
     *****************************************************************************/
    MmwDemo_hwaOpen(obj, gMmwMCB.socHandle);
    MmwDemo_edmaOpen(obj);
    MmwDemo_ADCBufOpen(obj);
}


/**
 *  @b 描述
 *  @n
 *      这个任务用来为mmWave控制进程提供执行环境。
 *
 *  @返回值
 *      不可用.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;       //错误代码

    while (1)
    {
        /* 执行mmWave控制模块: */
        if (MMWave_execute (gMmwMCB.ctrlHandle, &errCode) < 0)
        {
            //System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert (0);
        }    
    }
}

/**
 *  @b 描述
 *  @n
 *      为mmWave声明事件函数，该函数当接收到从BSS传来的事件时被调用。
 *
 *  @内部参数  msgId
 *      Message Identifier
 *  @内部参数  sbId
 *      Subblock identifier
 *  @内部参数  sbLen
 *      Length of the subblock
 *  @内部参数  payload
 *      Pointer to the payload buffer
 *
 *  @返回值
 *      总是return 0.
 */
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* 处理接收到的信息: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* 接受异步信息: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitomplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitomplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    gMmwMCB.stats.frameTriggerReady++;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    gMmwMCB.stats.failedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    gMmwMCB.stats.calibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    /*从BSS接受帧停止异步事件
                                                          不需要任何动作.*/
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b 描述
 *  @n
 *      该任务用来对data path进行处理，并且通过UART输出口传输检测对象。
 *
 *  @返回值
 *      不可用。
 */
void MmwDemo_dataPathTask(UArg arg0, UArg arg1)
{
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;
    uint16_t numDetectedObjects;
    uint32_t startTime, transmitOutStartTime;
    uint32_t txOrder[SYS_COMMON_NUM_TX_ANTENNAS] = {0,2,1};                 //天线顺序
    while(1)
    {
        Semaphore_pend(dataPathObj->frameStart_semHandle, BIOS_WAIT_FOREVER);   //发出信号

        Load_update();
        dataPathObj->timingInfo.interFrameCPULoad=Load_getCPULoad();

        MmwDemo_dataPathWait1D(dataPathObj);
        /* 一维FFT完成 */

        Load_update();
        dataPathObj->timingInfo.activeFrameCPULoad=Load_getCPULoad();

        startTime = Pmu_getCount(0);

        if(dataPathObj->cliCfg->calibDcRangeSigCfg.enabled)
        {
             if (dataPathObj->cliCfg->calibDcRangeSigCfg.numAvgChirps <  dataPathObj->numDopplerBins)
             {
                 dataPathObj->cliCfg->calibDcRangeSigCfg.enabled = 0;
                 dataPathObj->dcRangeForcedDisableCntr++;
             }
             else
             {
                 MmwDemo_dcRangeSignatureCompensation(dataPathObj);
             }
        }
           /*去除杂波 */
        if (dataPathObj->cliCfg->clutterRemovalCfg.enabled)
        {
            int32_t rngIdx, antIdx, dopIdx;
            cmplx16ImRe_t *fftOut1D = (cmplx16ImRe_t *) dataPathObj->radarCube;
            cmplx32ImRe_t meanVal;

            for (rngIdx = 0; rngIdx < dataPathObj->numRangeBins; rngIdx++)
            {
                for (antIdx = 0; antIdx < dataPathObj->numVirtualAntennas; antIdx++)
                {
                    meanVal.real = 0;
                    meanVal.imag = 0;
                    for (dopIdx = 0; dopIdx < dataPathObj->numDopplerBins; dopIdx++)
                    {
                        meanVal.real += fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) + 
                                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].real;
                        meanVal.imag += fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) + 
                                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].imag;
                    }
                    meanVal.real = meanVal.real/dataPathObj->numDopplerBins;
                    meanVal.imag = meanVal.imag/dataPathObj->numDopplerBins;
                    for (dopIdx = 0; dopIdx < dataPathObj->numDopplerBins; dopIdx++)
                    {
                        fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) + 
                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].real -= meanVal.real;
                        fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) + 
                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].imag -= meanVal.imag;
                    }
                }
            }
        }

        MmwDemo_process2D(dataPathObj);
        /* 二维FFT完成 */

        /* 范围偏差测量和Rx通道增益/相位偏移测量的程序 */
        if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
        {
            MmwDemo_rangeBiasRxChPhaseMeasure(
                    dataPathObj->cliCommonCfg->measureRxChanCfg.targetDistance,
                    dataPathObj->rangeResolution,           //范围分辨率
                    dataPathObj->cliCommonCfg->measureRxChanCfg.searchWinSize,
                    dataPathObj->rangeDopplerLogMagMatrix, //范围多普勒矩阵
                    dataPathObj->numDopplerBins,
                    dataPathObj->numVirtualAntennas,
                    dataPathObj->numVirtualAntennas * dataPathObj->numDopplerBins,
                    dataPathObj->radarCube,
                    dataPathObj->numRxAntennas,
                    dataPathObj->numTxAntennas,
                    txOrder,
                    &dataPathObj->cliCommonCfg->compRxChanCfg);

        }

        MmwDemo_processCfar(dataPathObj, &numDetectedObjects);
        /* CFAR完成*/

        /* 预处理/角度估计 */
        dataPathObj->numHwaCfarDetections = numDetectedObjects;
        MmwDemo_processAngle(dataPathObj);

        /* 计算噪音等级. 只用于EVM诊断。假定场景固定不变。 */
        dataPathObj->noiseEnergy = calcNoiseFloor (dataPathObj->radarCube, dataPathObj->numDopplerBins,
                dataPathObj->numRangeBins, dataPathObj->numVirtualAntennas);

        transmitOutStartTime = Pmu_getCount(0);

        /* 将范围偏差测量和Rx通道增益测量值传给MSS以及传给Cli*/
         if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
         {
             MmwDemo_measurementResultOutput (dataPathObj);
         }

        MmwDemo_transmitProcessedOutput(gMmwMCB.loggingUartHandle,
                                        dataPathObj);

        dataPathObj->timingInfo.transmitOutputCycles = Pmu_getCount(0) - transmitOutStartTime;

        /* 为下一帧做准备*/
        MmwDemo_config1D_HWA(dataPathObj);
        MmwDemo_dataPathTrigger1D(dataPathObj);

        /* 处理 2D, CFAR, 方位角/仰角 周期
                               处理    excluding sending out data */
        dataPathObj->timingInfo.interFrameProcessingEndTime = Pmu_getCount(0);
        dataPathObj->timingInfo.interFrameProcCycles = dataPathObj->timingInfo.interFrameProcessingEndTime - startTime -
            dataPathObj->timingInfo.transmitOutputCycles;
        dataPathObj->interFrameProcToken--;


    }
}


/**
 *  @b 描述
 *  @n
 *      帧起始中断处理程序
 *
 *  @返回值
 *      不可用。
 */
static void MmwDemo_frameStartIntHandler(uintptr_t arg)
{
    MmwDemo_DataPathObj * dpObj = &gMmwMCB.dataPathObj;

    /* 增量中断计数器（目的为debug）*/
    dpObj->frameStartIntCounter++;

    /* 注意: 第一帧后有效 */
    dpObj->timingInfo.interFrameProcessingEndMargin =
            Pmu_getCount(0) - dpObj->timingInfo.interFrameProcessingEndTime;

    /* 检查上一个啁啾处理是否完成 */
    MmwDemo_debugAssert(dpObj->interFrameProcToken == 0);
    dpObj->interFrameProcToken++;

    Semaphore_post(dpObj->frameStart_semHandle);

}


/**
 *  @b 描述
 *  @n
 *      系统初始化任务，初始化 系统中各个部分。
 *
 *  @返回值
 *     不可用.
 */
void MmwDemo_initTask(UArg arg0, UArg arg1)                 //初始化
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;

    /* debug信息: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     *初始化mmWave SDK components:
     *****************************************************************************/

    /* 初始化the UART */
    UART_init();

    /* 初始化Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /*初始化GPIO */
    GPIO_init ();

    /* 初始化Data Path: */
    MmwDemo_dataPathInit(&gMmwMCB.dataPathObj);

    /*****************************************************************************
     * 打开&配置驱动:
     *****************************************************************************/

    /* 配置PINMUX工具来打开UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN6_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN6_PADBE, SOC_XWR14XX_PINN6_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN5_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN5_PADBD, SOC_XWR14XX_PINN5_PADBD_MSS_UARTA_RX);

    /* 配置默认UART参数 */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone   = 1;

    /* 打开UART实例 */
    gMmwMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMCB.commandUartHandle == NULL)
    {
        //System_printf("Error: Unable to open the Command UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.commandUartHandle);

    /* 配置默认UART参数 */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 0;

    /* Open the Logging UART Instance: */
    gMmwMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMCB.loggingUartHandle == NULL)
    {
        //System_printf("Error: Unable to open the Logging UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.loggingUartHandle);

    /*****************************************************************************
     * mmWave: 高级模块初始化
     *****************************************************************************/

    /* 初始化mmWave初始化配置 */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* 填入初始化配置: */
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gMmwMCB.socHandle;
    initCfg.eventFxn                = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;

    /* 初始化、设置mmWave控制模块 */
    gMmwMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMCB.ctrlHandle == NULL)
    {
        /* 错误：无法初始化mmWave 控制模块 */
        //System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf ("Debug: mmWave Control Initialization was successful\n");

    /* 同步:这将同步不同区域之间控制模块的执行。
     * 这是一个前提，并且总是需要被调用。*/
    if (MMWave_sync (gMmwMCB.ctrlHandle, &errCode) < 0)
    {
        /* 错误：无法同步mmWave control模块 */
        //System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf ("Debug: mmWave Control Synchronization was successful\n");

    MmwDemo_dataPathOpen(&gMmwMCB.dataPathObj);

    /* 配置基准计数器 */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);

    /*****************************************************************************
     * 启动mmWave control执行任务
     * - 此任务用到了mmWave control API,应该比其他任务优先级更高
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 5;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * 初始化命令行界面模块
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * 初始化传感器管理模块
     *****************************************************************************/
    if (MmwDemo_sensorMgmtInit() < 0)
        return;

    //寄存器帧启动中断处理
    {
        SOC_SysIntListenerCfg  socIntCfg;
        int32_t errCode;

        Semaphore_Params       semParams;

        //寄存器帧启动中断监听
        socIntCfg.systemInterrupt  = SOC_XWR14XX_DSS_FRAME_START_IRQ;
        socIntCfg.listenerFxn      = MmwDemo_frameStartIntHandler;
        socIntCfg.arg              = (uintptr_t)NULL;
        if (SOC_registerSysIntListener(gMmwMCB.socHandle, &socIntCfg, &errCode) == NULL)
        {
            //System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
            MmwDemo_debugAssert (0);
            return;
        }

        Semaphore_Params_init(&semParams);
        semParams.mode = Semaphore_Mode_BINARY;
        gMmwMCB.dataPathObj.frameStart_semHandle = Semaphore_create(0, &semParams, NULL);
    }

    /*****************************************************************************
     * 启动main任务
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 4;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_dataPathTask, &taskParams, NULL);

    return;
}

/**
 *  @b 描述
 *  @n
 *      函数使用等待中断指令来休眠ARM Cortex-R4F内核。
 *      当R4F内核没有工作任务时，bios将会处在空闲进程中，并且引用这个函数。
 *      R4F将会被任意中断唤醒（例：啁啾中断）。
 *
 *  @返回值
 *      不可用
 */
void MmwDemo_sleep(void)
{
    //错误等待中断指令
    asm(" WFI ");
}

/**
 *  @b 描述
 *  @n
 *      mmWave demo演示 入口
 *
 *  @返回参数
 *      不可用
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    //初始化ESM
    ESM_init(0U);

    //初始化SOC配置
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    //封装SOC配置
    socCfg.clockCfg = SOC_SysClock_INIT;

    //初始化SOC模块。应用开始时即执行，以确保MPU正确配置
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        //System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return -1;
    }

    //初始化、配置demo MCB
    memset ((void*)&gMmwMCB, 0, sizeof(MmwDemo_MCB));

    gMmwMCB.socHandle = socHandle;

    //初始化demo配置
    gMmwMCB.cfg.sysClockFrequency = (200 * 1000000);
    gMmwMCB.cfg.loggingBaudRate   = 921600;
    gMmwMCB.cfg.commandBaudRate   = 115200;

    //默认GUI监视器选择
    gMmwMCB.cliCfg.guiMonSel.detectedObjects = 1;
    gMmwMCB.cliCfg.guiMonSel.logMagRange = 1;

    //默认CFAR配置
    gMmwMCB.cliCfg.cfarCfg.averageMode = HWA_NOISE_AVG_MODE_CFAR_CASO;
    gMmwMCB.cliCfg.cfarCfg.cyclicMode = HWA_FEATURE_BIT_DISABLE;
    gMmwMCB.cliCfg.cfarCfg.guardLen = MMW_HWA_CFAR_GUARD_LEN;
    gMmwMCB.cliCfg.cfarCfg.noiseDivShift = MMW_HWA_CFAR_NOISE_DIVISION_RIGHT_SHIFT;
    gMmwMCB.cliCfg.cfarCfg.thresholdScale = MMW_HWA_CFAR_THRESHOLD_SCALE;
    gMmwMCB.cliCfg.cfarCfg.winLen = MMW_HWA_CFAR_WINDOW_LEN;

    gMmwMCB.cliCfg.peakGroupingCfg.inRangeDirectionEn = MMW_HWA_CFAR_PEAK_GROUPING;

#if 0
    //debug信息
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Millimeter Wave Demo\n");
    System_printf ("**********************************************\n");
#endif

    //初始化Task Parameters
    Task_Params_init(&taskParams);
    Task_create(MmwDemo_initTask, &taskParams, NULL);

    //启动bios
    BIOS_start();
    return 0;
}


