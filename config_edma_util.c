#include <xdc/runtime/System.h>
#include <ti/drivers/edma/edma.h>  //EDMA底层驱动程序
#include "config_edma_util.h"      //本模块头文件
#include "mmw.h"                   //主头文件


static int32_t EDMA_setup_shadow_link (EDMA_Handle handle, uint32_t chId, uint32_t linkChId,
    EDMA_paramSetConfig_t *config, EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_paramConfig_t paramConfig;
    int32_t errorCode = EDMA_NO_ERROR;

    paramConfig.paramSetConfig = *config; //this will copy the entire param set config
    paramConfig.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    paramConfig.transferCompletionCallbackFxnArg = (uintptr_t) transferCompletionCallbackFxnArg;
    if ((errorCode = EDMA_configParamSet(handle, linkChId, &paramConfig)) != EDMA_NO_ERROR)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

    if ((errorCode = EDMA_linkParamSets(handle, chId, linkChId)) != EDMA_NO_ERROR)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

    if ((errorCode = EDMA_linkParamSets(handle, linkChId, linkChId)) != EDMA_NO_ERROR)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

exit:
    return(errorCode);
}

int32_t EDMAutil_configHwaOneHotSignature(EDMA_Handle handle, 
    uint8_t chId, bool isEventTriggered, 
    uint32_t* pSrcAddress, uint32_t * pDestAddress,
    uint16_t aCount, uint16_t bCount, uint16_t cCount, 
    uint16_t linkChId)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) pSrcAddress;
    config.paramSetConfig.destinationAddress = (uint32_t) pDestAddress;

    config.paramSetConfig.aCount = aCount;
    config.paramSetConfig.bCount = bCount;
    config.paramSetConfig.cCount = cCount;
    config.paramSetConfig.bCountReload = config.paramSetConfig.bCount;

    config.paramSetConfig.sourceBindex = 0;
    config.paramSetConfig.destinationBindex = 0;

    config.paramSetConfig.sourceCindex = 0;
    config.paramSetConfig.destinationCindex = 0;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = 0;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = false;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = NULL;

    if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        MmwDemo_debugAssert (0);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, linkChId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, NULL);

exit:
    return(errorCode);
}

int32_t EDMAutil_configHwaTranspose(EDMA_Handle handle,
    uint8_t chId, uint16_t linkChId, uint8_t chainChId,
    uint32_t* pSrcAddress, uint32_t * pDestAddress,
    uint8_t numAnt, uint16_t numRangeBins, uint16_t numChirpsPerFrame,
    bool isIntermediateChainingEnabled,
    bool isFinalChainingEnabled,
    bool isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) pSrcAddress;
    config.paramSetConfig.destinationAddress = (uint32_t) pDestAddress;

    config.paramSetConfig.aCount = numAnt*4;
    config.paramSetConfig.bCount = numRangeBins;
    config.paramSetConfig.cCount = numChirpsPerFrame/2;
    config.paramSetConfig.bCountReload = 0; //config.paramSetConfig.bCount;

    config.paramSetConfig.sourceBindex = numAnt*4;
    config.paramSetConfig.destinationBindex = numChirpsPerFrame*numAnt*4;

    config.paramSetConfig.sourceCindex = 0;
    config.paramSetConfig.destinationCindex = numAnt*4*2;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_AB;
    config.paramSetConfig.transferCompletionCode = chainChId;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;
    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled =
        isTransferCompletionEnabled;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled =
        isFinalChainingEnabled;
    config.paramSetConfig.isIntermediateChainingEnabled =
        isIntermediateChainingEnabled;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;

    if (transferCompletionCallbackFxn != NULL) {
        config.transferCompletionCallbackFxnArg = (uintptr_t) transferCompletionCallbackFxnArg;
    }

    if ((errorCode = EDMA_configChannel(handle, &config, true)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        MmwDemo_debugAssert (0);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, linkChId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}

int32_t EDMAutil_configHwaContiguous(EDMA_Handle handle,
    uint8_t chId, bool isEventTriggered,
    uint8_t linkChId, uint8_t chainChId,
    uint32_t * pSrcAddress,uint32_t * pDestAddress,
    uint16_t numBytes, uint16_t numBlocks,
    uint16_t srcIncrBytes, uint16_t dstIncrBytes,
    bool isIntermediateChainingEnabled,
    bool isFinalChainingEnabled,
    bool isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) pSrcAddress;
    config.paramSetConfig.destinationAddress = (uint32_t) pDestAddress;

    config.paramSetConfig.aCount = numBytes;
    config.paramSetConfig.bCount = numBlocks;
    config.paramSetConfig.cCount = 1;
    config.paramSetConfig.bCountReload = config.paramSetConfig.bCount;

    config.paramSetConfig.sourceBindex = srcIncrBytes;
    config.paramSetConfig.destinationBindex = dstIncrBytes;

    config.paramSetConfig.sourceCindex = 0;
    config.paramSetConfig.destinationCindex = 0;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = chainChId;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled =
        isTransferCompletionEnabled;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled =
        isFinalChainingEnabled;
    config.paramSetConfig.isIntermediateChainingEnabled =
        isIntermediateChainingEnabled;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    
    if (transferCompletionCallbackFxn != NULL) {
        config.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;
    }

    if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        MmwDemo_debugAssert (0);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, linkChId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}
