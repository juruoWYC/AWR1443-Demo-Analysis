#include <xdc/runtime/System.h>   //SYS/BIOS头文件
#include <ti/drivers/edma/edma.h> //EDMA底层驱动程序
#include "config_edma_util.h"     //本模块头文件
#include "mmw.h"                  //主头文件

/**
 * 配置PaRam Set寄存器并启动数据传输
 *
 * 参数:
 * handle                            指向EDMA通道的句柄
 * chId                              Channel ID
 * linkChId Link                     用于链接的Channel ID,相当于PaRAM ID(重新装载以防止重编程)
 * config                            含有Param Set配置信息的结构体
 * transferCompletionCallbackFxn     数据传输完毕的回调函数
 * transferCompletionCallbackFxnArg  数据传输完毕的回调函数实参(详见edma.h)
 *
 * 返回值:错误代码,定义详见edma.h
 **/
static int32_t EDMA_setup_shadow_link (EDMA_Handle handle, uint32_t chId, uint32_t linkChId,
    EDMA_paramSetConfig_t *config, EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_paramConfig_t paramConfig; //PaRam Set配置信息.定义详见edma.h
    int32_t errorCode = EDMA_NO_ERROR; //错误代码,初始化为无误.定义详见edma.h
	
	//将参数中的PaRam Set配置拷贝到paramConfig
    paramConfig.paramSetConfig = *config;
    paramConfig.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    paramConfig.transferCompletionCallbackFxnArg = (uintptr_t) transferCompletionCallbackFxnArg;
	//配置Param Set寄存器
    if ((errorCode = EDMA_configParamSet(handle, linkChId, &paramConfig)) != EDMA_NO_ERROR)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }
	//链接Param Set寄存器
    if ((errorCode = EDMA_linkParamSets(handle, chId, linkChId)) != EDMA_NO_ERROR)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }
	//链接Param Set寄存器
    if ((errorCode = EDMA_linkParamSets(handle, linkChId, linkChId)) != EDMA_NO_ERROR)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

exit:
    return(errorCode);
}

/**
 * 用于配置简单EDMA传输的API函数，可将特定的32位字(从各包含1个One-Hot签名的16字表中)传输
 * 到HWA的DMA完成寄存器。计数器和地址可以从用于配置HWA的API函数中获得。该函数用于实现基于
 * DMA的HWACC触发模式
 *
 * 参数:
 * handle            指向EDMA通道的句柄
 * chId              Channel ID
 * isEventTriggered  标记Channel ID是否由事件触发
 * pSrcAddress       源地址指针
 * pDestAddress      目标地址指针
 * aCount            A计数器
 * bCount            B计数器
 * cCount            C计数器
 * linkChId Link     用于链接的Channel ID,相当于PaRAM ID(重新装载以防止重编程)
 *
 * 返回值:错误代码,定义详见edma.h
 **/
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

/**
 * 将EDMA配置为转置传输的API函数(用于以转置的方式将1D-FFT输出写入L3缓存)
 *
 * 参数:
 * handle                            指向EDMA通道的句柄
 * chId                              Channel ID
 * linkChId Link                     用于链接的Channel ID,相当于PaRAM ID(重新装载以防止重编程)
 * chainChId                         Chain Channel Id, chId will be chained to this.
 * pSrcAddress                       源地址指针
 * pDestAddress                      目标地址指针
 * numAnt                            接收天线数量
 * numRangeBins                      1D-FFT点数
 * numChirpsPerFrame                 每帧的啁啾数
 * isIntermediateChainingEnabled     Set to 'true' if intermediate transfer chaining is to be enabled.
 * isFinalChainingEnabled            Set to 'true' if final transfer chaining to be enabled.
 * isTransferCompletionEnabled       Set to 'true' if final transfer completion indication is to be enabled.
 * transferCompletionCallbackFxn     数据传输完毕的回调函数
 * transferCompletionCallbackFxnArg  数据传输完毕的回调函数实参(详见edma.h)
 *
 * 返回值:错误代码,定义详见edma.h
 */
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

/**
 * 将EDMA配置为连续传输的API函数
 *
 * 参数:
 * handle                            指向EDMA通道的句柄
 * chId                              Channel ID
 * isEventTriggered                  标记Channel ID是否由事件触发
 * linkChId Link                     用于链接的Channel ID,相当于PaRAM ID(重新装载以防止重编程)
 * chainChId                         Chain Channel Id, chId will be chained to this.
 * pSrcAddress                       源地址指针
 * pDestAddress                      目标地址指针
 * numBytes                          字节数,决定aCount(传输器的第一个维度)
 * numBlocks                         数据块数,决定bCount(传输器的第二个维度)
 * srcIncrBytes                      源地址的增量字节数,决定sourceBindex(每次一维传输后源地址跳跃的字节数)
 * dstIncrBytes                      目标地址的增量字节数,决定destinatinoBindex(每次一维传输后目标地址跳跃的字节数)
 * isIntermediateChainingEnabled     Set to 'true' if intermediate transfer chaining is to be enabled.
 * isFinalChainingEnabled            Set to 'true' if final transfer chaining to be enabled.
 * isTransferCompletionEnabled       Set to 'true' if final transfer completion indication is to be enabled.
 * transferCompletionCallbackFxn     数据传输完毕的回调函数
 * transferCompletionCallbackFxnArg  数据传输完毕的回调函数实参(详见edma.h)
 *
 * 返回值:错误代码,定义详见edma.h
 */
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
