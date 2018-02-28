#ifndef _CONFIGEDMA_H
#define _CONFIGEDMA_H

#include <ti/drivers/edma/edma.h>  //EDMA底层驱动程序

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
    uint8_t chId,
    bool isEventTriggered,
    uint32_t *pSrcAddress, 
    uint32_t *pDestAddress,
    uint16_t aCount, 
    uint16_t bCount, 
    uint16_t cCount, 
    uint16_t linkChId);

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
 * numAnt                            接收天线编号
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
    uint8_t   chId,
    uint16_t  linkChId,
    uint8_t   chainChId,
    uint32_t  *pSrcAddress,
    uint32_t  *pDestAddress,
    uint8_t   numAnt,
    uint16_t  numRangeBins,
    uint16_t  numChirpsPerFrame,
    bool      isIntermediateChainingEnabled,
    bool      isFinalChainingEnabled,
    bool      isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg);

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
    uint8_t   chId,
    bool isEventTriggered,
    uint8_t   linkChId,
    uint8_t   chainChId,
    uint32_t  *pSrcAddress,
    uint32_t  *pDestAddress,
    uint16_t  numBytes,
    uint16_t  numBlocks,
    uint16_t  srcIncrBytes,
    uint16_t  dstIncrBytes,
    bool      isIntermediateChainingEnabled,
    bool      isFinalChainingEnabled,
    bool      isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
	uintptr_t transferCompletionCallbackFxnArg);
#endif
