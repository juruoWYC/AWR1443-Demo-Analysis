/**
 *   @file  rx_ch_bias_measure.c
 *
 *   @brief
 *      Implements Data path processing functionality.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
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

#include <ti/demo/utils/rx_ch_bias_measure.h>
#include <math.h>
#ifdef SUBSYS_DSS
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE" 
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop
#endif

void MmwDemo_quadFit(float *x, float*y, float *xv, float *yv)
{
    float a, b, c, denom;
    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float y0 = y[0];
    float y1 = y[1];
    float y2 = y[2];


    denom = (x0 - x1)*(x0 - x2)*(x1 - x2);
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom;
    b = (x2*x2 * (y0 - y1) + x1*x1 * (y2 - y0) + x0*x0 * (y1 - y2)) / denom;
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;

    *xv = -b/(2*a);
    *yv = c - b*b/(4*a);
}

void MmwDemo_rangeBiasRxChPhaseMeasure(

                                        float targetDistance,
                                        float rangeResolution,
                                        float searchWinSize,
                                       uint16_t *detMatrix,
                                       uint16_t numDopplerBins,
                                       uint32_t numVirtualAntennas,
                                       uint32_t numColInSymbolMatrix,
                                       uint32_t *symbolMatrix,
                                       uint32_t numRxAnt,
                                       uint32_t numTxAnt,
                                       uint32_t *txOrder,
                                       MmwDemo_compRxChannelBiasCfg_t *compRxChanCfg)
{
    cmplx16ImRe_t rxSym[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    cmplx16ImRe_t *tempPtr;
    float sumSqr;
    uint32_t * rxSymPtr = (uint32_t * ) rxSym;
    float xMagSq[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    int32_t iMax;
    float xMagSqMin;
    float scal;
    float truePosition = targetDistance / rangeResolution;
    int32_t truePositionIndex = (int32_t) (truePosition + 0.5);
    float y[3];
    float x[3];
    int32_t halfWinSize = (int32_t) (0.5 * searchWinSize / rangeResolution + 0.5);
    float estPeakPos;
    float estPeakVal;
    int32_t i, j, ind;
    int32_t txIdx, rxIdx;

    /**** Range calibration ****/
    iMax = truePositionIndex;
    uint16_t maxVal = 0;
    for (i = truePositionIndex - halfWinSize; i <= truePositionIndex + halfWinSize; i++)
    {
        if (detMatrix[i * numDopplerBins] > maxVal)
        {
            maxVal = detMatrix[i * numDopplerBins];
            iMax = i;
        }
    }

    /* Fine estimate of the peak position using quadratic fit */
    ind = 0;
    for (i = iMax-1; i <= iMax+1; i++)
    {
        for (j = 0, sumSqr = 0.0; j < numVirtualAntennas; j++)
        {
            tempPtr = (cmplx16ImRe_t *) &symbolMatrix[i * numColInSymbolMatrix + j];
            sumSqr += (float) tempPtr->real * (float) tempPtr->real +
                      (float) tempPtr->imag * (float) tempPtr->imag;
        }
#ifdef SUBSYS_DSS
        /* Although we can use rts's sqrt here because MIPS is not a concern,
         * because we already use sqrtsp in application code, adding sqrt will
         * increase code size because of linking of that function from rts lib.
         */
        y[ind] = sqrtsp(sumSqr);
#else
        y[ind] = sqrt(sumSqr);
#endif
        x[ind] = (float)i;
        ind++;
    }
    MmwDemo_quadFit(x, y, &estPeakPos, &estPeakVal);
    compRxChanCfg->rangeBias = (estPeakPos - truePosition) * rangeResolution;

    /*** Calculate Rx channel phase/gain compensation coefficients ***/
    for (i = 0; i < numVirtualAntennas; i++)
    {
        rxSymPtr[i] = symbolMatrix[iMax * numColInSymbolMatrix + i];
        xMagSq[i] = (float) rxSym[i].real * (float) rxSym[i].real +
                    (float) rxSym[i].imag * (float) rxSym[i].imag;
    }

    xMagSqMin = xMagSq[0];
    for (i = 1; i < numVirtualAntennas; i++)
    {
        if (xMagSq[i] < xMagSqMin)
        {
            xMagSqMin = xMagSq[i];
        }
    }

    i=0;
    for (txIdx=0; txIdx < numTxAnt; txIdx++)
    {
        for (rxIdx=0; rxIdx < numRxAnt; rxIdx++)
        {
            int32_t temp;
            scal = 32768./ xMagSq[i] * sqrt(xMagSqMin);

            temp = (int32_t) ROUND(scal * rxSym[i].real);
            temp = MMWDEMO_SATURATE_HIGH( temp);
            temp = MMWDEMO_SATURATE_LOW( temp);
            compRxChanCfg->rxChPhaseComp[txOrder[txIdx]*numRxAnt + rxIdx].real = (int16_t) (temp);

            temp = (int32_t) ROUND(-scal * rxSym[i].imag);
            temp = MMWDEMO_SATURATE_HIGH( temp);
            temp = MMWDEMO_SATURATE_LOW( temp);
            compRxChanCfg->rxChPhaseComp[txOrder[txIdx]*numRxAnt + rxIdx].imag = (int16_t) (temp);
            i++;
        }
    }

}


