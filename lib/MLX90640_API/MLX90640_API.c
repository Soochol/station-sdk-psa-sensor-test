/**
 * @file MLX90640_API.c
 * @brief MLX90640 IR Array Sensor API Implementation
 * 
 * Simplified implementation based on Melexis driver.
 * Provides temperature calculation from raw sensor data.
 */

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <math.h>
#include <string.h>

/*============================================================================*/
/* Private Defines                                                            */
/*============================================================================*/

#define MLX90640_EEPROM_START   0x2400
#define MLX90640_RAM_START      0x0400
#define MLX90640_STATUS_REG     0x8000
#define MLX90640_CTRL_REG       0x800D

#define POW2(x) (powf(2.0f, (float)(x)))  /* Use powf to handle any exponent value */
#define SCALEALPHA 0.000001f

/*============================================================================*/
/* Private Function Prototypes                                                */
/*============================================================================*/

static void ExtractVDDParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractPTATParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractGainParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractTgcParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractResolutionParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractKsTaParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractKsToParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractAlphaParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractOffsetParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractCPParameters(uint16_t *eeData, paramsMLX90640 *params);
static void ExtractCILCParameters(uint16_t *eeData, paramsMLX90640 *params);
static int ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90640 *params);
static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);
static float GetMedian(float *values, int n);
static int IsPixelBad(uint16_t pixel, const paramsMLX90640 *params);

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
    return MLX90640_I2CRead(slaveAddr, MLX90640_EEPROM_START, 832, eeData);
}

int MLX90640_ExtractParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int error = 0;
    
    ExtractVDDParameters(eeData, params);
    ExtractPTATParameters(eeData, params);
    ExtractGainParameters(eeData, params);
    ExtractTgcParameters(eeData, params);
    ExtractResolutionParameters(eeData, params);
    ExtractKsTaParameters(eeData, params);
    ExtractKsToParameters(eeData, params);
    ExtractAlphaParameters(eeData, params);
    ExtractOffsetParameters(eeData, params);
    ExtractKtaPixelParameters(eeData, params);
    ExtractKvPixelParameters(eeData, params);
    ExtractCPParameters(eeData, params);
    ExtractCILCParameters(eeData, params);
    error = ExtractDeviatingPixels(eeData, params);
    
    return error;
}

int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t rate)
{
    uint16_t controlRegister;
    uint16_t value;
    int error;
    
    rate = rate & 0x07;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return error;
    }
    
    value = (controlRegister & 0xFC7F) | (rate << 7);
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    
    return error;
}

int MLX90640_GetRefreshRate(uint8_t slaveAddr)
{
    uint16_t controlRegister;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return -1;
    }
    
    return (controlRegister & 0x0380) >> 7;
}

int MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution)
{
    uint16_t controlRegister;
    uint16_t value;
    int error;
    
    resolution = (resolution - 16) & 0x03;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return error;
    }
    
    value = (controlRegister & 0xFCFF) | (resolution << 10);
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    
    return error;
}

int MLX90640_GetCurResolution(uint8_t slaveAddr)
{
    uint16_t controlRegister;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return -1;
    }
    
    return ((controlRegister & 0x0C00) >> 10) + 16;
}

int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData)
{
    uint16_t statusRegister;
    int error;
    int cnt = 0;
    
    /* Wait for new data */
    do {
        error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister);
        if (error != 0) {
            return error;
        }
        cnt++;
        if (cnt > 10000) {
            return -1;  /* Timeout */
        }
    } while ((statusRegister & 0x0008) == 0);
    
    /* Clear status */
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_STATUS_REG, 0x0030);
    if (error != 0) {
        return error;
    }
    
    /* Read RAM data */
    error = MLX90640_I2CRead(slaveAddr, MLX90640_RAM_START, 832, frameData);
    if (error != 0) {
        return error;
    }
    
    /* Read control register for mode info */
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &frameData[832]);
    if (error != 0) {
        return error;
    }
    
    /* Read status for subpage */
    error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &frameData[833]);
    
    return error;
}

int MLX90640_GetSubPageNumber(uint16_t *frameData)
{
    return frameData[833] & 0x0001;
}

float MLX90640_GetVdd(uint16_t *frameData, const paramsMLX90640 *params)
{
    float vdd;
    float resolutionCorrection;
    int resolutionRAM;
    
    resolutionRAM = (frameData[832] & 0x0C00) >> 10;
    resolutionCorrection = POW2(params->resolutionEE) / POW2(resolutionRAM);
    
    vdd = (resolutionCorrection * (int16_t)frameData[810] - params->vdd25) / params->kVdd + 3.3f;
    
    return vdd;
}

float MLX90640_GetTa(uint16_t *frameData, const paramsMLX90640 *params)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = MLX90640_GetVdd(frameData, params);
    
    ptat = (int16_t)frameData[800];
    ptatArt = (int16_t)frameData[768];
    ptatArt = (ptat / (ptat * params->alphaPTAT + ptatArt)) * POW2(18);
    
    ta = (ptatArt / (1 + params->KvPTAT * (vdd - 3.3f)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25;
    
    return ta;
}

void MLX90640_CalculateTo(uint16_t *frameData, const paramsMLX90640 *params,
                          float emissivity, float tr, float *result)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    int8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorrR[5];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
    
    subPage = MLX90640_GetSubPageNumber(frameData);
    vdd = MLX90640_GetVdd(frameData, params);
    ta = MLX90640_GetTa(frameData, params);
    
    ta4 = (ta + 273.15f);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15f);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    taTr = tr4 - (tr4 - ta4) / emissivity;
    
    ktaScale = POW2(params->ktaScale);
    kvScale = POW2(params->kvScale);
    alphaScale = POW2(params->alphaScale);
    
    alphaCorrR[0] = 1 / (1 + params->ksTo[0] * 40);
    alphaCorrR[1] = 1;
    alphaCorrR[2] = (1 + params->ksTo[1] * params->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + params->ksTo[2] * (params->ct[3] - params->ct[2]));
    alphaCorrR[4] = alphaCorrR[3] * (1 + params->ksTo[3] * (params->ct[4] - params->ct[3]));
    
    /* Gain calculation */
    gain = (int16_t)frameData[778];
    if (gain > 32767) {
        gain -= 65536;
    }
    gain = params->gainEE / gain;
    
    /* CP calculation */
    mode = (frameData[832] & 0x1000) >> 12;
    
    irDataCP[0] = (int16_t)frameData[776];
    irDataCP[1] = (int16_t)frameData[808];
    
    irDataCP[0] = irDataCP[0] * gain;
    irDataCP[1] = irDataCP[1] * gain;
    
    irDataCP[0] -= params->cpOffset[0] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3f));
    if (mode == params->calibrationModeEE) {
        irDataCP[1] -= params->cpOffset[1] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3f));
    } else {
        irDataCP[1] -= (params->cpOffset[1] + params->ilChessC[0]) * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3f));
    }
    
    /* Pixel calculation */
    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);
        
        if (mode == 0) {
            pattern = ilPattern;
        } else {
            pattern = chessPattern;
        }
        
        if (pattern == subPage) {
            irData = (int16_t)frameData[pixelNumber];
            irData = irData * gain;
            
            kta = params->kta[pixelNumber] / ktaScale;
            kv = params->kv[pixelNumber] / kvScale;
            
            irData -= params->offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3f));
            
            if (mode != params->calibrationModeEE) {
                irData += params->ilChessC[2] * (2 * ilPattern - 1) - params->ilChessC[1] * conversionPattern;
            }
            
            irData = irData - params->tgc * irDataCP[subPage];
            irData = irData / emissivity;
            
            /* CRITICAL: Correct alphaCompensated calculation (from Melexis reference) */
            if (params->alpha[pixelNumber] == 0) {
                result[pixelNumber] = 0.0f;  /* Skip broken pixel */
                continue;
            }
            alphaCompensated = SCALEALPHA * alphaScale / params->alpha[pixelNumber];
            alphaCompensated = alphaCompensated * (1 + params->KsTa * (ta - 25));
            alphaCompensated = alphaCompensated - params->tgc * params->cpAlpha[subPage];
            
            Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
            Sx = sqrtf(sqrtf(Sx)) * params->ksTo[1];
            
            To = sqrtf(sqrtf(irData / (alphaCompensated * (1 - params->ksTo[1] * 273.15f) + Sx) + taTr)) - 273.15f;
            
            if (To < params->ct[1]) {
                range = 0;
            } else if (To < params->ct[2]) {
                range = 1;
            } else if (To < params->ct[3]) {
                range = 2;
            } else if (To < params->ct[4]) {
                range = 3;
            } else {
                range = 4;
            }
            
            To = sqrtf(sqrtf(irData / (alphaCompensated * alphaCorrR[range] * (1 + params->ksTo[range] * (To - params->ct[range]))) + taTr)) - 273.15f;
            
            result[pixelNumber] = To;
        }
    }
}

int MLX90640_SetInterleavedMode(uint8_t slaveAddr)
{
    uint16_t controlRegister;
    uint16_t value;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return error;
    }
    
    value = controlRegister & 0xEFFF;
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    
    return error;
}

int MLX90640_SetChessMode(uint8_t slaveAddr)
{
    uint16_t controlRegister;
    uint16_t value;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return error;
    }
    
    value = controlRegister | 0x1000;
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    
    return error;
}

int MLX90640_GetCurMode(uint8_t slaveAddr)
{
    uint16_t controlRegister;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        return -1;
    }
    
    return (controlRegister & 0x1000) >> 12;
}

void MLX90640_BadPixelsCorrection(uint16_t *pixels, float *to, int mode,
                                   const paramsMLX90640 *params)
{
    (void)mode;
    (void)params;
    
    /* Simple bad pixel interpolation from neighbors */
    for (int i = 0; i < 5; i++) {
        if (pixels[i] != 0xFFFF && pixels[i] < 768) {
            uint16_t pix = pixels[i];
            int row = pix / 32;
            int col = pix % 32;
            float sum = 0;
            int count = 0;
            
            /* Average from valid neighbors */
            if (row > 0) { sum += to[pix - 32]; count++; }
            if (row < 23) { sum += to[pix + 32]; count++; }
            if (col > 0) { sum += to[pix - 1]; count++; }
            if (col < 31) { sum += to[pix + 1]; count++; }
            
            if (count > 0) {
                to[pix] = sum / count;
            }
        }
    }
}

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

static void ExtractVDDParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int16_t kVdd;
    int16_t vdd25;
    
    kVdd = eeData[51];
    kVdd = (eeData[51] & 0xFF00) >> 8;
    if (kVdd > 127) {
        kVdd -= 256;
    }
    kVdd = 32 * kVdd;
    params->kVdd = kVdd;
    
    vdd25 = eeData[51] & 0x00FF;
    vdd25 = ((vdd25 - 256) << 5) - 8192;
    params->vdd25 = vdd25;
}

static void ExtractPTATParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    
    KvPTAT = (eeData[50] & 0xFC00) >> 10;
    if (KvPTAT > 31) {
        KvPTAT -= 64;
    }
    KvPTAT /= 4096;
    params->KvPTAT = KvPTAT;
    
    KtPTAT = eeData[50] & 0x03FF;
    if (KtPTAT > 511) {
        KtPTAT -= 1024;
    }
    KtPTAT /= 8;
    params->KtPTAT = KtPTAT;
    
    vPTAT25 = eeData[49];
    params->vPTAT25 = vPTAT25;
    
    alphaPTAT = (eeData[16] & 0xF000) / POW2(14) + 8.0f;
    params->alphaPTAT = alphaPTAT;
}

static void ExtractGainParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int16_t gainEE;
    
    gainEE = eeData[48];
    if (gainEE > 32767) {
        gainEE -= 65536;
    }
    params->gainEE = gainEE;
}

static void ExtractTgcParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    float tgc;
    
    tgc = eeData[60] & 0x00FF;
    if (tgc > 127) {
        tgc -= 256;
    }
    tgc /= 32.0f;
    params->tgc = tgc;
}

static void ExtractResolutionParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    uint8_t resolutionEE;
    
    resolutionEE = (eeData[56] & 0x3000) >> 12;
    params->resolutionEE = resolutionEE;
}

static void ExtractKsTaParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    float KsTa;
    
    KsTa = (eeData[60] & 0xFF00) >> 8;
    if (KsTa > 127) {
        KsTa -= 256;
    }
    KsTa /= 8192.0f;
    params->KsTa = KsTa;
}

static void ExtractKsToParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int32_t KsToScale;
    int8_t step;
    
    step = ((eeData[63] & 0x3000) >> 12) * 10;
    
    params->ct[0] = -40;
    params->ct[1] = 0;
    params->ct[2] = (eeData[63] & 0x00F0) >> 4;
    params->ct[3] = (eeData[63] & 0x0F00) >> 8;
    
    params->ct[2] = params->ct[2] * step;
    params->ct[3] = params->ct[2] + params->ct[3] * step;
    params->ct[4] = 400;
    
    KsToScale = (eeData[63] & 0x000F) + 8;
    KsToScale = 1 << KsToScale;
    
    params->ksTo[0] = eeData[61] & 0x00FF;
    params->ksTo[1] = (eeData[61] & 0xFF00) >> 8;
    params->ksTo[2] = eeData[62] & 0x00FF;
    params->ksTo[3] = (eeData[62] & 0xFF00) >> 8;
    
    for (int i = 0; i < 4; i++) {
        if (params->ksTo[i] > 127) {
            params->ksTo[i] -= 256;
        }
        params->ksTo[i] /= KsToScale;
    }
    
    params->ksTo[4] = -0.0002f;
}

static void ExtractAlphaParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;
    float alphaTemp[768];
    float temp;
    
    accRemScale = eeData[32] & 0x000F;
    accColumnScale = (eeData[32] & 0x00F0) >> 4;
    accRowScale = (eeData[32] & 0x0F00) >> 8;
    alphaScale = ((eeData[32] & 0xF000) >> 12) + 30;
    alphaRef = eeData[33];
    
    for (int i = 0; i < 6; i++) {
        p = i * 4;
        accRow[p + 0] = (eeData[34 + i] & 0x000F);
        accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
        accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
        accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
    }
    
    for (int i = 0; i < 24; i++) {
        if (accRow[i] > 7) {
            accRow[i] -= 16;
        }
    }
    
    for (int i = 0; i < 8; i++) {
        p = i * 4;
        accColumn[p + 0] = (eeData[40 + i] & 0x000F);
        accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
        accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
        accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
    }
    
    for (int i = 0; i < 32; i++) {
        if (accColumn[i] > 7) {
            accColumn[i] -= 16;
        }
    }
    
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            p = 32 * i + j;
            alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
            if (alphaTemp[p] > 31) {
                alphaTemp[p] -= 64;
            }
            alphaTemp[p] *= POW2(accRemScale);
            alphaTemp[p] += alphaRef + accRow[i] * POW2(accRowScale) + accColumn[j] * POW2(accColumnScale);
            alphaTemp[p] /= POW2(alphaScale);
        }
    }
    
    temp = alphaTemp[0];
    for (int i = 1; i < 768; i++) {
        if (alphaTemp[i] > temp) {
            temp = alphaTemp[i];
        }
    }

    /* Guard against infinite loop: temp must be positive */
    if (temp <= 0.0f) {
        temp = 1.0f;  /* Prevent divide-by-zero and infinite loop */
    }

    alphaScale = 0;
    while (temp < 32767.4f && alphaScale < 32) {  /* Max 32 iterations */
        temp *= 2;
        alphaScale++;
    }
    
    for (int i = 0; i < 768; i++) {
        temp = alphaTemp[i] * POW2(alphaScale);
        params->alpha[i] = (uint16_t)(temp + 0.5f);
    }
    
    params->alphaScale = alphaScale;
}

static void ExtractOffsetParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;
    
    occRemScale = eeData[16] & 0x000F;
    occColumnScale = (eeData[16] & 0x00F0) >> 4;
    occRowScale = (eeData[16] & 0x0F00) >> 8;
    offsetRef = eeData[17];
    if (offsetRef > 32767) {
        offsetRef -= 65536;
    }
    
    for (int i = 0; i < 6; i++) {
        p = i * 4;
        occRow[p + 0] = (eeData[18 + i] & 0x000F);
        occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4;
        occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
        occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
    }
    
    for (int i = 0; i < 24; i++) {
        if (occRow[i] > 7) {
            occRow[i] -= 16;
        }
    }
    
    for (int i = 0; i < 8; i++) {
        p = i * 4;
        occColumn[p + 0] = (eeData[24 + i] & 0x000F);
        occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
        occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
        occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
    }
    
    for (int i = 0; i < 32; i++) {
        if (occColumn[i] > 7) {
            occColumn[i] -= 16;
        }
    }
    
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            p = 32 * i + j;
            params->offset[p] = (eeData[64 + p] & 0xFC00) >> 10;
            if (params->offset[p] > 31) {
                params->offset[p] -= 64;
            }
            params->offset[p] *= (1 << occRemScale);
            params->offset[p] += offsetRef + occRow[i] * (1 << occRowScale) + occColumn[j] * (1 << occColumnScale);
        }
    }
}

static void ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int p = 0;
    int8_t KtaRoRe;
    int8_t KtaRoCo;
    int8_t KtaReCo;
    int8_t KtaReRe;
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
    float ktaTemp[768];
    float temp;
    
    KtaRoRe = (eeData[54] & 0xFF00) >> 8;
    if (KtaRoRe > 127) {
        KtaRoRe -= 256;
    }
    KtaReCo = eeData[54] & 0x00FF;
    if (KtaReCo > 127) {
        KtaReCo -= 256;
    }
    KtaRoCo = (eeData[55] & 0xFF00) >> 8;
    if (KtaRoCo > 127) {
        KtaRoCo -= 256;
    }
    KtaReRe = eeData[55] & 0x00FF;
    if (KtaReRe > 127) {
        KtaReRe -= 256;
    }
    
    ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
    ktaScale2 = eeData[56] & 0x000F;
    
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            p = 32 * i + j;
            split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
            ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
            if (ktaTemp[p] > 3) {
                ktaTemp[p] -= 8;
            }
            ktaTemp[p] *= (1 << ktaScale2);
            
            if (split == 0) {
                ktaTemp[p] += KtaRoRe;
            } else if (split == 1) {
                ktaTemp[p] += KtaReCo;
            } else if (split == 2) {
                ktaTemp[p] += KtaRoCo;
            } else {
                ktaTemp[p] += KtaReRe;
            }
            
            ktaTemp[p] /= POW2(ktaScale1);
        }
    }
    
    temp = fabsf(ktaTemp[0]);
    for (int i = 1; i < 768; i++) {
        if (fabsf(ktaTemp[i]) > temp) {
            temp = fabsf(ktaTemp[i]);
        }
    }
    
    ktaScale1 = 0;
    while (temp < 63.4f) {
        temp *= 2;
        ktaScale1++;
    }
    
    for (int i = 0; i < 768; i++) {
        temp = ktaTemp[i] * POW2(ktaScale1);
        if (temp < 0) {
            params->kta[i] = (int8_t)(temp - 0.5f);
        } else {
            params->kta[i] = (int8_t)(temp + 0.5f);
        }
    }
    
    params->ktaScale = ktaScale1;
}

static void ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    int p = 0;
    int8_t KvRoRe;
    int8_t KvRoCo;
    int8_t KvReCo;
    int8_t KvReRe;
    uint8_t kvScale;
    uint8_t split;
    float kvTemp[768];
    float temp;
    
    KvRoRe = (eeData[52] & 0xF000) >> 12;
    if (KvRoRe > 7) {
        KvRoRe -= 16;
    }
    KvReCo = (eeData[52] & 0x0F00) >> 8;
    if (KvReCo > 7) {
        KvReCo -= 16;
    }
    KvRoCo = (eeData[52] & 0x00F0) >> 4;
    if (KvRoCo > 7) {
        KvRoCo -= 16;
    }
    KvReRe = eeData[52] & 0x000F;
    if (KvReRe > 7) {
        KvReRe -= 16;
    }
    
    kvScale = (eeData[56] & 0x0F00) >> 8;
    
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            p = 32 * i + j;
            split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
            
            if (split == 0) {
                kvTemp[p] = KvRoRe;
            } else if (split == 1) {
                kvTemp[p] = KvReCo;
            } else if (split == 2) {
                kvTemp[p] = KvRoCo;
            } else {
                kvTemp[p] = KvReRe;
            }
            
            kvTemp[p] /= POW2(kvScale);
        }
    }
    
    temp = fabsf(kvTemp[0]);
    for (int i = 1; i < 768; i++) {
        if (fabsf(kvTemp[i]) > temp) {
            temp = fabsf(kvTemp[i]);
        }
    }
    
    kvScale = 0;
    while (temp < 63.4f) {
        temp *= 2;
        kvScale++;
    }
    
    for (int i = 0; i < 768; i++) {
        temp = kvTemp[i] * POW2(kvScale);
        if (temp < 0) {
            params->kv[i] = (int8_t)(temp - 0.5f);
        } else {
            params->kv[i] = (int8_t)(temp + 0.5f);
        }
    }
    
    params->kvScale = kvScale;
}

static void ExtractCPParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;
    
    alphaScale = ((eeData[32] & 0xF000) >> 12) + 27;
    
    offsetSP[0] = eeData[58] & 0x03FF;
    if (offsetSP[0] > 511) {
        offsetSP[0] -= 1024;
    }
    
    offsetSP[1] = (eeData[58] & 0xFC00) >> 10;
    if (offsetSP[1] > 31) {
        offsetSP[1] -= 64;
    }
    offsetSP[1] += offsetSP[0];
    
    alphaSP[0] = eeData[57] & 0x03FF;
    if (alphaSP[0] > 511) {
        alphaSP[0] -= 1024;
    }
    alphaSP[0] /= POW2(alphaScale);
    
    alphaSP[1] = (eeData[57] & 0xFC00) >> 10;
    if (alphaSP[1] > 31) {
        alphaSP[1] -= 64;
    }
    alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0];
    
    cpKta = eeData[59] & 0x00FF;
    if (cpKta > 127) {
        cpKta -= 256;
    }
    ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
    params->cpKta = cpKta / POW2(ktaScale1);
    
    cpKv = (eeData[59] & 0xFF00) >> 8;
    if (cpKv > 127) {
        cpKv -= 256;
    }
    kvScale = (eeData[56] & 0x0F00) >> 8;
    params->cpKv = cpKv / POW2(kvScale);
    
    params->cpAlpha[0] = alphaSP[0];
    params->cpAlpha[1] = alphaSP[1];
    params->cpOffset[0] = offsetSP[0];
    params->cpOffset[1] = offsetSP[1];
}

static void ExtractCILCParameters(uint16_t *eeData, paramsMLX90640 *params)
{
    float ilChessC[3];
    uint8_t calibrationModeEE;
    
    calibrationModeEE = (eeData[10] & 0x0800) >> 4;
    calibrationModeEE = calibrationModeEE ^ 0x80;
    
    ilChessC[0] = eeData[53] & 0x003F;
    if (ilChessC[0] > 31) {
        ilChessC[0] -= 64;
    }
    ilChessC[0] /= 16.0f;
    
    ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
    if (ilChessC[1] > 15) {
        ilChessC[1] -= 32;
    }
    ilChessC[1] /= 2.0f;
    
    ilChessC[2] = (eeData[53] & 0xF800) >> 11;
    if (ilChessC[2] > 15) {
        ilChessC[2] -= 32;
    }
    ilChessC[2] /= 8.0f;
    
    params->calibrationModeEE = calibrationModeEE;
    params->ilChessC[0] = ilChessC[0];
    params->ilChessC[1] = ilChessC[1];
    params->ilChessC[2] = ilChessC[2];
}

static int ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90640 *params)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    
    for (int i = 0; i < 5; i++) {
        params->brokenPixels[i] = 0xFFFF;
        params->outlierPixels[i] = 0xFFFF;
    }
    
    for (int i = 0; i < 768; i++) {
        if (eeData[64 + i] == 0) {
            params->brokenPixels[brokenPixCnt] = i;
            brokenPixCnt++;
        } else if ((eeData[64 + i] & 0x0001) != 0) {
            params->outlierPixels[outlierPixCnt] = i;
            outlierPixCnt++;
        }
        
        if (brokenPixCnt > 4 || outlierPixCnt > 4) {
            return MLX90640_BROKEN_PIXEL_ERROR;
        }
    }
    
    pixCnt = brokenPixCnt + outlierPixCnt;
    
    if (pixCnt > 4) {
        return MLX90640_BROKEN_PIXEL_ERROR;
    }
    
    for (int i = 0; i < brokenPixCnt; i++) {
        for (int j = i + 1; j < brokenPixCnt; j++) {
            if (CheckAdjacentPixels(params->brokenPixels[i], params->brokenPixels[j])) {
                return MLX90640_BROKEN_PIXEL_ERROR;
            }
        }
    }
    
    for (int i = 0; i < outlierPixCnt; i++) {
        for (int j = i + 1; j < outlierPixCnt; j++) {
            if (CheckAdjacentPixels(params->outlierPixels[i], params->outlierPixels[j])) {
                return MLX90640_BROKEN_PIXEL_ERROR;
            }
        }
    }
    
    for (int i = 0; i < brokenPixCnt; i++) {
        for (int j = 0; j < outlierPixCnt; j++) {
            if (CheckAdjacentPixels(params->brokenPixels[i], params->outlierPixels[j])) {
                return MLX90640_BROKEN_PIXEL_ERROR;
            }
        }
    }
    
    return MLX90640_NO_ERROR;
}

static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
{
    int pixPosDif;
    
    pixPosDif = pix1 - pix2;
    if (pixPosDif > -34 && pixPosDif < -30) {
        return -6;
    }
    if (pixPosDif > -2 && pixPosDif < 2) {
        return -6;
    }
    if (pixPosDif > 30 && pixPosDif < 34) {
        return -6;
    }
    
    return 0;
}

static float GetMedian(float *values, int n)
{
    float temp;
    
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (values[j] < values[i]) {
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
    
    if (n % 2 == 0) {
        return (values[n / 2] + values[n / 2 - 1]) / 2.0f;
    } else {
        return values[n / 2];
    }
}

static int IsPixelBad(uint16_t pixel, const paramsMLX90640 *params)
{
    for (int i = 0; i < 5; i++) {
        if (pixel == params->brokenPixels[i] || pixel == params->outlierPixels[i]) {
            return 1;
        }
    }
    return 0;
}
