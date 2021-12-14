/*
 * FreeBSD License
 * Copyright (c) 2016-2021, Guenael Jouchet (VA2GKA)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once


#include <unistd.h>


/* Sampling definition for RTL devices & FT8 protocol */
#define SIGNAL_LENGHT       14       // EVAL float here? 14.75?
#define SIGNAL_SAMPLE_RATE  3200     // EVAL 6400 or 9600
#define SAMPLING_RATE       2400000
#define FS4_RATE            (SAMPLING_RATE / 4)
#define DOWNSAMPLING        (SAMPLING_RATE / SIGNAL_SAMPLE_RATE)
#define DEFAULT_BUF_LENGTH  (4 * 16384)
#define FIR_TAPS            32


#define K_MIN_SCORE         10
#define K_MAX_CANDIDATES    120
#define K_LDPC_ITERS        20
#define K_MAX_MESSAGES      50
#define K_FREQ_OSR          2
#define K_TIME_OSR          2
#define K_FSK_DEV           6.25f

#define NUM_BIN             (uint32_t)(SIGNAL_SAMPLE_RATE / (2.0f * K_FSK_DEV)) // 256
#define BLOCK_SIZE          (uint32_t)(SIGNAL_SAMPLE_RATE / K_FSK_DEV)          // 512
#define SUB_BLOCK_SIZE      (uint32_t)(BLOCK_SIZE / K_TIME_OSR)                 // 256
#define NFFT                (uint32_t)(BLOCK_SIZE * K_FREQ_OSR)                 // 1024
#define NUM_BLOCKS          (uint32_t)(((SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE) - NFFT + SUB_BLOCK_SIZE) / BLOCK_SIZE) // 92 vs 92.25 DCHECK
#define MAG_ARRAY           (uint32_t)(NUM_BLOCKS * K_FREQ_OSR * K_TIME_OSR * NUM_BIN)            // 94208 vs 94464 DCHECK

/* Possible PATIENCE options for FFTW:
 * - FFTW_ESTIMATE
 * - FFTW_ESTIMATE_PATIENT
 * - FFTW_MEASURE
 * - FFTW_PATIENT
 * - FFTW_EXHAUSTIVE
 */
#define PATIENCE FFTW_ESTIMATE



#ifndef bool
	typedef uint32_t bool;
	#define true  1
	#define false 0
#endif



struct receiver_state {
    /* Variables used for stop conditions */
    bool     exit_flag;

    /* Double buffering used for sampling */
    float    iSamples[2][SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];
    float    qSamples[2][SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];

    /* Sample index */
    uint32_t iqIndex;

    /* Buffer selected (0 or 1) */
    uint32_t bufferIndex;
};


/* Option & config of the receiver */
struct receiver_options {
    uint32_t dialfreq;
    uint32_t realfreq;
    int32_t  gain;
    int32_t  autogain;
    int32_t  ppm;
    int32_t  shift;
    int32_t  upconverter;
    int32_t  directsampling;
    int32_t  maxloop;
    int32_t  device;
    bool     selftest;
    bool     writefile;
    bool     readfile;
    char     filename[33];
    char     date[7];
    char     uttime[5];
};


/* Option & config of decoder */
struct decoder_options {
    uint32_t freq;         // Dial frequency
    char     rcall[13];    // Callsign of the RX station
    char     rloc[7];      // Locator of the RX station
    char     date[7];      // Date & time of the processes samples
    char     uttime[5];    //  ''
};

struct decoder_results {
    char     call[13];
    char     loc[7];
    int32_t  freq;
    int32_t  snr;
};


static void rtlsdr_callback(unsigned char *samples, uint32_t samples_count, void *ctx);
static void *rtlsdr_rx(void *arg);
static void sigint_callback_handler(int signum);
static void *decoder(void *arg);
void postSpots(uint32_t n_results);
void printSpots();
void saveSample(float *iSamples, float *qSamples);
double atofs(char *s);
int32_t parse_u64(char *s, uint64_t *const value);
void initSampleStorage();
void initrx_options();
void initFFTW();
void freeFFTW();
int32_t readRawIQfile(float *iSamples, float *qSamples, char *filename);
int32_t writeRawIQfile(float *iSamples, float *qSamples, char *filename);
void decodeRecordedFile(char *filename);
float whiteGaussianNoise(float factor);
int32_t ft8DecoderSelfTest();
void usage(void);
void ft8_subsystem(float *iSamples, float *qSamples, uint32_t samples_len, struct decoder_results *decodes, int32_t *n_results);
