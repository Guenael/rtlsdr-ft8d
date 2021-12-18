/*
 * rtlsrd-ft8d, FT8 daemon for RTL receivers
 * Copyright (C) 2016-2021, Guenael Jouchet (VA2GKA)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once


#include <unistd.h>


#ifndef bool
    typedef uint32_t bool;
    #define true  1
    #define false 0
#endif


#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)


/* Sampling definition for RTL devices & FT8 protocol */
#define SIGNAL_LENGHT       15
#define SIGNAL_SAMPLE_RATE  3200
#define SAMPLING_RATE       2400000
#define FS4_RATE            (SAMPLING_RATE / 4)
#define DOWNSAMPLING        (SAMPLING_RATE / SIGNAL_SAMPLE_RATE)
#define DEFAULT_BUF_LENGTH  (4 * 16384)
#define FIR_TAPS            56


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


/* Thread for decoding */
struct decoder_thread {
    pthread_t        thread;
    pthread_attr_t   attr;
    pthread_cond_t   ready_cond;
    pthread_mutex_t  ready_mutex;
};


/* Option & config of the receiver */
struct receiver_state {
    /* Variables used for stop conditions */
    bool     exit_flag;

    /* Double buffering used for sampling */
    float    iSamples[2][SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];
    float    qSamples[2][SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];

    /* Sample index */
    uint32_t iqIndex[2];

    /* Buffer selected (0 or 1) */
    uint32_t bufferIndex;

    /* Time at the beginning of the frame to decode */
    struct tm *gtm;
};

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
};


/* Option & config of decoder */
struct decoder_options {
    uint32_t freq;         // Dial frequency
    char     rcall[13];    // Callsign of the RX station
    char     rloc[7];      // Locator of the RX station
    // char     date[7];      // Date & time of the processes samples
    // char     uttime[5];    //  ''
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
int32_t decoderSelfTest();
void usage(void);
void ft8_subsystem(float *iSamples, float *qSamples, uint32_t samples_len, struct decoder_results *decodes, int32_t *n_results);
