/*
 * FreeBSD License
 * Copyright (c) 2016-2021, Guenael Jouchet (VA2GKA)
 * All rights reserved.
 *
 * This file is based on rtlsdr-wsprd project:
 *   https://github.com/Guenael/rtlsdr-wsprd
 * FT8 decoder used is under MIT licence, located in this git sub-module:
 *   https://github.com/kgoba/ft8_lib
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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <assert.h>
#include <rtl-sdr.h>
#include <fftw3.h>

#include "./rtlsdr_ft8d.h"

#include "./ft8_lib/ft8/constants.h"
#include "./ft8_lib/ft8/pack.h"
#include "./ft8_lib/ft8/unpack.h"
#include "./ft8_lib/ft8/ldpc.h"
#include "./ft8_lib/ft8/crc.h"
#include "./ft8_lib/ft8/decode.h"
#include "./ft8_lib/ft8/encode.h"


/* Sampling definition for RTL devices & FT8 protocol */
#define SIGNAL_LENGHT       15
#define SIGNAL_SAMPLE_RATE  3200
#define SIGNAL_NUM_SAMPLES  SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE                  // = 48_000 (IQ signal)
#define SAMPLING_RATE       2400000
#define FS4_RATE            (SAMPLING_RATE / 4)                                 // = 600_000
#define DOWNSAMPLING        (SAMPLING_RATE / SIGNAL_SAMPLE_RATE)                // = 750
#define DEFAULT_BUF_LENGTH  (4 * 16384)                                         // = 65_536

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
#define NUM_BLOCKS          (uint32_t)((SIGNAL_NUM_SAMPLES - NFFT + SUB_BLOCK_SIZE) / BLOCK_SIZE) // 92 vs 92.25 CHECK/FIXME
#define MAG_ARRAY           (uint32_t)(NUM_BLOCKS * K_FREQ_OSR * K_TIME_OSR * NUM_BIN)            // 94208 vs 94464 CHECK/FIXME

/* Possible PATIENCE options for FFTW:
 * - FFTW_ESTIMATE
 * - FFTW_ESTIMATE_PATIENT
 * - FFTW_MEASURE
 * - FFTW_PATIENT
 * - FFTW_EXHAUSTIVE
 */
#define PATIENCE FFTW_ESTIMATE


/* Global declaration for states & options */
struct receiver_state   rx_state;
struct receiver_options rx_options;
struct decoder_options  dec_options;
struct decoder_results  dec_results[50];
static rtlsdr_dev_t     *rtl_device = NULL;


/* Thread stuff for separate decoding */
struct decoder_state {
    pthread_t        thread;
    pthread_attr_t   tattr;

    pthread_rwlock_t rw;
    pthread_cond_t   ready_cond;
    pthread_mutex_t  ready_mutex;
};
struct decoder_state dec_state;


/* Thread stuff for separate RX (blocking function) */
struct dongle_state {
    pthread_t thread;
};
struct dongle_state dongle;


/* FFTW pointers & stuff */
static fftwf_plan fft_plan;
static fftwf_complex *fft_in, *fft_out;
static FILE *fp_fftw_wisdom_file;
static float *hann;


/* Callback for each buffer received */
static void rtlsdr_callback(unsigned char *samples, uint32_t samples_count, void *ctx) {
    int8_t *sigIn = (int8_t *)samples;

    /* CIC buffers/vars */
    static int32_t  Ix1 = 0, Ix2 = 0,
                    Qx1 = 0, Qx2 = 0;
    static int32_t  Iy1 = 0, It1y = 0, It1z = 0,
                    Qy1 = 0, Qt1y = 0, Qt1z = 0;
    static int32_t  Iy2 = 0, It2y = 0, It2z = 0,
                    Qy2 = 0, Qt2y = 0, Qt2z = 0;
    static uint32_t decimationIndex = 0;

    /* FIR compensation filter coefs
       Using : Octave/MATLAB code for generating compensation FIR coefficients
       URL : https://github.com/WestCoastDSP/CIC_Octave_Matlab
     */
    const static float zCoef[33] = {
        0.000358374, -0.003301220, -0.000609181, 0.006729330,
        0.001403270, -0.014276800, -0.003107820, 0.027350100,
        0.006522470, -0.048282500, -0.013725800, 0.082059600,
        0.031488900, -0.142080000, -0.091379400, 0.271348000,
        0.500000000,
        0.271348000, -0.091379400, -0.142080000, 0.031488900,
        0.082059600, -0.013725800, -0.048282500, 0.006522470,
        0.027350100, -0.003107820, -0.014276800, 0.001403270,
        0.006729330, -0.000609181, -0.003301220, 0.000358374,
    };

    /* FIR compensation filter buffers */
    static float firI[32] = {0.0},
                 firQ[32] = {0.0};

    /* Convert unsigned to signed */
    for (uint32_t i = 0; i < samples_count; i++) {
        sigIn[i] ^= 0x80;  // XOR with a binary mask to flip the first bit (sign)
    }

    /* Economic mixer @ fs/4 (upper band)
       At fs/4, sin and cosin calculation are no longer necessary.

               0   | pi/2 |  pi  | 3pi/2
             ----------------------------
       sin =   0   |  1   |  0   |  -1  |
       cos =   1   |  0   | -1   |   0  |

       out_I = in_I * cos(x) - in_Q * sin(x)
       out_Q = in_Q * cos(x) + in_I * sin(x)
       (Weaver technique, keep the upper band, IQ inverted on RTL devices)
    */
    int8_t tmp;
    for (uint32_t i = 0; i < samples_count; i += 8) {
        tmp = -sigIn[i + 3];
        sigIn[i + 3] = sigIn[i + 2];
        sigIn[i + 2] = tmp;

        sigIn[i + 4] = -sigIn[i + 4];
        sigIn[i + 5] = -sigIn[i + 5];

        tmp = -sigIn[i + 6];
        sigIn[i + 6] = sigIn[i + 7];
        sigIn[i + 7] = tmp;
    }

    /* CIC decimator (N=2)
       Info: * Understanding CIC Compensation Filters
               https://www.altera.com/en_US/pdfs/literature/an/an455.pdf
             * Understanding cascaded integrator-comb filters
               http://www.embedded.com/design/configurable-systems/4006446/Understanding-cascaded-integrator-comb-filters
    */
    for (int32_t i = 0; i < samples_count / 2; i++) {
        /* Integrator stages (N=2) */
        // EVAL: option to move sigIn in float here
        Ix1 += (int32_t)sigIn[i * 2];
        Qx1 += (int32_t)sigIn[i * 2 + 1];
        Ix2 += Ix1;
        Qx2 += Qx1;

        /* Decimation stage */
        decimationIndex++;
        if (decimationIndex <= DOWNSAMPLING) {
            continue;
        }

        // FIXME/TODO : some optimization here
        /* 1st Comb */
        Iy1  = Ix2 - It1z;
        It1z = It1y;
        It1y = Ix2;
        Qy1  = Qx2 - Qt1z;
        Qt1z = Qt1y;
        Qt1y = Qx2;

        /* 2nd Comd */
        Iy2  = Iy1 - It2z;
        It2z = It2y;
        It2y = Iy1;
        Qy2  = Qy1 - Qt2z;
        Qt2z = Qt2y;
        Qt2y = Qy1;

        /* FIR compensation filter */
        float Isum = 0.0, Qsum = 0.0;
        for (uint32_t j = 0; j < 32; j++) {
            Isum += firI[j] * zCoef[j];
            Qsum += firQ[j] * zCoef[j];
            if (j < 31) {
                firI[j] = firI[j + 1];
                firQ[j] = firQ[j + 1];
            }
        }
        firI[31] = (float)Iy2;
        firQ[31] = (float)Qy2;
        Isum += firI[31] * zCoef[32];
        Qsum += firQ[31] * zCoef[32];

        /* Save the result in the buffer */
        if (rx_state.iqIndex < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE)) {
            /* Lock the buffer during writing */
            pthread_rwlock_wrlock(&dec_state.rw);
            rx_state.iSamples[rx_state.iqIndex] = Isum / (8192.0 * DOWNSAMPLING);
            rx_state.qSamples[rx_state.iqIndex] = Qsum / (8192.0 * DOWNSAMPLING);
            pthread_rwlock_unlock(&dec_state.rw);
            rx_state.iqIndex++;
        } else {
            if (rx_state.decode_flag == false) {
                /* Send a signal to the other thread to start the decoding */
                pthread_mutex_lock(&dec_state.ready_mutex);
                pthread_cond_signal(&dec_state.ready_cond);
                pthread_mutex_unlock(&dec_state.ready_mutex);
                rx_state.decode_flag = true;
            }
        }
        decimationIndex = 0;
    }
}


/* Thread for RX blocking function */
static void *rtlsdr_rx(void *arg) {
    /* Read & blocking call */
    rtlsdr_read_async(rtl_device, rtlsdr_callback, NULL, 0, DEFAULT_BUF_LENGTH);
    exit(0);
    return 0;
}


/* PSKreporter protocol documentation:
 * https://pskreporter.info/pskdev.html
 */
void postSpots(uint32_t n_results) {
    // use struct decoder_options
    // use struct decoder_results
    // use snprintf

    // const unsigned char headerDescriptor[] = {
    //     0x00, 0x0A,                                      // ID
    //     0x00, 0x00, 0x00, 0x00,                          // Message Length (update)
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // CurrentDateTime (update)
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Sequence Number (update)
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // Random number (update)
    // };

    // const unsigned char rxInfoDescriptor[] = {  // (RX = RTLsdr owner)
    //     0x00, 0x03,                          // Options Template Set ID
    //     0x00, 0x2C,                          // Length
    //     0x50, 0xE2,                          // Link ID
    //     0x00, 0x04,                          // Field Count
    //     0x00, 0x00,                          // Scope Field Count
    //     0x80, 0x02,                          // Receiver Callsign ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x80, 0x04,                          // Receiver Locator ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x80, 0x08,                          // Receiver Decoder Software ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x80, 0x09,                          // Receiver Antenna ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x00, 0x00                           // Padding
    // };

    // const unsigned char txInfoDescriptor[] = {  // (TX = Signal received)
    //     0x00, 0x02,                          // Options Template Set ID
    //     0x00, 0x3C,                          // Length
    //     0x50, 0xE3,                          // Link ID
    //     0x00, 0x07,                          // Field Count
    //     0x80, 0x01,                          // Sender Callsign ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x80, 0x05,                          // Frequency ID
    //     0x00, 0x04, 0x00, 0x00, 0x76, 0x8F,  // Fixed field (4) length & enterprise number
    //     0x80, 0x06,                          // SNR ID
    //     0x00, 0x01, 0x00, 0x00, 0x76, 0x8F,  // Fixed field (1) length & enterprise number
    //     0x80, 0x0A,                          // Mode ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x80, 0x03,                          // Sender Locator ID
    //     0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F,  // Variable field length & enterprise number
    //     0x80, 0x0B,                          // Information Source ID
    //     0x00, 0x01, 0x00, 0x00, 0x76, 0x8F,  // Fixed field (1) length & enterprise number
    //     0x00, 0x96,                          // DateTimeSeconds ID
    //     0x00, 0x04                           // Field Length
    // };

    // // FT8 tag
    // // socket.sendto(packet, ("report.pskreporter.info", 4739))
    // // TX every 5 minutes

    // const unsigned char rxInfoData[] = "\x50\xE2+++"; // Malloc
    // const unsigned char txInfoData[] = "\x50\xE3+++"; // Malloc

    return;
}


void printSpots(uint32_t n_results) {
    printf("  Score     Freq       Call    Loc\n");

    for (uint32_t i = 0; i < n_results; i++) {
        printf("     %2d %8d %10s %6s\n",
               dec_results[i].snr,
               dec_results[i].freq,
               dec_results[i].call,
               dec_results[i].loc);
    }
}


void saveSample(float *iSamples, float *qSamples) {
    if (rx_options.writefile == true) {
        char filename[32];

        time_t rawtime;
        time(&rawtime);
        struct tm *gtm = gmtime(&rawtime);

        snprintf(filename, sizeof(filename) - 1, "%.8s_%04d-%02d-%02d_%02d-%02d-%02d.iq",
                 rx_options.filename,
                 gtm->tm_year + 1900,
                 gtm->tm_mon + 1,
                 gtm->tm_mday,
                 gtm->tm_hour,
                 gtm->tm_min,
                 gtm->tm_sec);

        writeRawIQfile(iSamples, qSamples, filename);
    }
}


static void *decoder(void *arg) {
    /* FT8 decoder use buffers of 48000 samples
       (15 sec max @ 3200sps = 48000 samples)
    */
    static float iSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static float qSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static uint32_t samples_len;
    int32_t n_results = 0;

    while (!rx_state.exit_flag) {
        pthread_mutex_lock(&dec_state.ready_mutex);
        pthread_cond_wait(&dec_state.ready_cond, &dec_state.ready_mutex);
        pthread_mutex_unlock(&dec_state.ready_mutex);

        if (rx_state.exit_flag)
            break;  /* Abort case, final sig */

        /* Lock the buffer access and make a local copy */
        pthread_rwlock_wrlock(&dec_state.rw);
        memcpy(iSamples, rx_state.iSamples, rx_state.iqIndex * sizeof(float));
        memcpy(qSamples, rx_state.qSamples, rx_state.iqIndex * sizeof(float));
        samples_len = rx_state.iqIndex;  // Overkill ?
        pthread_rwlock_unlock(&dec_state.rw);

        /* Date and time will be updated/overload during the search & decoding process
           Make a simple copy
        */
        memcpy(dec_options.date, rx_options.date, sizeof(rx_options.date));
        memcpy(dec_options.uttime, rx_options.uttime, sizeof(rx_options.uttime));

        /* Search & decode the signal */
        ft8_subsystem(iSamples, qSamples, samples_len, dec_results, &n_results);
        saveSample(iSamples, qSamples);
        //postSpots(n_results);
        printSpots(n_results);
    }
    pthread_exit(NULL);
}


double atofs(char *s) {
    /* standard suffixes */
    char last;
    uint32_t len;
    double suff = 1.0;
    len = strlen(s);
    last = s[len - 1];
    s[len - 1] = '\0';

    switch (last) {
        case 'g':
        case 'G':
            suff *= 1e3;
        case 'm':
        case 'M':
            suff *= 1e3;
        case 'k':
        case 'K':
            suff *= 1e3;
            suff *= atof(s);
            s[len - 1] = last;
            return suff;
    }
    s[len - 1] = last;
    return atof(s);
}


int32_t parse_u64(char *s, uint64_t *const value) {
    uint_fast8_t base = 10;
    char *s_end;
    uint64_t u64_value;

    if (strlen(s) > 2) {
        if (s[0] == '0') {
            if ((s[1] == 'x') || (s[1] == 'X')) {
                base = 16;
                s += 2;
            } else if ((s[1] == 'b') || (s[1] == 'B')) {
                base = 2;
                s += 2;
            }
        }
    }

    s_end = s;
    u64_value = strtoull(s, &s_end, base);
    if ((s != s_end) && (*s_end == 0)) {
        *value = u64_value;
        return 1;
    } else {
        return 0;
    }
}


/* Reset flow control variable & decimation variables */
void initSampleStorage() {
    rx_state.decode_flag = false;
    rx_state.iqIndex = 0;
}


/* Default options for the receiver */
void initrx_options() {
    rx_options.gain           = 290;
    rx_options.autogain       = 0;
    rx_options.ppm            = 0;
    rx_options.shift          = 0;
    rx_options.directsampling = 0;
    rx_options.maxloop        = 0;
    rx_options.device         = 0;
    rx_options.selftest       = false;
    rx_options.writefile      = false;
    rx_options.readfile       = false;
}


void initFFTW() {
    /* Recover existing FFTW optimisations */
    if ((fp_fftw_wisdom_file = fopen("fftw_wisdom.dat", "r"))) {
        fftwf_import_wisdom_from_file(fp_fftw_wisdom_file);
        fclose(fp_fftw_wisdom_file);
    }

    /* Allocate FFT buffers */
    fft_in  = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * NFFT);
    fft_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * NFFT);

    /* FFTW internal plan */
    fft_plan = fftwf_plan_dft_1d(NFFT, fft_in, fft_out, FFTW_FORWARD, PATIENCE);

    /* Calculate Hann function only one time
     * https://en.wikipedia.org/wiki/Hann_function
     */
    hann = malloc(sizeof(float) * NFFT);
    for (int i = 0; i < NFFT; i++) {
        hann[i] = sinf((M_PI / NFFT) * i);
    }
}


void freeFFTW() {
  fftwf_free(fft_in);
  fftwf_free(fft_out);

  if ((fp_fftw_wisdom_file = fopen("fftw_wisdom.dat", "w"))) {
      fftwf_export_wisdom_to_file(fp_fftw_wisdom_file);
      fclose(fp_fftw_wisdom_file);
  }
  fftwf_destroy_plan(fft_plan);
}


void sigint_callback_handler(int signum) {
    fprintf(stdout, "Caught signal %d\n", signum);
    rx_state.exit_flag = true;
}


int32_t readRawIQfile(float *iSamples, float *qSamples, char *filename) {
    float filebuffer[2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];  // Allocate the max. size allowed
    FILE *fd = fopen(filename, "rb");

    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 0;
    }

    /* Get the size of the file */
    fseek(fd, 0L, SEEK_END);
    int32_t recsize = ftell(fd) / (2 * sizeof(float));
    fseek(fd, 0L, SEEK_SET);


    /* Limit the file/buffer to 45000 samples (120 sec signal) */
    if (recsize > SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE) {
        recsize = SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE;
    }

    /* Read the iq file */
    int32_t nread = fread(filebuffer, sizeof(float), 2 * recsize, fd);
    if (nread != 2 * recsize) {
        fprintf(stderr, "Cannot read all the data! %d\n", nread);
        fclose(fd);
        return 0;
    } else {
        fclose(fd);
    }

    /* Convert the interleaved buffer into 2 buffers */
    for (int32_t i = 0; i < recsize; i++) {
        iSamples[i] =  filebuffer[2 * i];
        qSamples[i] = -filebuffer[2 * i + 1];  // neg, convention used by wsprsim
    }

    return recsize;
}


int32_t writeRawIQfile(float *iSamples, float *qSamples, char *filename) {
    float filebuffer[2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];

    FILE *fd = fopen(filename, "wb");
    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 0;
    }

    for (int32_t i = 0; i < SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE; i++) {
        filebuffer[2 * i]     =  iSamples[i];
        filebuffer[2 * i + 1] = -qSamples[i];  // neg, convention used by wsprsim
    }

    int32_t nwrite = fwrite(filebuffer, sizeof(float), 2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE, fd);
    if (nwrite != 2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE) {
        fprintf(stderr, "Cannot write all the data!\n");
        return 0;
    }

    fclose(fd);
    return SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE;
}


void decodeRecordedFile(char *filename) {
    static float iSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static float qSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static uint32_t samples_len;
    int32_t n_results = 0;

    samples_len = readRawIQfile(iSamples, qSamples, filename);
    printf("Number of samples: %d\n", samples_len);

    if (samples_len) {
        /* Search & decode the signal */
        ft8_subsystem(iSamples, qSamples, samples_len, dec_results, &n_results);

        printSpots(n_results);
    }
}


float whiteGaussianNoise(float factor) {
    static double V1, V2, U1, U2, S, X;
    static int phase = 0;

    if (phase == 0) {
        do {
            U1 = rand() / (double)RAND_MAX;
            U2 = rand() / (double)RAND_MAX;
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);

        X = V1 * sqrt(-2 * log(S) / S);
    } else {
        X = V2 * sqrt(-2 * log(S) / S);
    }

    phase = 1 - phase;
    return (float)X * factor;
}


int32_t ft8DecoderSelfTest() {
    static float iSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static float qSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static uint32_t samples_len = SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE;
    int32_t n_results = 0;

    /* Ref test message
     * Message : "CQ K1JT FN20QI"
     * Packed data: 00 00 00 20 4d fc dc 8a 14 08
     * FSK tones: 3140652000000001005477547106035036373140652547441342116056460065174427143140652
     */
    const char message[] = "CQ K1JT FN20QI";
    uint8_t packed[FT8_LDPC_K_BYTES];

    if (pack77(message, packed) < 0){
        printf("Cannot parse message!\n");
        return 0;
    }

    // Second, encode the binary message as a sequence of FSK tones
    uint8_t tones[FT8_NN];
    ft8_encode(packed, tones);


    // Encoding, simple FSK modulation
    float  f0  = 50.0;
    float  t0  = 0.0;  // Caution!! Possible buffer overflow with the index calculation (no user input here!)
    float  amp = 0.5;
    float  wgn = 0.02;
    double phi = 0.0;
    double df  = 3200.0 / 512.0;
    double dt  = 1 / 3200.0;
    double twopidt = 8.0 * atan(1.0) / 3200.0;

    // Add signal
    for (int i = 0; i < FT8_NN; i++) {
        double dphi = twopidt * (f0 + ( (double)tones[i]-3.5) * df);
        for (int j = 0; j < 512; j++) {
            int index = t0 / dt + 512 * i + j;
            iSamples[index] = amp * cos(phi) + whiteGaussianNoise(wgn);
            qSamples[index] = amp * sin(phi) + whiteGaussianNoise(wgn);
            phi = phi + dphi;
        }
    }
    
    /* Save the test sample */
    writeRawIQfile(iSamples, qSamples, "selftest.iq");

    /* Search & decode the signal */
    ft8_subsystem(iSamples, qSamples, samples_len, dec_results, &n_results);

    printSpots(n_results);

    /* Simple consistency check */
    if (strcmp(dec_results[0].call, "K1JT") &&
        strcmp(dec_results[0].loc,  "FN20")) {
        return 0;
    } else {
        return 1;
    }
}


void usage(void) {
    fprintf(stderr,
            "rtlsdr_ft8d, a simple FT8 daemon for RTL receivers\n\n"
            "Use:\rtlsdr_ft8d -f frequency -c callsign -l locator [options]\n"
            "\t-f dial frequency [(,k,M) Hz] or band string\n"
            "\t   If band string is used, the default dial frequency will used.\n"
            "\t   Bands: LF MF 160m 80m 60m 40m 30m 20m 17m 15m 12m 10m 6m 4m 2m 1m25 70cm 23cm\n"
            "\t-c your callsign (12 chars max)\n"
            "\t-l your locator grid (6 chars max)\n"
            "Receiver extra options:\n"
            "\t-g gain [0-49] (default: 29)\n"
            "\t-a auto gain (off by default, no parameter)\n"
            "\t-o frequency offset (default: 0)\n"
            "\t-p crystal correction factor (ppm) (default: 0)\n"
            "\t-u upconverter (default: 0, example: 125M)\n"
            "\t-d direct dampling [0,1,2] (default: 0, 1 for I input, 2 for Q input)\n"
            "\t-n max iterations (default: 0 = infinite loop)\n"
            "\t-i device index (in case of multiple receivers, default: 0)\n"
            "Debugging options:\n"
            "\t-t decoder self-test (generate a signal & decode), no parameter\n"
            "\t-w write received signal and exit [filename prefix]\n"
            "\t-r read signal, decode and exit [filename]\n"
            "\t   (raw format: 3200sps, float 32 bits, 2 channels)\n"
            "Example:\n"
            "\rtlsdr_ft8d -f 2m -c A1XYZ -l AB12cd -g 29\n");
    exit(1);
}


int main(int argc, char **argv) {
    uint32_t opt;

    int32_t rtl_result;
    int32_t rtl_count;
    char    rtl_vendor[256], rtl_product[256], rtl_serial[256];

    initrx_options();

    /* FFTW init & allocation */
    initFFTW();

    /* Stop condition setup */
    rx_state.exit_flag   = false;
    rx_state.decode_flag = false;
    uint32_t nLoop = 0;

    if (argc <= 1)
        usage();

    while ((opt = getopt(argc, argv, "f:c:l:g:ao:p:u:dn:i:tw:r:")) != -1) {
        switch (opt) {
            case 'f':  // Frequency
                if (!strcasecmp(optarg, "LF")) {
                    rx_options.dialfreq = 136000;
                } else if (!strcasecmp(optarg, "MF")) {
                    rx_options.dialfreq = 474200;
                } else if (!strcasecmp(optarg, "160m")) {
                    rx_options.dialfreq = 1836600;
                } else if (!strcasecmp(optarg, "80m")) {
                    rx_options.dialfreq = 3592600;
                } else if (!strcasecmp(optarg, "60m")) {
                    rx_options.dialfreq = 5287200;
                } else if (!strcasecmp(optarg, "40m")) {
                    rx_options.dialfreq = 7038600;
                } else if (!strcasecmp(optarg, "30m")) {
                    rx_options.dialfreq = 10138700;
                } else if (!strcasecmp(optarg, "20m")) {
                    rx_options.dialfreq = 14095600;
                } else if (!strcasecmp(optarg, "17m")) {
                    rx_options.dialfreq = 18104600;
                } else if (!strcasecmp(optarg, "15m")) {
                    rx_options.dialfreq = 21094600;
                } else if (!strcasecmp(optarg, "12m")) {
                    rx_options.dialfreq = 24924600;
                } else if (!strcasecmp(optarg, "10m")) {
                    rx_options.dialfreq = 28124600;
                } else if (!strcasecmp(optarg, "6m")) {
                    rx_options.dialfreq = 50293000;
                } else if (!strcasecmp(optarg, "4m")) {
                    rx_options.dialfreq = 70091000;
                } else if (!strcasecmp(optarg, "2m")) {
                    rx_options.dialfreq = 144489000;
                } else if (!strcasecmp(optarg, "1m25")) {
                    rx_options.dialfreq = 222280000;
                } else if (!strcasecmp(optarg, "70cm")) {
                    rx_options.dialfreq = 432300000;
                } else if (!strcasecmp(optarg, "23cm")) {
                    rx_options.dialfreq = 1296500000;
                } else {
                    rx_options.dialfreq = (uint32_t)atofs(optarg);
                }
                break;
            case 'c':  // Callsign
                snprintf(dec_options.rcall, sizeof(dec_options.rcall), "%.12s", optarg);
                break;
            case 'l':  // Locator / Grid
                snprintf(dec_options.rloc, sizeof(dec_options.rloc), "%.6s", optarg);
                break;
            case 'g':  // Small signal amplifier gain
                rx_options.gain = atoi(optarg);
                if (rx_options.gain < 0) rx_options.gain = 0;
                if (rx_options.gain > 49) rx_options.gain = 49;
                rx_options.gain *= 10;
                break;
            case 'a':  // Auto gain
                rx_options.autogain = 1;
                break;
            case 'o':  // Fine frequency correction
                rx_options.shift = atoi(optarg);
                break;
            case 'p':  // Crystal correction
                rx_options.ppm = atoi(optarg);
                break;
            case 'u':  // Upconverter frequency
                rx_options.upconverter = (uint32_t)atofs(optarg);
                break;
            case 'd':  // Direct Sampling
                rx_options.directsampling = 1;
                break;
            case 'n':  // Stop after n iterations
                rx_options.maxloop = (uint32_t)atofs(optarg);
                break;
            case 'i':  // Select the device to use
                rx_options.device = (uint32_t)atofs(optarg);
                break;
            case 't':  // Seft test (used in unit-test CI pipeline)
                rx_options.selftest = true;
                break;
            case 'w':  // Read a signal and decode
                rx_options.writefile = true;
                snprintf(rx_options.filename, sizeof(rx_options.filename), "%.32s", optarg);
                break;
            case 'r':  // Write a signal and exit
                rx_options.readfile = true;
                snprintf(rx_options.filename, sizeof(rx_options.filename), "%.32s", optarg);
                break;
            default:
                usage();
                break;
        }
    }

    if (rx_options.selftest == true) {
        if (ft8DecoderSelfTest()) {
            fprintf(stdout, "Self-test SUCCESS!\n");
            exit(0);
        }
        else {
            fprintf(stderr, "Self-test FAILED!\n");
            exit(1);
        }
    }

    if (rx_options.readfile == true) {
        fprintf(stdout, "Reading IQ file: %s\n", rx_options.filename);
        decodeRecordedFile(rx_options.filename);
        exit(0);
    }

    if (rx_options.writefile == true) {
        fprintf(stdout, "Saving IQ file planned with prefix: %.8s\n", rx_options.filename);
    }

    if (rx_options.dialfreq == 0) {
        fprintf(stderr, "Please specify a dial frequency.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    if (dec_options.rcall[0] == 0) {
        fprintf(stderr, "Please specify your callsign.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    if (dec_options.rloc[0] == 0) {
        fprintf(stderr, "Please specify your locator.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    /* Calcule shift offset */
    rx_options.realfreq = rx_options.dialfreq + rx_options.shift + rx_options.upconverter;

    /* Store the frequency used for the decoder */
    dec_options.freq = rx_options.dialfreq;

    /* RX buffer allocation */
    rx_state.iSamples = malloc(sizeof(float) * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE);
    rx_state.qSamples = malloc(sizeof(float) * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE);

    /* If something goes wrong... */
    signal(SIGINT,  &sigint_callback_handler);
    signal(SIGTERM, &sigint_callback_handler);
    signal(SIGILL,  &sigint_callback_handler);
    signal(SIGFPE,  &sigint_callback_handler);
    signal(SIGSEGV, &sigint_callback_handler);
    signal(SIGABRT, &sigint_callback_handler);

    /* Init & parameter the device */
    rtl_count = rtlsdr_get_device_count();
    if (!rtl_count) {
        fprintf(stderr, "No supported devices found\n");
        return EXIT_FAILURE;
    }

    fprintf(stderr, "Found %d device(s):\n", rtl_count);
    for (uint32_t i = 0; i < rtl_count; i++) {
        rtlsdr_get_device_usb_strings(i, rtl_vendor, rtl_product, rtl_serial);
        fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, rtl_vendor, rtl_product, rtl_serial);
    }
    fprintf(stderr, "\nUsing device %d: %s\n", rx_options.device, rtlsdr_get_device_name(rx_options.device));

    rtl_result = rtlsdr_open(&rtl_device, rx_options.device);
    if (rtl_result < 0) {
        fprintf(stderr, "ERROR: Failed to open rtlsdr device #%d.\n", rx_options.device);
        return EXIT_FAILURE;
    }

    if (rx_options.directsampling) {
        rtl_result = rtlsdr_set_direct_sampling(rtl_device, rx_options.directsampling);
        if (rtl_result < 0) {
            fprintf(stderr, "ERROR: Failed to set direct sampling\n");
            rtlsdr_close(rtl_device);
            return EXIT_FAILURE;
        }
    }

    rtl_result = rtlsdr_set_sample_rate(rtl_device, SAMPLING_RATE);
    if (rtl_result < 0) {
        fprintf(stderr, "ERROR: Failed to set sample rate\n");
        rtlsdr_close(rtl_device);
        return EXIT_FAILURE;
    }

    rtl_result = rtlsdr_set_tuner_gain_mode(rtl_device, 1);
    if (rtl_result < 0) {
        fprintf(stderr, "ERROR: Failed to enable manual gain\n");
        rtlsdr_close(rtl_device);
        return EXIT_FAILURE;
    }

    if (rx_options.autogain) {
        rtl_result = rtlsdr_set_tuner_gain_mode(rtl_device, 0);
        if (rtl_result != 0) {
            fprintf(stderr, "ERROR: Failed to set tuner gain\n");
            rtlsdr_close(rtl_device);
            return EXIT_FAILURE;
        }
    } else {
        rtl_result = rtlsdr_set_tuner_gain(rtl_device, rx_options.gain);
        if (rtl_result != 0) {
            fprintf(stderr, "ERROR: Failed to set tuner gain\n");
            rtlsdr_close(rtl_device);
            return EXIT_FAILURE;
        }
    }

    if (rx_options.ppm != 0) {
        rtl_result = rtlsdr_set_freq_correction(rtl_device, rx_options.ppm);
        if (rtl_result < 0) {
            fprintf(stderr, "ERROR: Failed to set ppm error\n");
            rtlsdr_close(rtl_device);
            return EXIT_FAILURE;
        }
    }

    rtl_result = rtlsdr_set_center_freq(rtl_device, rx_options.realfreq + FS4_RATE);
    if (rtl_result < 0) {
        fprintf(stderr, "ERROR: Failed to set frequency\n");
        rtlsdr_close(rtl_device);
        return EXIT_FAILURE;
    }

    rtl_result = rtlsdr_reset_buffer(rtl_device);
    if (rtl_result < 0) {
        fprintf(stderr, "ERROR: Failed to reset buffers.\n");
        rtlsdr_close(rtl_device);
        return EXIT_FAILURE;
    }


    /* Time & date tools */
    time_t rawtime;
    time(&rawtime);
    struct tm *gtm = gmtime(&rawtime);
    struct timeval lTime;
    gettimeofday(&lTime, NULL);

    /* Print used parameter */
    printf("\nStarting rtlsdr-ft8d (%04d-%02d-%02d, %02d:%02dz) -- Version 0.2alpha\n",
           gtm->tm_year + 1900, gtm->tm_mon + 1, gtm->tm_mday, gtm->tm_hour, gtm->tm_min);
    printf("  Callsign     : %s\n", dec_options.rcall);
    printf("  Locator      : %s\n", dec_options.rloc);
    printf("  Dial freq.   : %d Hz\n", rx_options.dialfreq);
    printf("  Real freq.   : %d Hz\n", rx_options.realfreq);
    printf("  PPM factor   : %d\n", rx_options.ppm);
    if (rx_options.autogain)
        printf("  Auto gain    : enable\n");
    else
        printf("  Gain         : %d dB\n", rx_options.gain / 10);

    // FIXME: time adjustment @ 14s or 14.5s

    /* Time alignment stuff */
    uint32_t sec = lTime.tv_sec % 15;
    uint32_t usec = sec * 1000000 + lTime.tv_usec;
    uint32_t uwait = 15000000 - usec;
    printf("Wait for time sync (start in %d sec)\n\n", uwait / 1000000);

    /* Prepare a low priority param for the decoder thread */
    struct sched_param param;
    pthread_attr_init(&dec_state.tattr);
    pthread_attr_setschedpolicy(&dec_state.tattr, SCHED_RR);
    pthread_attr_getschedparam(&dec_state.tattr, &param);
    param.sched_priority = 90;  // = sched_get_priority_min();
    pthread_attr_setschedparam(&dec_state.tattr, &param);

    /* Create a thread and stuff for separate decoding
       Info : https://computing.llnl.gov/tutorials/pthreads/
    */
    pthread_rwlock_init(&dec_state.rw, NULL);
    pthread_cond_init(&dec_state.ready_cond, NULL);
    pthread_mutex_init(&dec_state.ready_mutex, NULL);
    pthread_create(&dongle.thread, NULL, rtlsdr_rx, NULL);
    pthread_create(&dec_state.thread, &dec_state.tattr, decoder, NULL);

    /* Main loop : Wait, read, decode */
    while (!rx_state.exit_flag && !(rx_options.maxloop && (nLoop >= rx_options.maxloop))) {
        /* Wait for time Sync on 2 mins */
        gettimeofday(&lTime, NULL);
        sec = lTime.tv_sec % 15;
        usec = sec * 1000000 + lTime.tv_usec;
        uwait = 15000000 - usec + 10000;  // Adding 10ms, to be sure to reach this next minute
        usleep(uwait);

        /* Use the Store the date at the begin of the frame */
        time(&rawtime);
        gtm = gmtime(&rawtime);
        snprintf(rx_options.date, sizeof(rx_options.date), "%02d%02d%02d", gtm->tm_year - 100, gtm->tm_mon + 1, gtm->tm_mday);
        snprintf(rx_options.uttime, sizeof(rx_options.uttime), "%02d%02d", gtm->tm_hour, gtm->tm_min);

        /* Start to store the samples */
        initSampleStorage();

        while ((rx_state.exit_flag == false) &&
               (rx_state.iqIndex < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE))) {
            usleep(250000);
        }
        nLoop++;
    }

    /* Stop the RX and free the blocking function */
    rtlsdr_cancel_async(rtl_device);

    /* Close the RTL device */
    rtlsdr_close(rtl_device);

    /* Free FFTW buffers */
    freeFFTW();

    printf("Bye!\n");

    /* Wait the thread join (send a signal before to terminate the job) */
    pthread_mutex_lock(&dec_state.ready_mutex);
    pthread_cond_signal(&dec_state.ready_cond);
    pthread_mutex_unlock(&dec_state.ready_mutex);
    pthread_join(dec_state.thread, NULL);
    pthread_join(dongle.thread, NULL);

    /* Destroy the lock/cond/thread */
    pthread_rwlock_destroy(&dec_state.rw);
    pthread_cond_destroy(&dec_state.ready_cond);
    pthread_mutex_destroy(&dec_state.ready_mutex);
    pthread_exit(NULL);

    return EXIT_SUCCESS;
}


/*
 * FT8 Protocol doc high level
 * - https://physics.princeton.edu/pulsar/k1jt/FT4_FT8_QEX.pdf
 * - http://laarc.weebly.com/uploads/7/3/2/9/73292865/ft8syncv8.pdf
 * - http://www.sportscliche.com/wb2fko/WB2FKO_TAPR_revised.pdf
 */
void ft8_subsystem(float *iSamples,
                   float *qSamples,
                   uint32_t samples_len,
                   struct decoder_results *decodes,
                   int32_t *n_results) {

    // FIXME: adjust with samples_len !!

    /* Compute FFT over the whole signal and store it */
    uint8_t mag_power[MAG_ARRAY];

    int offset = 0;
    float max_mag = -120.0f;

    for (int idx_block = 0; idx_block < NUM_BLOCKS; ++idx_block) {
        // Loop over two possible time offsets (0 and BLOCK_SIZE/2)
        for (int time_sub = 0; time_sub < K_TIME_OSR; ++time_sub) {
            float mag_db[NFFT];

            for (int i = 0; i < NFFT; ++i) {
                fft_in[i][0] = iSamples[(idx_block * BLOCK_SIZE) + (time_sub * SUB_BLOCK_SIZE) + i] * hann[i];
                fft_in[i][1] = qSamples[(idx_block * BLOCK_SIZE) + (time_sub * SUB_BLOCK_SIZE) + i] * hann[i];
            }
            fftwf_execute(fft_plan);

            // Compute log magnitude in decibels
            for (int i = 0; i < NFFT; ++i) {
                float mag2 = fft_out[i][0] * fft_out[i][0] + fft_out[i][1] * fft_out[i][1];
                mag_db[i] = 10.0f * log10f(1E-12f + mag2 * 4.0f / (NFFT * NFFT));
            }

            // Loop over two possible frequency bin offsets (for averaging)
            for (int freq_sub = 0; freq_sub < K_FREQ_OSR; ++freq_sub) {
                for (int pos = 0; pos < NUM_BIN; ++pos) {
                    float db = mag_db[pos * K_FREQ_OSR + freq_sub];
                    // Scale decibels to unsigned 8-bit range and clamp the value
                    // Range 0-240 covers -120..0 dB in 0.5 dB steps
                    int scaled = (int)(2 * db + 240);

                    mag_power[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
                    ++offset;

                    if (db > max_mag)
                        max_mag = db;
                }
            }
        }
    }
    //fprintf(stderr, "Max magnitude: %.1f dB\n", max_mag);

    /* Find top candidates by Costas sync score and localize them in time and frequency */
    candidate_t candidate_list[K_MAX_CANDIDATES];
    waterfall_t power = {
        .num_blocks = NUM_BLOCKS,
        .num_bins = NUM_BIN,
        .time_osr = K_TIME_OSR,
        .freq_osr = K_FREQ_OSR,
        .mag = mag_power
    };
    int num_candidates = ft8_find_sync(&power, K_MAX_CANDIDATES, candidate_list, K_MIN_SCORE);

    // Hash table for decoded messages (to check for duplicates)
    int num_decoded = 0;
    message_t decoded[K_MAX_MESSAGES];
    message_t *decoded_hashtable[K_MAX_MESSAGES];

    // Initialize hash table pointers
    for (int i = 0; i < K_MAX_MESSAGES; ++i) {
        decoded_hashtable[i] = NULL;
    }

    //fprintf(stdout, "SNR  DT   Freq ~ Message\n");

    // Go over candidates and attempt to decode messages
    for (int idx = 0; idx < num_candidates; ++idx) {
        const candidate_t *cand = &candidate_list[idx];
        if (cand->score < K_MIN_SCORE)
            continue;

        float freq_hz = (cand->freq_offset + (float)cand->freq_sub / K_FREQ_OSR) * K_FSK_DEV;
        // float time_sec = (cand->time_offset + (float)cand->time_sub / K_TIME_OSR) / K_FSK_DEV;
        // fprintf(stderr, "Checking hash table for %4.1fs / %4.1fHz [%d]...\n", time_sec, freq_hz, cand->score);

        message_t message;
        decode_status_t status;
        if (!ft8_decode(&power, cand, &message, K_LDPC_ITERS, &status)) {
            if (status.ldpc_errors > 0) {
                //fprintf(stderr, "LDPC decode: %d errors\n", status.ldpc_errors);
            } else if (status.crc_calculated != status.crc_extracted) {
                //fprintf(stderr, "CRC mismatch!\n");
            } else if (status.unpack_status != 0) {
                //fprintf(stderr, "Error while unpacking!\n");
            }
            continue;
        }

        int idx_hash = message.hash % K_MAX_MESSAGES;
        bool found_empty_slot = false;
        bool found_duplicate = false;
        do {
            if (decoded_hashtable[idx_hash] == NULL) {
                //fprintf(stderr, "Found an empty slot\n");
                found_empty_slot = true;
            } else if ((decoded_hashtable[idx_hash]->hash == message.hash) && (0 == strcmp(decoded_hashtable[idx_hash]->text, message.text))) {
                //fprintf(stderr, "Found a duplicate [%s]\n", message.text);
                found_duplicate = true;
            } else {
                //fprintf(stderr, "Hash table clash!\n");
                // Move on to check the next entry in hash table
                idx_hash = (idx_hash + 1) % K_MAX_MESSAGES;
            }
        } while (!found_empty_slot && !found_duplicate);

        /* Add this entry to an empty hashtable slot */
        if (found_empty_slot) {
            memcpy(&decoded[idx_hash], &message, sizeof(message));
            decoded_hashtable[idx_hash] = &decoded[idx_hash];

            char *strPtr = strtok(message.text, " ");
            if (!strncmp(strPtr, "CQ", 2)) {  // Only get the CQ messages
                strPtr = strtok(NULL, " ");   // Move on the Callsign part
                snprintf(decodes[num_decoded].call, sizeof(decodes[num_decoded].call), "%.12s", strPtr);
                strPtr = strtok(NULL, " ");   // Move on the Locator part
                snprintf(decodes[num_decoded].loc, sizeof(decodes[num_decoded].loc), "%.6s", strPtr);

                decodes[num_decoded].freq = (int32_t)freq_hz;
                decodes[num_decoded].snr  = (int32_t)cand->score;  // FIXME: it's not true, score != snr
            }

            num_decoded++;
        }
    }
    *n_results = num_decoded;
}
