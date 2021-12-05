/*
 * FreeBSD License
 * Copyright (c) 2021, Guenael
 * All rights reserved.
 *
 * This file is based on rtl-sdr project, contribution :
 *   Copyright (C) 2012 by Steve Markgraf <steve{at}steve-m.de>
 *   Copyright (C) 2012 by Hoernchen <la{at}tfc-server.de>
 *   Copyright (C) 2012 by Kyle Keen <keenerd{at}gmail.com>
 *   Copyright (C) 2013 by Elias Oenal <EliasOenal{at}gmail.com>
 *   Copyright (C) 2016 by Guenael Jouchet <guenael{at}jouchet.ca>
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
#include <curl/curl.h>
#include <rtl-sdr.h>

#include "./rtlsdr_ft8d.h"

#include "ft8/unpack.h"
#include "ft8/ldpc.h"
#include "ft8/decode.h"
#include "ft8/constants.h"
#include "ft8/encode.h"
#include "ft8/crc.h"
#include "kiss_fft/kiss_fft.h"


/* snprintf possible truncation allowed to prevent possible buffer overflow */
#pragma GCC diagnostic ignored "-Wformat-truncation"


#define SIGNAL_LENGHT       15
#define SIGNAL_SAMPLE_RATE  12000                               // UPDATE to 4000 or 3000 (iq = 3kHz BW)
#define SAMPLING_RATE       2400000
#define FS4_RATE            SAMPLING_RATE / 4                   // = 600 kHz
#define DOWNSAMPLING        SAMPLING_RATE / SIGNAL_SAMPLE_RATE  // = 200
#define DEFAULT_BUF_LENGTH  (4 * 16384)                         // = 65536

#define K_MIN_SCORE         10
#define K_MAX_CANDIDATES    120
#define K_LDPC_ITERS        20
#define K_MAX_MESSAGES      50
#define K_FREQ_OSR          2
#define K_TIME_OSR          2
#define K_FSK_DEV           6.25

#define LOG_LEVEL LOG_INFO  // FIXME



// FIXME
const int kMin_score = 10;  // Minimum sync score threshold for candidates
const int kMax_candidates = 120;
const int kLDPC_iterations = 20;
const int kMax_decoded_messages = 50;
const int kFreq_osr = 2;
const int kTime_osr = 2;
const float kFSK_dev = 6.25f;  // tone deviation in Hz and symbol rate


/* Global declaration for these structs */
struct receiver_state   rx_state;
struct receiver_options rx_options;
struct decoder_options  dec_options;
struct decoder_results  dec_results[50];
static rtlsdr_dev_t *rtl_device = NULL;


/* Thread stuff for separate decoding */
struct decoder_state {
    pthread_t        thread;
    pthread_attr_t   tattr;

    pthread_rwlock_t rw;
    pthread_cond_t   ready_cond;
    pthread_mutex_t  ready_mutex;
};
struct decoder_state dec;


/* Thread stuff for separate RX (blocking function) */
struct dongle_state {
    pthread_t        thread;
};
struct dongle_state dongle;


/* Callback for each buffer received */
static void rtlsdr_callback(unsigned char *samples,
                            uint32_t samples_count,
                            void *ctx) {
    /* FIR compensation filter coefs
       Using : Octave/MATLAB code for generating compensation FIR coefficients
       URL : https://github.com/WestCoastDSP/CIC_Octave_Matlab
     */
    // FIXME -- Update new coefs !!!
    const static float zCoef[33] = {
        0.000358366, -0.003301180, -0.000609166, 0.006729260,
        0.001403240, -0.014276600, -0.003107740, 0.027349800,
        0.006522290, -0.048282000, -0.013725400, 0.082058900,
        0.031488100, -0.142079000, -0.091377400, 0.271349000,
        0.500000000,
        0.271349000, -0.091377400, -0.142079000, 0.031488100,
        0.082058900, -0.013725400, -0.048282000, 0.006522290,
        0.027349800, -0.003107740, -0.014276600, 0.001403240,
        0.006729260, -0.000609166, -0.003301180, 0.000358366
    };
    int8_t *sigIn = (int8_t*)samples;
    uint32_t sigLenght = samples_count;
    static uint32_t decimationIndex = 0;

    /* CIC buffers */
    static int32_t  Ix1, Ix2, Qx1, Qx2;
    static int32_t  Iy1, It1y, It1z, Qy1, Qt1y, Qt1z;
    static int32_t  Iy2, It2y, It2z, Qy2, Qt2y, Qt2z;

    /* FIR compensation filter buffers */
    static float firI[32], firQ[32];

    float Isum, Qsum;

    /* Convert unsigned to signed */
    for (uint32_t i = 0; i < sigLenght; i++)
        // XOR with a binary mask to flip the first bit (sign)
        sigIn[i] ^= 0x80;

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
    for (uint32_t i = 0; i < sigLenght; i+=8) {
        tmp = -sigIn[i+3];
        sigIn[i+3] = sigIn[i+2];
        sigIn[i+2] = tmp;

        sigIn[i+4] = -sigIn[i+4];
        sigIn[i+5] = -sigIn[i+5];

        tmp = -sigIn[i+6];
        sigIn[i+6] = sigIn[i+7];
        sigIn[i+7] = tmp;
    }

    /* CIC decimator (N=2)
       Info: * Understanding CIC Compensation Filters
               https://www.altera.com/en_US/pdfs/literature/an/an455.pdf
             * Understanding cascaded integrator-comb filters
               http://www.embedded.com/design/configurable-systems/4006446/Understanding-cascaded-integrator-comb-filters
    */
    for (int32_t i = 0; i < sigLenght/2; i++) {
        /* Integrator stages (N=2) */
        Ix1 += (int32_t)sigIn[i*2];
        Qx1 += (int32_t)sigIn[i*2+1];
        Ix2 += Ix1;
        Qx2 += Qx1;

        /* Decimation R=6400 */
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

        // FIXME/TODO : could be made with int32_t (8 bits, 20 bits)
        /* FIR compensation filter */
        Isum = 0.0, Qsum = 0.0;
        for (uint32_t j = 0; j < 32; j++) {
            Isum += firI[j] * zCoef[j];
            Qsum += firQ[j] * zCoef[j];
            if (j < 31) {
                firI[j] = firI[j+1];
                firQ[j] = firQ[j+1];
            }
        }
        firI[31] = (float)Iy2;
        firQ[31] = (float)Qy2;
        Isum += firI[31]*zCoef[32];
        Qsum += firQ[31]*zCoef[32];

        /* Save the result in the buffer */
        if (rx_state.iqIndex < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE)) {
            /* Lock the buffer during writing */
            pthread_rwlock_wrlock(&dec.rw);
            rx_state.iSamples[rx_state.iqIndex] = Isum;
            rx_state.qSamples[rx_state.iqIndex] = Qsum;
            pthread_rwlock_unlock(&dec.rw);
            rx_state.iqIndex++;
        } else {
            if (rx_state.decode_flag == false) {
                /* Send a signal to the other thread to start the decoding */
                pthread_mutex_lock(&dec.ready_mutex);
                pthread_cond_signal(&dec.ready_cond);
                pthread_mutex_unlock(&dec.ready_mutex);
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


void postSpots(uint32_t n_results) {
    CURL *curl;
    CURLcode res;
    char url[256];

    time_t rawtime;
    time(&rawtime);
    struct tm *gtm = gmtime(&rawtime);

    for (uint32_t i=0; i < n_results; i++) {
        snprintf(url, sizeof(url)-1, "http://wsprnet.org/post?function=wspr&rcall=%s&rgrid=%s&rqrg=%.6f&date=%s&time=%s&sig=%.0f&dt=%.1f&tqrg=%.6f&tcall=%s&tgrid=%s&dbm=%s&version=0.2r_wsprd&mode=2",
            dec_options.rcall,
            dec_options.rloc,
            dec_results[i].freq,
            dec_options.date,
            dec_options.uttime,
            dec_results[i].snr,
            dec_results[i].dt,
            dec_results[i].freq,
            dec_results[i].call,
            dec_results[i].loc,
            dec_results[i].pwr);

        printf("Spot :  %04d-%02d-%02d %02d:%02d:%02d %6.2f %6.2f %10.6f %2d %7s %6s %2s\n",
                gtm->tm_year + 1900,
                gtm->tm_mon + 1,
                gtm->tm_mday,
                gtm->tm_hour,
                gtm->tm_min,
                gtm->tm_sec,
                dec_results[i].snr,
                dec_results[i].dt,
                dec_results[i].freq,
                (int)dec_results[i].drift,
                dec_results[i].call,
                dec_results[i].loc,
                dec_results[i].pwr);

        curl = curl_easy_init();
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url);
            curl_easy_setopt(curl, CURLOPT_NOBODY, 1);
            res = curl_easy_perform(curl);

            if (res != CURLE_OK)
                fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

            curl_easy_cleanup(curl);
        }
    }

    if (n_results == 0) {
        printf("No spot %04d-%02d-%02d %02d:%02dz\n",
            gtm->tm_year + 1900,
            gtm->tm_mon + 1,
            gtm->tm_mday,
            gtm->tm_hour,
            gtm->tm_min);
    }
}


static void *decoder(void *arg) {
    /* FT8 decoder use buffers of 180000 samples
       (15 sec max @ 12000sps = 180000 samples)
    */
    static float iSamples[SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE] = {0};
    static float qSamples[SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE] = {0};
    static uint32_t samples_len;
    int32_t n_results = 0;

    while (!rx_state.exit_flag) {
        pthread_mutex_lock(&dec.ready_mutex);
        pthread_cond_wait(&dec.ready_cond, &dec.ready_mutex);
        pthread_mutex_unlock(&dec.ready_mutex);

        if (rx_state.exit_flag)  // Abord case, final sig
            break;

        /* Lock the buffer access and make a local copy */
        pthread_rwlock_wrlock(&dec.rw);
        memcpy(iSamples, rx_state.iSamples, rx_state.iqIndex * sizeof(float));
        memcpy(qSamples, rx_state.qSamples, rx_state.iqIndex * sizeof(float));
        samples_len = rx_state.iqIndex;  // Overkill ?
        pthread_rwlock_unlock(&dec.rw);

        /* Date and time will be updated/overload during the search & decoding process
           Make a simple copy
        */
        memcpy(dec_options.date, rx_options.date, sizeof(rx_options.date));
        memcpy(dec_options.uttime, rx_options.uttime, sizeof(rx_options.uttime));

        /* Search & decode the signal */
        ft8_subsystem(iSamples, qSamples, samples_len, dec_options, dec_results, &n_results);
        postSpots(n_results);
    }
    pthread_exit(NULL);
}


double atofs(char *s) {
    /* standard suffixes */
    char last;
    uint32_t len;
    double suff = 1.0;
    len = strlen(s);
    last = s[len-1];
    s[len-1] = '\0';

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
            s[len-1] = last;
            return suff;
    }
    s[len-1] = last;
    return atof(s);
}


int32_t parse_u64(char* s, uint64_t* const value) {
    uint_fast8_t base = 10;
    char* s_end;
    uint64_t u64_value;

    if (strlen(s) > 2) {
        if (s[0] == '0') {
            if ( (s[1] == 'x') || (s[1] == 'X') ) {
                base = 16;
                s += 2;
            } else if ( (s[1] == 'b') || (s[1] == 'B') ) {
                base = 2;
                s += 2;
            }
        }
    }

    s_end = s;
    u64_value = strtoull(s, &s_end, base);
    if ( (s != s_end) && (*s_end == 0) ) {
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
    rx_options.gain = 290;
    rx_options.autogain = 0;
    rx_options.ppm = 0;
    rx_options.shift = 0;
    rx_options.directsampling = 0;
    rx_options.maxloop = 0;
    rx_options.device = 0;
}


void sigint_callback_handler(int signum) {
    fprintf(stdout, "Caught signal %d\n", signum);
    rx_state.exit_flag = true;
}


int32_t readfile(float *iSamples, float *qSamples, char *filename) {
    FILE* fd = NULL;
    float filebuffer[2*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE];

    fd = fopen(filename, "rb");
    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 1;
    }

    int32_t res = fseek(fd, 26, SEEK_SET);
    if (res) {
        fprintf(stderr, "Cannot set file offset...\n");
        return 1;
    }

    int32_t nread = fread(filebuffer, sizeof(float), 2*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE, fd);
    if (nread != 2*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE) {
        fprintf(stderr, "Cannot read all the data!\n");
        return 1;
    }

    for (int32_t i=0; i < SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE; i++) {
        iSamples[i] =  filebuffer[2*i];
        qSamples[i] = -filebuffer[2*i+1];
    }

    fclose(fd);

    return 0;
}


int32_t writefile(float *iSamples, float *qSamples, char *filename, uint32_t type, double freq) {
    FILE* fd = NULL;
    char info[15] = {};  // Info descriptor, not used for now

    float filebuffer[2*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE];
    for (int32_t i=0; i < SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE; i++) {
        filebuffer[2*i]   =  iSamples[i];
        filebuffer[2*i+1] = -qSamples[i];
    }

    fd = fopen(filename, "wb");
    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 1;
    }

    // Header
    fwrite(&info, sizeof(char), 14, fd);
    fwrite(&type, sizeof(uint32_t), 1, fd);
    fwrite(&freq, sizeof(double), 1, fd);

    int32_t nwrite = fwrite(filebuffer, sizeof(float), 2*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE, fd);
    if (nwrite != 2*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE) {
        fprintf(stderr, "Cannot write all the data!\n");
        return 1;
    }

    fclose(fd);

    return 0;
}


void usage(void) {
    fprintf(stderr,
            "rtlsdr_ft8d, a simple FT8 daemon for RTL receivers\n\n"
            "Use:\trtlsdr_ft8d -f frequency -c callsign -l locator [options]\n"
            "\t-f dial frequency [(,k,M) Hz] or band string\n"
            "\t   If band string is used, the default dial frequency will used.\n"
            "\t   Bands: LF LF-15 MF MF-15 160m 160m-15 80m 60m 40m 30m 20m 17m 15m 12m 10m 6m 4m 2m 1m25 70cm 23cm\n"
            "\t   ('-15' suffix indicates the WSPR-15 region of band.)\n"
            "\t-c your callsign (12 chars max)\n"
            "\t-l your locator grid (6 chars max)\n"
            "Receiver extra options:\n"
            "\t-g gain [0-49] (default: 29)\n"
            "\t-a auto gain (default: off)\n"
            "\t-o frequency offset (default: 0)\n"
            "\t-p crystal correction factor (ppm) (default: 0)\n"
            "\t-u upconverter (default: 0, example: 125M)\n"
            "\t-d direct dampling [0,1,2] (default: 0, 1 for I input, 2 for Q input)\n"
            "\t-n max iterations (default: 0 = infinite loop)\n"
            "\t-i device index (in case of multiple receivers, default: 0)\n"
            "Example:\n"
            "\trtlsdr_ft8d -f 2m -c A1XYZ -l AB12cd -g 29 -o -4200\n");
    exit(1);
}


int main(int argc, char** argv) {
    uint32_t opt;

    int32_t  rtl_result;
    int32_t  rtl_count;
    char     rtl_vendor[256], rtl_product[256], rtl_serial[256];

    initrx_options();

    /* RX buffer allocation */
    rx_state.iSamples = malloc(sizeof(float)*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE);
    rx_state.qSamples = malloc(sizeof(float)*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE);

    /* Stop condition setup */
    rx_state.exit_flag   = false;
    rx_state.decode_flag = false;
    uint32_t nLoop = 0;

    if (argc <= 1)
        usage();

    while ((opt = getopt(argc, argv, "f:c:l:g:a:o:p:u:d:n:i:H:Q:S")) != -1) {
        switch (opt) {
        case 'f':  // Frequency
            if (!strcasecmp(optarg, "LF")) {
                rx_options.dialfreq = 136000.0;
            } else if (!strcasecmp(optarg, "LF-15")) {
                rx_options.dialfreq = 136112.5;
            } else if (!strcasecmp(optarg, "MF")) {
                rx_options.dialfreq = 474200.0;
            } else if (!strcasecmp(optarg, "MF-15")) {
                rx_options.dialfreq = 474312.5;
            } else if (!strcasecmp(optarg, "160m")) {
                rx_options.dialfreq = 1836600.0;
            } else if (!strcasecmp(optarg, "160m-15")) {
                rx_options.dialfreq = 1838212.5;
            } else if (!strcasecmp(optarg, "80m")) {
                rx_options.dialfreq = 3592600.0;
            } else if (!strcasecmp(optarg, "60m")) {
                rx_options.dialfreq = 5287200.0;
            } else if (!strcasecmp(optarg, "40m")) {
                rx_options.dialfreq = 7038600.0;
            } else if (!strcasecmp(optarg, "30m")) {
                rx_options.dialfreq = 10138700.0;
            } else if (!strcasecmp(optarg, "20m")) {
                rx_options.dialfreq = 14095600.0;
            } else if (!strcasecmp(optarg, "17m")) {
                rx_options.dialfreq = 18104600.0;
            } else if (!strcasecmp(optarg, "15m")) {
                rx_options.dialfreq = 21094600.0;
            } else if (!strcasecmp(optarg, "12m")) {
                rx_options.dialfreq = 24924600.0;
            } else if (!strcasecmp(optarg, "10m")) {
                rx_options.dialfreq = 28124600.0;
            } else if (!strcasecmp(optarg, "6m")) {
                rx_options.dialfreq = 50293000.0;
            } else if (!strcasecmp(optarg, "4m")) {
                rx_options.dialfreq = 70091000.0;
            } else if (!strcasecmp(optarg, "2m")) {
                rx_options.dialfreq = 144489000.0;
            } else if (!strcasecmp(optarg, "1m25")) {
                rx_options.dialfreq = 222280000.0;
            } else if (!strcasecmp(optarg, "70cm")) {
                rx_options.dialfreq = 432300000.0;
            } else if (!strcasecmp(optarg, "23cm")) {
                rx_options.dialfreq = 1296500000.0;
            } else {
            // Not a string. Parse it as a double.
            rx_options.dialfreq = (uint32_t)atofs(optarg);
            }
            break;
        case 'c':  // Callsign
            snprintf(dec_options.rcall, sizeof(dec_options.rcall), "%.12s", optarg);
            break;
        case 'l':  // Locator / Grid
            snprintf(dec_options.rloc, sizeof(dec_options.rloc),  "%.6s", optarg);
            break;
        case 'g':  // Small signal amplifier gain
            rx_options.gain = atoi(optarg);
            if (rx_options.gain < 0) rx_options.gain = 0;
            if (rx_options.gain > 49) rx_options.gain = 49;
            rx_options.gain *= 10;
            break;
        case 'a':  // Auto gain
            rx_options.autogain = atoi(optarg);
            if (rx_options.autogain < 0) rx_options.autogain = 0;
            if (rx_options.autogain > 1) rx_options.autogain = 1;
            break;
        case 'o':  // Fine frequency correction
            rx_options.shift = atoi(optarg);
            break;
        case 'p':
            rx_options.ppm = atoi(optarg);
            break;
        case 'u':  // Upconverter frequency
            rx_options.upconverter = (uint32_t)atofs(optarg);
            break;
        case 'd':  // Direct Sampling
            rx_options.directsampling = (uint32_t)atofs(optarg);
            break;
        case 'n':  // Stop after n iterations
            rx_options.maxloop = (uint32_t)atofs(optarg);
            break;
        case 'i':  // Select the device to use
            rx_options.device = (uint32_t)atofs(optarg);
            break;
        default:
            usage();
            break;
        }
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

    /* If something goes wrong... */
    signal(SIGINT, &sigint_callback_handler);
    signal(SIGTERM, &sigint_callback_handler);
    signal(SIGILL, &sigint_callback_handler);
    signal(SIGFPE, &sigint_callback_handler);
    signal(SIGSEGV, &sigint_callback_handler);
    signal(SIGABRT, &sigint_callback_handler);

    /* Init & parameter the device */
    rtl_count = rtlsdr_get_device_count();
    if (!rtl_count) {
        fprintf(stderr, "No supported devices found\n");
        return EXIT_FAILURE;
    }


    fprintf(stderr, "Found %d device(s):\n", rtl_count);
    for (uint32_t i=0; i < rtl_count; i++) {
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


    rtl_result = rtlsdr_set_center_freq(rtl_device, rx_options.realfreq + FS4_RATE + 1500);
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


    /* Print used parameter */
    time_t rawtime;
    time(&rawtime);
    struct tm *gtm = gmtime(&rawtime);

    printf("\nStarting rtlsdr-wsprd (%04d-%02d-%02d, %02d:%02dz) -- Version 0.3\n",
           gtm->tm_year + 1900, gtm->tm_mon + 1, gtm->tm_mday, gtm->tm_hour, gtm->tm_min);
    printf("  Callsign     : %s\n", dec_options.rcall);
    printf("  Locator      : %s\n", dec_options.rloc);
    printf("  Dial freq.   : %d Hz\n", rx_options.dialfreq);
    printf("  Real freq.   : %d Hz\n", rx_options.realfreq);
    printf("  PPM factor   : %d\n", rx_options.ppm);
    if (rx_options.autogain)
        printf("  Auto gain    : enable\n");
    else
        printf("  Gain         : %d dB\n", rx_options.gain/10);


    /* Time alignment stuff */
    struct timeval lTime;
    gettimeofday(&lTime, NULL);
    uint32_t sec   = lTime.tv_sec % 120;
    uint32_t usec  = sec * 1000000 + lTime.tv_usec;
    uint32_t uwait = 120000000 - usec;
    printf("Wait for time sync (start in %d sec)\n\n", uwait/1000000);
    printf("              Date  Time(z)    SNR     DT       Freq Dr    Call    Loc Pwr\n");

    /* Prepare a low priority param for the decoder thread */
    struct sched_param param;
    pthread_attr_init(&dec.tattr);
    pthread_attr_setschedpolicy(&dec.tattr, SCHED_RR);
    pthread_attr_getschedparam(&dec.tattr, &param);
    param.sched_priority = 90;  // = sched_get_priority_min();
    pthread_attr_setschedparam(&dec.tattr, &param);

    /* Create a thread and stuff for separate decoding
       Info : https://computing.llnl.gov/tutorials/pthreads/
    */
    pthread_rwlock_init(&dec.rw, NULL);
    pthread_cond_init(&dec.ready_cond, NULL);
    pthread_mutex_init(&dec.ready_mutex, NULL);
    pthread_create(&dongle.thread, NULL, rtlsdr_rx, NULL);
    pthread_create(&dec.thread, &dec.tattr, decoder, NULL);


    /* Main loop : Wait, read, decode */
    while (!rx_state.exit_flag && !(rx_options.maxloop && (nLoop >= rx_options.maxloop))) {
        /* Wait for time Sync on 2 mins */
        gettimeofday(&lTime, NULL);
        sec   = lTime.tv_sec % 120;
        usec  = sec * 1000000 + lTime.tv_usec;
        uwait = 120000000 - usec + 10000;  // Adding 10ms, to be sure to reach this next minute
        usleep(uwait);

        /* Use the Store the date at the begin of the frame */
        time(&rawtime);
        gtm = gmtime(&rawtime);
        // FIXME: Complier warning about mixing int & date
        snprintf(rx_options.date, sizeof(rx_options.date), "%02d%02d%02d", gtm->tm_year - 100, gtm->tm_mon + 1, gtm->tm_mday);
        snprintf(rx_options.uttime, sizeof(rx_options.uttime), "%02d%02d", gtm->tm_hour, gtm->tm_min);

        /* Start to store the samples */
        initSampleStorage();

        while ( (rx_state.exit_flag == false) &&
                (rx_state.iqIndex < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE) ) ) {
            usleep(250000);
        }
        nLoop++;
    }

    /* Stop the RX and free the blocking function */
    rtlsdr_cancel_async(rtl_device);

    /* Close the RTL device */
    rtlsdr_close(rtl_device);

    printf("Bye!\n");

    /* Wait the thread join (send a signal before to terminate the job) */
    pthread_mutex_lock(&dec.ready_mutex);
    pthread_cond_signal(&dec.ready_cond);
    pthread_mutex_unlock(&dec.ready_mutex);
    pthread_join(dec.thread, NULL);
    pthread_join(dongle.thread, NULL);

    /* Destroy the lock/cond/thread */
    pthread_rwlock_destroy(&dec.rw);
    pthread_cond_destroy(&dec.ready_cond);
    pthread_mutex_destroy(&dec.ready_mutex);
    pthread_exit(NULL);

    return EXIT_SUCCESS;
}

/* FT8 decoding stuff */

float hann_i(int i, int N) {
    float x = sinf((float)M_PI * i / N);
    return x * x;
}

// Compute FFT magnitudes (log power) for each timeslot in the signal
void extract_power(float *idat,
                   float *qdat,
                   waterfall_t* power,
                   int block_size) {
    const int subblock_size = block_size / power->time_osr;
    const int nfft = block_size * power->freq_osr;
    const float fft_norm = 2.0f / nfft;
    const int len_window = 1.8f * block_size; // hand-picked and optimized

    float window[nfft];
    for (int i=0; i < nfft; ++i) {
        // window[i] = 1;
        // window[i] = hann_i(i, nfft);
        // window[i] = blackman_i(i, nfft);
        // window[i] = hamming_i(i, nfft);
        window[i] = (i < len_window) ? hann_i(i, len_window) : 0;
    }

    size_t fft_work_size;
    kiss_fft_alloc(nfft, 0, 0, &fft_work_size);

    fprintf(stderr, "Block size = %d\n", block_size);
    fprintf(stderr, "Subblock size = %d\n", subblock_size);
    fprintf(stderr, "N_FFT = %d\n", nfft);
    fprintf(stderr, "FFT work area = %lu\n", fft_work_size);

    void* fft_work = malloc(fft_work_size);
    kiss_fft_cfg fft_cfg = kiss_fft_alloc(nfft, 0, fft_work, &fft_work_size);

    int offset = 0;
    float max_mag = -120.0f;
    for (int idx_block = 0; idx_block < power->num_blocks; ++idx_block) {
        // Loop over two possible time offsets (0 and block_size/2)
        for (int time_sub = 0; time_sub < power->time_osr; ++time_sub) {
            kiss_fft_cpx cx_in[nfft];
            kiss_fft_cpx cx_out[nfft];
            float mag_db[nfft];

            // Extract windowed signal block
            for (int pos = 0; pos < nfft; ++pos) {
                cx_in[pos].r = window[pos] * idat[(idx_block * block_size) + (time_sub * subblock_size) + pos];
                cx_in[pos].i = window[pos] * qdat[(idx_block * block_size) + (time_sub * subblock_size) + pos];
            }

            kiss_fft(fft_cfg, cx_in, cx_out);

            // Compute log magnitude in decibels
            for (int idx_bin = 0; idx_bin < nfft; ++idx_bin) {
                float mag2 = (cx_out[idx_bin].i * cx_out[idx_bin].i) + (cx_out[idx_bin].r * cx_out[idx_bin].r);
                mag_db[idx_bin] = 10.0f * log10f(1E-12f + mag2 * fft_norm * fft_norm);
            }

            // Loop over two possible frequency bin offsets (for averaging)
            for (int freq_sub = 0; freq_sub < power->freq_osr; ++freq_sub) {
                for (int pos = 0; pos < power->num_bins; ++pos) {
                    float db = mag_db[pos * power->freq_osr + freq_sub];
                    // Scale decibels to unsigned 8-bit range and clamp the value
                    // Range 0-240 covers -120..0 dB in 0.5 dB steps
                    int scaled = (int)(2 * db + 240);

                    power->mag[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
                    ++offset;

                    if (db > max_mag)
                        max_mag = db;
                }
            }
        }
    }
    fprintf(stderr, "Max magnitude: %.1f dB\n", max_mag);
    free(fft_work);
}


void ft8_subsystem(float *idat, 
                   float *qdat, 
                   uint32_t npoints, 
                   struct decoder_options options, 
                   struct decoder_results *decodes, 
                   int32_t *n_results) {
    // Inputs
    int sample_rate = SIGNAL_SAMPLE_RATE;
    int num_samples = SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE; 
    assert (num_samples == npoints);

    // Compute DSP parameters that depend on the sample rate
    const int num_bins = (int)(sample_rate / (2 * kFSK_dev)); // number bins of FSK tone width that the spectrum can be divided into
    const int block_size = (int)(sample_rate / kFSK_dev); // samples corresponding to one FSK symbol
    const int subblock_size = block_size / kTime_osr;
    const int nfft = block_size * kFreq_osr;
    const int num_blocks = (num_samples - nfft + subblock_size) / block_size;

    fprintf(stderr, "Sample rate %d Hz, %d blocks, %d bins\n", sample_rate, num_blocks, num_bins);

    // Compute FFT over the whole signal and store it
    uint8_t mag_power[num_blocks * kFreq_osr * kTime_osr * num_bins];
    waterfall_t power = {
        .num_blocks = num_blocks,
        .num_bins = num_bins,
        .time_osr = kTime_osr,
        .freq_osr = kFreq_osr,
        .mag = mag_power
    };
    extract_power(idat, qdat, &power, block_size);

    // Find top candidates by Costas sync score and localize them in time and frequency
    candidate_t candidate_list[kMax_candidates];
    int num_candidates = ft8_find_sync(&power, kMax_candidates, candidate_list, kMin_score);

    // Hash table for decoded messages (to check for duplicates)
    int num_decoded = 0;
    message_t decoded[kMax_decoded_messages];
    message_t* decoded_hashtable[kMax_decoded_messages];

    // Initialize hash table pointers
    for (int i = 0; i < kMax_decoded_messages; ++i) {
        decoded_hashtable[i] = NULL;
    }

    // Go over candidates and attempt to decode messages
    for (int idx = 0; idx < num_candidates; ++idx) {
        const candidate_t* cand = &candidate_list[idx];
        if (cand->score < kMin_score)
            continue;

        float freq_hz = (cand->freq_offset + (float)cand->freq_sub / kFreq_osr) * kFSK_dev;
        float time_sec = (cand->time_offset + (float)cand->time_sub / kTime_osr) / kFSK_dev;

        message_t message;
        decode_status_t status;
        if (!ft8_decode(&power, cand, &message, kLDPC_iterations, &status)) {
            if (status.ldpc_errors > 0) {
                fprintf(stderr, "LDPC decode: %d errors\n", status.ldpc_errors);
            }
            else if (status.crc_calculated != status.crc_extracted) {
                fprintf(stderr, "CRC mismatch!\n");
            }
            else if (status.unpack_status != 0) {
                fprintf(stderr, "Error while unpacking!\n");
            }
            continue;
        }

        fprintf(stderr, "Checking hash table for %4.1fs / %4.1fHz [%d]...\n", time_sec, freq_hz, cand->score);
        int idx_hash = message.hash % kMax_decoded_messages;
        bool found_empty_slot = false;
        bool found_duplicate = false;
        do {
            if (decoded_hashtable[idx_hash] == NULL) {
                fprintf(stderr, "Found an empty slot\n");
                found_empty_slot = true;
            }
            else if ((decoded_hashtable[idx_hash]->hash == message.hash) && (0 == strcmp(decoded_hashtable[idx_hash]->text, message.text))) {
                fprintf(stderr, "Found a duplicate [%s]\n", message.text);
                found_duplicate = true;
            }
            else {
                fprintf(stderr, "Hash table clash!\n");
                // Move on to check the next entry in hash table
                idx_hash = (idx_hash + 1) % kMax_decoded_messages;
            }
        } while (!found_empty_slot && !found_duplicate);

        if (found_empty_slot) {
            // Fill the empty hashtable slot
            memcpy(&decoded[idx_hash], &message, sizeof(message));
            decoded_hashtable[idx_hash] = &decoded[idx_hash];
            ++num_decoded;

            // Fake WSJT-X-like output for now
            // int snr = 0;  // TODO: compute SNR
            printf("000000 %3d %+4.2f %4.0f ~  %s\n", cand->score, time_sec, freq_hz, message.text);
        }
    }
    fprintf(stderr, "Decoded %d messages\n", num_decoded);
}
