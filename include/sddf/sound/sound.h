/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SOUND_D_OUTPUT = 0,
    SOUND_D_INPUT
} sound_direction_t;

// Supported PCM sample formats
typedef enum {
    // Analog formats (width / physical width)
    SOUND_PCM_FMT_IMA_ADPCM = 0, /* 4 / 4 bits */
    SOUND_PCM_FMT_MU_LAW, /* 8 / 8 bits */
    SOUND_PCM_FMT_A_LAW, /* 8 / 8 bits */
    SOUND_PCM_FMT_S8, /* 8 / 8 bits */
    SOUND_PCM_FMT_U8, /* 8 / 8 bits */
    SOUND_PCM_FMT_S16, /* 16 / 16 bits */
    SOUND_PCM_FMT_U16, /* 16 / 16 bits */
    SOUND_PCM_FMT_S18_3, /* 18 / 24 bits */
    SOUND_PCM_FMT_U18_3, /* 18 / 24 bits */
    SOUND_PCM_FMT_S20_3, /* 20 / 24 bits */
    SOUND_PCM_FMT_U20_3, /* 20 / 24 bits */
    SOUND_PCM_FMT_S24_3, /* 24 / 24 bits */
    SOUND_PCM_FMT_U24_3, /* 24 / 24 bits */
    SOUND_PCM_FMT_S20, /* 20 / 32 bits */
    SOUND_PCM_FMT_U20, /* 20 / 32 bits */
    SOUND_PCM_FMT_S24, /* 24 / 32 bits */
    SOUND_PCM_FMT_U24, /* 24 / 32 bits */
    SOUND_PCM_FMT_S32, /* 32 / 32 bits */
    SOUND_PCM_FMT_U32, /* 32 / 32 bits */
    SOUND_PCM_FMT_FLOAT, /* 32 / 32 bits */
    SOUND_PCM_FMT_FLOAT64, /* 64 / 64 bits */
    // Digital formats (width / physical width)
    SOUND_PCM_FMT_DSD_U8, /* 8 / 8 bits */
    SOUND_PCM_FMT_DSD_U16, /* 16 / 16 bits */
    SOUND_PCM_FMT_DSD_U32, /* 32 / 32 bits */
    SOUND_PCM_FMT_IEC958_SUBFRAME /* 32 / 32 bits */
} sound_pcm_fmt_t;

// Supported PCM frame rates
typedef enum {
    SOUND_PCM_RATE_5512 = 0,
    SOUND_PCM_RATE_8000,
    SOUND_PCM_RATE_11025,
    SOUND_PCM_RATE_16000,
    SOUND_PCM_RATE_22050,
    SOUND_PCM_RATE_32000,
    SOUND_PCM_RATE_44100,
    SOUND_PCM_RATE_48000,
    SOUND_PCM_RATE_64000,
    SOUND_PCM_RATE_88200,
    SOUND_PCM_RATE_96000,
    SOUND_PCM_RATE_176400,
    SOUND_PCM_RATE_192000,
    SOUND_PCM_RATE_384000
} sound_pcm_rate_t;

typedef struct sound_pcm_info {
    uint64_t formats; /* 1 << SOUND_PCM_FMT_XXX */
    uint64_t rates;   /* 1 << SOUND_PCM_RATE_XXX */
    uint8_t direction;
    uint8_t channels_min;
    uint8_t channels_max;
} sound_pcm_info_t;

#define SOUND_MAX_STREAM_COUNT 32

typedef struct sound_shared_state {
    bool ready;
    uint32_t streams;
    sound_pcm_info_t stream_info[SOUND_MAX_STREAM_COUNT];
} sound_shared_state_t;
