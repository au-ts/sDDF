#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SDDF_SND_D_OUTPUT = 0,
    SDDF_SND_D_INPUT
} sddf_snd_direction_t;

// Supported PCM sample formats
typedef enum {
    // Analog formats (width / physical width)
    SDDF_SND_PCM_FMT_IMA_ADPCM = 0, /* 4 / 4 bits */
    SDDF_SND_PCM_FMT_MU_LAW, /* 8 / 8 bits */
    SDDF_SND_PCM_FMT_A_LAW, /* 8 / 8 bits */
    SDDF_SND_PCM_FMT_S8, /* 8 / 8 bits */
    SDDF_SND_PCM_FMT_U8, /* 8 / 8 bits */
    SDDF_SND_PCM_FMT_S16, /* 16 / 16 bits */
    SDDF_SND_PCM_FMT_U16, /* 16 / 16 bits */
    SDDF_SND_PCM_FMT_S18_3, /* 18 / 24 bits */
    SDDF_SND_PCM_FMT_U18_3, /* 18 / 24 bits */
    SDDF_SND_PCM_FMT_S20_3, /* 20 / 24 bits */
    SDDF_SND_PCM_FMT_U20_3, /* 20 / 24 bits */
    SDDF_SND_PCM_FMT_S24_3, /* 24 / 24 bits */
    SDDF_SND_PCM_FMT_U24_3, /* 24 / 24 bits */
    SDDF_SND_PCM_FMT_S20, /* 20 / 32 bits */
    SDDF_SND_PCM_FMT_U20, /* 20 / 32 bits */
    SDDF_SND_PCM_FMT_S24, /* 24 / 32 bits */
    SDDF_SND_PCM_FMT_U24, /* 24 / 32 bits */
    SDDF_SND_PCM_FMT_S32, /* 32 / 32 bits */
    SDDF_SND_PCM_FMT_U32, /* 32 / 32 bits */
    SDDF_SND_PCM_FMT_FLOAT, /* 32 / 32 bits */
    SDDF_SND_PCM_FMT_FLOAT64, /* 64 / 64 bits */
    // Digital formats (width / physical width)
    SDDF_SND_PCM_FMT_DSD_U8, /* 8 / 8 bits */
    SDDF_SND_PCM_FMT_DSD_U16, /* 16 / 16 bits */
    SDDF_SND_PCM_FMT_DSD_U32, /* 32 / 32 bits */
    SDDF_SND_PCM_FMT_IEC958_SUBFRAME /* 32 / 32 bits */
} sddf_snd_pcm_fmt_t;

// Supported PCM frame rates
typedef enum {
    SDDF_SND_PCM_RATE_5512 = 0,
    SDDF_SND_PCM_RATE_8000,
    SDDF_SND_PCM_RATE_11025,
    SDDF_SND_PCM_RATE_16000,
    SDDF_SND_PCM_RATE_22050,
    SDDF_SND_PCM_RATE_32000,
    SDDF_SND_PCM_RATE_44100,
    SDDF_SND_PCM_RATE_48000,
    SDDF_SND_PCM_RATE_64000,
    SDDF_SND_PCM_RATE_88200,
    SDDF_SND_PCM_RATE_96000,
    SDDF_SND_PCM_RATE_176400,
    SDDF_SND_PCM_RATE_192000,
    SDDF_SND_PCM_RATE_384000
} sddf_snd_pcm_rate_t;

typedef struct sddf_snd_pcm_info {
    uint64_t formats; /* 1 << SDDF_SND_PCM_FMT_XXX */
    uint64_t rates;   /* 1 << SDDF_SND_PCM_RATE_XXX */
    uint8_t direction;
    uint8_t channels_min;
    uint8_t channels_max;
} sddf_snd_pcm_info_t;

#define SDDF_SND_MAX_STREAM_COUNT 32

typedef struct sddf_snd_shared_state {
    uint32_t streams;
    sddf_snd_pcm_info_t stream_info[SDDF_SND_MAX_STREAM_COUNT];
} sddf_snd_shared_state_t;
