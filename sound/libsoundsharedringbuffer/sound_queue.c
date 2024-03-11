#include <sddf/sound/sound_queue.h>
#include <sddf/util/fence.h>

void sddf_snd_ring_init(sddf_snd_ring_state_t *ring_state, uint32_t buffer_count)
{
    ring_state->read_idx = 0;
    ring_state->write_idx = 0;
    ring_state->size = buffer_count;
}

void sddf_snd_rings_init_default(sddf_snd_rings_t *rings)
{
    sddf_snd_ring_init(&rings->cmd_req->state,  SDDF_SND_NUM_BUFFERS);
    sddf_snd_ring_init(&rings->cmd_res->state, SDDF_SND_NUM_BUFFERS);
    sddf_snd_ring_init(&rings->pcm_req->state, SDDF_SND_NUM_BUFFERS);
    sddf_snd_ring_init(&rings->pcm_res->state, SDDF_SND_NUM_BUFFERS);
}

bool sddf_snd_ring_empty(sddf_snd_ring_state_t *ring_state)
{
    return ring_state->write_idx == ring_state->read_idx;
    // return !((ring_state->write_idx - ring_state->read_idx) % ring_state->size);
}

bool sddf_snd_ring_full(sddf_snd_ring_state_t *ring_state)
{
    return (ring_state->write_idx - ring_state->read_idx + 1) == ring_state->size;
    // return !((ring_state->write_idx - ring_state->read_idx + 1) % ring_state->size);
}

int sddf_snd_ring_size(sddf_snd_ring_state_t *ring_state)
{
    return ring_state->write_idx - ring_state->read_idx;
}

int sddf_snd_enqueue_cmd(sddf_snd_cmd_ring_t *ring,
                         const sddf_snd_cmd_t *command)
{
    if (sddf_snd_ring_full(&ring->state)) {
        return -1;
    }

    sddf_snd_cmd_t *dest = &ring->buffers[ring->state.write_idx % ring->state.size];

    dest->code = command->code;
    dest->cookie = command->cookie;
    dest->stream_id = command->stream_id;
    dest->set_params = command->set_params;

    THREAD_MEMORY_RELEASE();
    ring->state.write_idx++;

    return 0;
}

int sddf_snd_enqueue_pcm_data(sddf_snd_pcm_data_ring_t *ring, sddf_snd_pcm_data_t *pcm)
{
    if (sddf_snd_ring_full(&ring->state)) {
        return -1;
    }

    sddf_snd_pcm_data_t *data = &ring->buffers[ring->state.write_idx % ring->state.size];

    data->cookie = pcm->cookie;
    data->stream_id = pcm->stream_id;
    data->addr = pcm->addr;
    data->len = pcm->len;
    data->status = pcm->status;
    data->latency_bytes = pcm->latency_bytes;

    THREAD_MEMORY_RELEASE();
    ring->state.write_idx++;

    return 0;
}

int sddf_snd_dequeue_cmd(sddf_snd_cmd_ring_t *ring, sddf_snd_cmd_t *out)
{
    if (sddf_snd_ring_empty(&ring->state)) {
        return -1;
    }

    sddf_snd_cmd_t *src = &ring->buffers[ring->state.read_idx % ring->state.size];
    out->code = src->code;
    out->cookie = src->cookie;
    out->stream_id = src->stream_id;
    out->set_params = src->set_params;

    THREAD_MEMORY_RELEASE();
    ring->state.read_idx++;

    return 0;
}

int sddf_snd_dequeue_pcm_data(sddf_snd_pcm_data_ring_t *ring, sddf_snd_pcm_data_t *out)
{
    if (sddf_snd_ring_empty(&ring->state)) {
        return -1;
    }

    sddf_snd_pcm_data_t *pcm = &ring->buffers[ring->state.read_idx % ring->state.size];

    out->cookie = pcm->cookie;
    out->stream_id = pcm->stream_id;
    out->addr = pcm->addr;
    out->len = pcm->len;
    out->status = pcm->status;
    out->latency_bytes = pcm->latency_bytes;

    THREAD_MEMORY_RELEASE();
    ring->state.read_idx++;

    return 0;
}

int sddf_snd_ring_dequeue(sddf_snd_ring_state_t *ring)
{
    if (sddf_snd_ring_empty(ring)) {
        return -1;
    }

    THREAD_MEMORY_RELEASE();
    ring->read_idx++;

    return 0;
}

sddf_snd_cmd_t *sddf_snd_cmd_ring_front(sddf_snd_cmd_ring_t *ring)
{
    return &ring->buffers[ring->state.read_idx % ring->state.size];
}

sddf_snd_pcm_data_t *sddf_snd_pcm_data_front(sddf_snd_pcm_data_ring_t *ring)
{
    return &ring->buffers[ring->state.read_idx % ring->state.size];
}

const char *sddf_snd_command_code_str(sddf_snd_command_code_t code)
{
    switch (code) {
        case SDDF_SND_CMD_PCM_TAKE:
            return "PCM_SET_PARAMS";
        case SDDF_SND_CMD_PCM_PREPARE:
            return "PCM_PREPARE";
        case SDDF_SND_CMD_PCM_RELEASE:
            return "PCM_RELEASE";
        case SDDF_SND_CMD_PCM_START:
            return "PCM_START";
        case SDDF_SND_CMD_PCM_STOP:
            return "PCM_STOP";
        default:
            return "<unknown>";
    }
}

const char *sddf_snd_status_code_str(sddf_snd_status_code_t code)
{
    switch (code) {
        case SDDF_SND_S_OK:
            return "OK";
        case SDDF_SND_S_BAD_MSG:
            return "BAD_MSG";
        case SDDF_SND_S_NOT_SUPP:
            return "NOT_SUPP";
        case SDDF_SND_S_IO_ERR:
            return "IO_ERR";
        default:
            return "<unknown>";
    }
}

const char *sddf_snd_event_code_str(sddf_snd_event_code_t code)
{
    switch (code) {
        case SDDF_SND_EVT_PCM_PERIOD_ELAPSED:
            return "PCM_PERIOD_ELAPSED";
        case SDDF_SND_EVT_PCM_XRUN:
            return "PCM_XRUN";
        default:
            return "<unknown>";
    }
}

const char *sddf_snd_pcm_fmt_str(sddf_snd_pcm_fmt_t fmt)
{
    switch (fmt) {
        case SDDF_SND_PCM_FMT_IMA_ADPCM: return "IMA_ADPCM";
        case SDDF_SND_PCM_FMT_MU_LAW:    return "MU_LAW";
        case SDDF_SND_PCM_FMT_A_LAW:     return "A_LAW";
        case SDDF_SND_PCM_FMT_S8:        return "S8";
        case SDDF_SND_PCM_FMT_U8:        return "U8";
        case SDDF_SND_PCM_FMT_S16:       return "S16";
        case SDDF_SND_PCM_FMT_U16:       return "U16";
        case SDDF_SND_PCM_FMT_S18_3:     return "S18_3";
        case SDDF_SND_PCM_FMT_U18_3:     return "U18_3";
        case SDDF_SND_PCM_FMT_S20_3:     return "S20_3";
        case SDDF_SND_PCM_FMT_U20_3:     return "U20_3";
        case SDDF_SND_PCM_FMT_S24_3:     return "S24_3";
        case SDDF_SND_PCM_FMT_U24_3:     return "U24_3";
        case SDDF_SND_PCM_FMT_S20:       return "S20";
        case SDDF_SND_PCM_FMT_U20:       return "U20";
        case SDDF_SND_PCM_FMT_S24:       return "S24";
        case SDDF_SND_PCM_FMT_U24:       return "U24";
        case SDDF_SND_PCM_FMT_S32:       return "S32";
        case SDDF_SND_PCM_FMT_U32:       return "U32";
        case SDDF_SND_PCM_FMT_FLOAT:     return "FLOAT";
        case SDDF_SND_PCM_FMT_FLOAT64:   return "FLOAT64";
        case SDDF_SND_PCM_FMT_DSD_U8:    return "DSD_U8";
        case SDDF_SND_PCM_FMT_DSD_U16:   return "DSD_U16";
        case SDDF_SND_PCM_FMT_DSD_U32:   return "DSD_U32";
        case SDDF_SND_PCM_FMT_IEC958_SUBFRAME: return "IEC958_SUBFRAME";
        default:
            return "<unknown>";
    }
}
