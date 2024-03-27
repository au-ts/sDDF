#include <sddf/sound/queue.h>
#include <sddf/util/fence.h>

void sound_queue_init(sound_queue_state_t *queue_state, uint32_t buffer_count)
{
    queue_state->read_idx = 0;
    queue_state->write_idx = 0;
    queue_state->size = buffer_count;
}

void sound_queues_init_default(sound_queues_t *queues)
{
    sound_queue_init(&queues->cmd_req->state,  SOUND_NUM_BUFFERS);
    sound_queue_init(&queues->cmd_res->state, SOUND_NUM_BUFFERS);
    sound_queue_init(&queues->pcm_req->state, SOUND_NUM_BUFFERS);
    sound_queue_init(&queues->pcm_res->state, SOUND_NUM_BUFFERS);
}

bool sound_queue_empty(sound_queue_state_t *queue_state)
{
    return queue_state->write_idx == queue_state->read_idx;
    // return !((queue_state->write_idx - queue_state->read_idx) % queue_state->size);
}

bool sound_queue_full(sound_queue_state_t *queue_state)
{
    return (queue_state->write_idx - queue_state->read_idx + 1) == queue_state->size;
    // return !((queue_state->write_idx - queue_state->read_idx + 1) % queue_state->size);
}

int sound_queue_size(sound_queue_state_t *queue_state)
{
    return queue_state->write_idx - queue_state->read_idx;
}

int sound_enqueue_cmd(sound_cmd_queue_t *queue,
                         const sound_cmd_t *command)
{
    if (sound_queue_full(&queue->state)) {
        return -1;
    }

    sound_cmd_t *dest = &queue->buffers[queue->state.write_idx % queue->state.size];

    dest->code = command->code;
    dest->cookie = command->cookie;
    dest->stream_id = command->stream_id;
    dest->set_params = command->set_params;

    THREAD_MEMORY_RELEASE();
    queue->state.write_idx++;

    return 0;
}

int sound_enqueue_pcm(sound_pcm_queue_t *queue, sound_pcm_t *pcm)
{
    if (sound_queue_full(&queue->state)) {
        return -1;
    }

    sound_pcm_t *data = &queue->buffers[queue->state.write_idx % queue->state.size];

    data->cookie = pcm->cookie;
    data->stream_id = pcm->stream_id;
    data->addr = pcm->addr;
    data->len = pcm->len;
    data->status = pcm->status;
    data->latency_bytes = pcm->latency_bytes;

    THREAD_MEMORY_RELEASE();
    queue->state.write_idx++;

    return 0;
}

int sound_dequeue_cmd(sound_cmd_queue_t *queue, sound_cmd_t *out)
{
    if (sound_queue_empty(&queue->state)) {
        return -1;
    }

    sound_cmd_t *src = &queue->buffers[queue->state.read_idx % queue->state.size];
    out->code = src->code;
    out->cookie = src->cookie;
    out->stream_id = src->stream_id;
    out->set_params = src->set_params;

    THREAD_MEMORY_RELEASE();
    queue->state.read_idx++;

    return 0;
}

int sound_dequeue_pcm(sound_pcm_queue_t *queue, sound_pcm_t *out)
{
    if (sound_queue_empty(&queue->state)) {
        return -1;
    }

    sound_pcm_t *pcm = &queue->buffers[queue->state.read_idx % queue->state.size];

    out->cookie = pcm->cookie;
    out->stream_id = pcm->stream_id;
    out->addr = pcm->addr;
    out->len = pcm->len;
    out->status = pcm->status;
    out->latency_bytes = pcm->latency_bytes;

    THREAD_MEMORY_RELEASE();
    queue->state.read_idx++;

    return 0;
}

int sound_queue_dequeue(sound_queue_state_t *queue)
{
    if (sound_queue_empty(queue)) {
        return -1;
    }

    THREAD_MEMORY_RELEASE();
    queue->read_idx++;

    return 0;
}

sound_cmd_t *sound_cmd_queue_front(sound_cmd_queue_t *queue)
{
    return &queue->buffers[queue->state.read_idx % queue->state.size];
}

sound_pcm_t *sound_pcm_front(sound_pcm_queue_t *queue)
{
    return &queue->buffers[queue->state.read_idx % queue->state.size];
}

const char *sound_command_code_str(sound_cmd_code_t code)
{
    switch (code) {
        case SOUND_CMD_TAKE:
            return "PCM_SET_PARAMS";
        case SOUND_CMD_PREPARE:
            return "PCM_PREPARE";
        case SOUND_CMD_RELEASE:
            return "PCM_RELEASE";
        case SOUND_CMD_START:
            return "PCM_START";
        case SOUND_CMD_STOP:
            return "PCM_STOP";
        default:
            return "<unknown>";
    }
}

const char *sound_status_code_str(sound_status_t code)
{
    switch (code) {
        case SOUND_S_OK:
            return "OK";
        case SOUND_S_BAD_MSG:
            return "BAD_MSG";
        case SOUND_S_NOT_SUPP:
            return "NOT_SUPP";
        case SOUND_S_IO_ERR:
            return "IO_ERR";
        default:
            return "<unknown>";
    }
}

const char *sound_pcm_fmt_str(sound_pcm_fmt_t fmt)
{
    switch (fmt) {
        case SOUND_PCM_FMT_IMA_ADPCM: return "IMA_ADPCM";
        case SOUND_PCM_FMT_MU_LAW:    return "MU_LAW";
        case SOUND_PCM_FMT_A_LAW:     return "A_LAW";
        case SOUND_PCM_FMT_S8:        return "S8";
        case SOUND_PCM_FMT_U8:        return "U8";
        case SOUND_PCM_FMT_S16:       return "S16";
        case SOUND_PCM_FMT_U16:       return "U16";
        case SOUND_PCM_FMT_S18_3:     return "S18_3";
        case SOUND_PCM_FMT_U18_3:     return "U18_3";
        case SOUND_PCM_FMT_S20_3:     return "S20_3";
        case SOUND_PCM_FMT_U20_3:     return "U20_3";
        case SOUND_PCM_FMT_S24_3:     return "S24_3";
        case SOUND_PCM_FMT_U24_3:     return "U24_3";
        case SOUND_PCM_FMT_S20:       return "S20";
        case SOUND_PCM_FMT_U20:       return "U20";
        case SOUND_PCM_FMT_S24:       return "S24";
        case SOUND_PCM_FMT_U24:       return "U24";
        case SOUND_PCM_FMT_S32:       return "S32";
        case SOUND_PCM_FMT_U32:       return "U32";
        case SOUND_PCM_FMT_FLOAT:     return "FLOAT";
        case SOUND_PCM_FMT_FLOAT64:   return "FLOAT64";
        case SOUND_PCM_FMT_DSD_U8:    return "DSD_U8";
        case SOUND_PCM_FMT_DSD_U16:   return "DSD_U16";
        case SOUND_PCM_FMT_DSD_U32:   return "DSD_U32";
        case SOUND_PCM_FMT_IEC958_SUBFRAME: return "IEC958_SUBFRAME";
        default:
            return "<unknown>";
    }
}
