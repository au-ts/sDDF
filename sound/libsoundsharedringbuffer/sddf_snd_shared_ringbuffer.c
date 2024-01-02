#include "sddf_snd_shared_ringbuffer.h"


void sddf_snd_ring_init(sddf_snd_ring_state_t *ring_state, uint32_t buffer_count)
{
    ring_state->read_idx = 0;
    ring_state->write_idx = 0;
    ring_state->size = buffer_count;
}

void sddf_snd_rings_init_default(sddf_snd_rings_t *rings)
{
    sddf_snd_ring_init(&rings->commands->state, SDDF_SND_NUM_CMD_BUFFERS);
    sddf_snd_ring_init(&rings->tx_free->state, SDDF_SND_NUM_PCM_DATA_BUFFERS);
    sddf_snd_ring_init(&rings->tx_used->state, SDDF_SND_NUM_PCM_DATA_BUFFERS);
    sddf_snd_ring_init(&rings->rx_free->state, SDDF_SND_NUM_PCM_DATA_BUFFERS);
    sddf_snd_ring_init(&rings->rx_used->state, SDDF_SND_NUM_PCM_DATA_BUFFERS);
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

int sddf_snd_enqueue_cmd(sddf_snd_cmd_ring_t *cmd_ring,
                         const sddf_snd_command_t *command)
{
    if (sddf_snd_ring_full(&cmd_ring->state)) {
        return -1;
    }

    sddf_snd_command_t *dest = &cmd_ring->buffers[cmd_ring->state.write_idx % cmd_ring->state.size];

    dest->code = command->code;
    dest->stream_id = command->stream_id;
    dest->set_params = command->set_params;

    THREAD_MEMORY_RELEASE();
    cmd_ring->state.write_idx++;

    return 0;
}

int sddf_snd_enqueue_pcm_data(sddf_snd_pcm_data_ring_t *pcm_ring,
                              uint32_t stream_id, intptr_t addr,
                              unsigned int len)
{
    if (sddf_snd_ring_full(&pcm_ring->state)) {
        return -1;
    }

    sddf_snd_pcm_data_t *data =
        &pcm_ring->buffers[pcm_ring->state.write_idx % pcm_ring->state.size];

    data->stream_id = stream_id;
    data->addr = addr;
    data->len = len;

    THREAD_MEMORY_RELEASE();
    pcm_ring->state.write_idx++;

    return 0;
}

int sddf_snd_dequeue_cmd(sddf_snd_cmd_ring_t *cmd_ring, sddf_snd_command_t *out)
{
    if (sddf_snd_ring_empty(&cmd_ring->state)) {
        return -1;
    }

    sddf_snd_command_t *command = &cmd_ring->buffers[cmd_ring->state.read_idx % cmd_ring->state.size];

    out->code = command->code;
    out->stream_id = command->stream_id;
    out->set_params = command->set_params;

    THREAD_MEMORY_RELEASE();
    cmd_ring->state.read_idx++;

    return 0;
}

int sddf_snd_dequeue_pcm_data(sddf_snd_pcm_data_ring_t *pcm_ring, sddf_snd_pcm_data_t *out)
{
    if (sddf_snd_ring_empty(&pcm_ring->state)) {
        return -1;
    }

    *out = pcm_ring->buffers[pcm_ring->state.read_idx % pcm_ring->state.size];

    THREAD_MEMORY_RELEASE();
    pcm_ring->state.read_idx++;

    return 0;
}
