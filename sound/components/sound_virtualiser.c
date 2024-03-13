#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/sound/sound_queue.h>

#define DRIVER_CH 3
#define CLIENT_CH_BEGIN 1
#define CLIENT_COUNT 2

#define NO_OWNER -1
#define MAX_STREAMS 16

uintptr_t drv_cmd_req;
uintptr_t drv_cmd_res;
uintptr_t drv_pcm_req;
uintptr_t drv_pcm_res;

uintptr_t c0_cmd_req;
uintptr_t c0_cmd_res;
uintptr_t c0_pcm_req;
uintptr_t c0_pcm_res;

uintptr_t c1_cmd_req;
uintptr_t c1_cmd_res;
uintptr_t c1_pcm_req;
uintptr_t c1_pcm_res;

uintptr_t sound_shared_state;

static sddf_snd_rings_t clients[CLIENT_COUNT];
static sddf_snd_rings_t driver_rings;

static int owners[MAX_STREAMS];
static bool started;

int notified_by_client(int client) {

    if (client < 0 || client > CLIENT_COUNT) {
        microkit_dbg_puts("SND VIRT|ERR: invalid client id\n");
        return -1;
    }

    sddf_snd_rings_t *client_rings = &clients[client];

    sddf_snd_cmd_t cmd;
    while (sddf_snd_dequeue_cmd(client_rings->cmd_req, &cmd) == 0) {

        if (cmd.stream_id > MAX_STREAMS) {
            microkit_dbg_puts("SND VIRT|ERR: stream id too large\n");
            continue;
        }

        if (owners[cmd.stream_id] == -1) {
            if (cmd.code == SDDF_SND_CMD_PCM_TAKE) {
                owners[cmd.stream_id] = client;
            } else {
                // TODO: send error response: must take first
                microkit_dbg_puts("SND VIRT|ERR: client must take first\n");
                continue;
            }
        }

        int owner = owners[cmd.stream_id];
        if (owner != client) {
            // TODO: send error response: not owned by client
            microkit_dbg_puts("SND VIRT|ERR: stream busy\n");
            continue;
        }

        if (sddf_snd_enqueue_cmd(driver_rings.cmd_req, &cmd) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue command\n");
            // TODO: send error response
            return -1;
        }
    }

    sddf_snd_pcm_data_t pcm;
    while (sddf_snd_dequeue_pcm_data(client_rings->pcm_req, &pcm) == 0) {

        if (pcm.stream_id > MAX_STREAMS) {
            microkit_dbg_puts("SND VIRT|ERR: stream id too large\n");
            continue;
        }

        int owner = owners[pcm.stream_id];
        if (owner != client) {
            // TODO: send error response: not owned by client
            microkit_dbg_puts("SND VIRT|ERR: driver replied to busy stream\n");
            continue;
        }

        if (sddf_snd_enqueue_pcm_data(driver_rings.pcm_req, &pcm) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
    }

    // TODO: only notify if we have enqueued to driver
    microkit_notify(DRIVER_CH);

    return 0;
}

int notified_by_driver(void) {

    bool notify[CLIENT_COUNT] = {0};

    sddf_snd_cmd_t cmd;
    while (sddf_snd_dequeue_cmd(driver_rings.cmd_res, &cmd) == 0) {

        if (cmd.stream_id > MAX_STREAMS) {
            microkit_dbg_puts("SND VIRT|ERR: stream id too large\n");
            continue;
        }

        int owner = owners[cmd.stream_id];
        if (owner < 0 || owner > CLIENT_COUNT) {
            microkit_dbg_puts("SND VIRT|ERR: invalid owner id\n");
            continue;
        }

        if (cmd.code == SDDF_SND_CMD_PCM_RELEASE ||
            (cmd.code == SDDF_SND_CMD_PCM_TAKE && cmd.status != SDDF_SND_S_OK))
        {
            owners[cmd.stream_id] = -1;
        }

        if (sddf_snd_enqueue_cmd(clients[owner].cmd_res, &cmd) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue command response\n");
            return -1;
        }
        notify[owner] = true;
    }

    sddf_snd_pcm_data_t pcm;
    while (sddf_snd_dequeue_pcm_data(driver_rings.pcm_res, &pcm) == 0) {

        if (pcm.stream_id > MAX_STREAMS) {
            microkit_dbg_puts("SND VIRT|ERR: stream id too large\n");
            continue;
        }

        int owner = owners[pcm.stream_id];
        if (owner < 0 || owner > CLIENT_COUNT) {
            microkit_dbg_puts("SND VIRT|ERR: invalid owner id\n");
            continue;
        }
        
        if (sddf_snd_enqueue_pcm_data(clients[owner].pcm_res, &pcm) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
        notify[owner] = true;
    }

    for (int client = 0; client < CLIENT_COUNT; client++) {
        if (notify[client]) {
            microkit_notify(CLIENT_CH_BEGIN + client);
        }
    }

    if (!started) {
        for (int client = 0; client < CLIENT_COUNT; client++) {
            microkit_notify(CLIENT_CH_BEGIN + client);
        }
        started = true;
    }

    return 0;
}

void init(void) {
    clients[0].cmd_req = (void *)c0_cmd_req;
    clients[0].cmd_res = (void *)c0_cmd_res;
    clients[0].pcm_req = (void *)c0_pcm_req;
    clients[0].pcm_res = (void *)c0_pcm_res;

    clients[1].cmd_req = (void *)c1_cmd_req;
    clients[1].cmd_res = (void *)c1_cmd_res;
    clients[1].pcm_req = (void *)c1_pcm_req;
    clients[1].pcm_res = (void *)c1_pcm_res;

    driver_rings.cmd_req = (void *)drv_cmd_req;
    driver_rings.cmd_res = (void *)drv_cmd_res;
    driver_rings.pcm_req = (void *)drv_pcm_req;
    driver_rings.pcm_res = (void *)drv_pcm_res;
    sddf_snd_rings_init_default(&driver_rings);

    for (int i = 0; i < MAX_STREAMS; i++) {
        owners[i] = NO_OWNER;
    }
    started = false;
}

void notified(microkit_channel ch) {

    if (ch == DRIVER_CH) {
        if (notified_by_driver() != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to handle driver notification\n");
        }
    } else if (ch >= CLIENT_CH_BEGIN && ch < CLIENT_CH_BEGIN + CLIENT_COUNT) {
        if (notified_by_client(ch - CLIENT_CH_BEGIN) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to handle client notification\n");
        }
    } else {
        microkit_dbg_puts("SND VIRT|ERR: Received a bad client channel\n");
    }
}
