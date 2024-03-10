#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/sound/sound_queue.h>

#define DRIVER_CH 3
#define CLIENT_CH_BEGIN 1
#define CLIENT_COUNT 2

uintptr_t drv_commands;
uintptr_t drv_responses;
uintptr_t drv_tx_req;
uintptr_t drv_tx_res;
uintptr_t drv_rx_res;
uintptr_t drv_rx_req;

uintptr_t c0_commands;
uintptr_t c0_responses;
uintptr_t c0_tx_req;
uintptr_t c0_tx_res;
uintptr_t c0_rx_res;
uintptr_t c0_rx_req;

uintptr_t c1_commands;
uintptr_t c1_responses;
uintptr_t c1_tx_req;
uintptr_t c1_tx_res;
uintptr_t c1_rx_res;
uintptr_t c1_rx_req;

uintptr_t sound_shared_state;

static sddf_snd_rings_t clients[CLIENT_COUNT];
static sddf_snd_rings_t driver_rings;

int notified_by_client(int client) {

    if (client < 0 || client > CLIENT_COUNT) {
        return -1;
    }

    sddf_snd_rings_t *client_rings = &clients[client];

    sddf_snd_command_t cmd;
    while (sddf_snd_dequeue_cmd(client_rings->commands, &cmd) == 0) {
        if (sddf_snd_enqueue_cmd(driver_rings.commands, &cmd) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue command\n");
            return -1;
        }
    }

    sddf_snd_pcm_data_t pcm;
    while (sddf_snd_dequeue_pcm_data(client_rings->tx_req, &pcm) == 0) {
        if (sddf_snd_enqueue_pcm_data(driver_rings.tx_req, &pcm) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
    }

    while (sddf_snd_dequeue_pcm_data(client_rings->rx_req, &pcm) == 0) {
        if (sddf_snd_enqueue_pcm_data(driver_rings.rx_req, &pcm) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
    }

    // TODO: only notify if we have enqueued to driver
    microkit_notify(DRIVER_CH);

    return 0;
}

int notified_by_driver(void) {

    // TODO: use correct client
    int client = 0;
    sddf_snd_rings_t *client_rings = &clients[client];

    sddf_snd_response_t response;
    while (sddf_snd_dequeue_response(driver_rings.responses, &response) == 0) {
        if (sddf_snd_enqueue_response(client_rings->responses, &response) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue response\n");
            return -1;
        }
    }

    sddf_snd_pcm_data_t pcm;
    while (sddf_snd_dequeue_pcm_data(driver_rings.tx_res, &pcm) == 0) {
        if (sddf_snd_enqueue_pcm_data(client_rings->tx_res, &pcm) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
    }

    while (sddf_snd_dequeue_pcm_data(driver_rings.rx_res, &pcm) == 0) {
        if (sddf_snd_enqueue_pcm_data(client_rings->rx_res, &pcm) != 0) {
            microkit_dbg_puts("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
    }

    microkit_notify(CLIENT_CH_BEGIN + client);

    return 0;
}

void init(void) {
    clients[0].commands = (void *)c0_commands;
    clients[0].responses = (void *)c0_responses;
    clients[0].tx_req = (void *)c0_tx_req;
    clients[0].tx_res = (void *)c0_tx_res;
    clients[0].rx_res = (void *)c0_rx_res;
    clients[0].rx_req = (void *)c0_rx_req;

    clients[1].commands = (void *)c1_commands;
    clients[1].responses = (void *)c1_responses;
    clients[1].tx_req = (void *)c1_tx_req;
    clients[1].tx_res = (void *)c1_tx_res;
    clients[1].rx_res = (void *)c1_rx_res;
    clients[1].rx_req = (void *)c1_rx_req;

    driver_rings.commands = (void *)drv_commands;
    driver_rings.responses = (void *)drv_responses;
    driver_rings.tx_req = (void *)drv_tx_req;
    driver_rings.tx_res = (void *)drv_tx_res;
    driver_rings.rx_res = (void *)drv_rx_res;
    driver_rings.rx_req = (void *)drv_rx_req;
    sddf_snd_rings_init_default(&driver_rings);

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
