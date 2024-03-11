/*
 * Copyright 2020, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#include <string.h>
#include <microkit.h>

#include "lwip/ip.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <sddf/network/socket.h>
#include "util.h"
#include "client.h"
/* This file implements a TCP based profiler measurment process that starts
 * and stops profiler measurements based on a client's requests.
 * The protocol used to communicate is as follows:
 * - Client connects
 * - Server sends: 100 SEL4 PROFILING CLIENT\n
 * - Client sends: HELLO\n
 * - Server sends: 200 OK (Ready to go)\n
 * - Client sends: START\n
 * - Client sends: STOP\n
 * - Client sends: START\n
 * - Client sends: EXIT\n
 * - Server closes socket.
 *
 *
 * The server starts recording profiler stats when it receives START and
 * finishes recording when it receives STOP or EXIT.
 *
 * Only one client can be connected.
 */

static struct tcp_pcb *utiliz_socket;
uintptr_t data_packet;

#define WHOAMI "100 SEL4 PROFILING CLIENT\n"
#define HELLO "HELLO\n"
#define OK_READY "200 OK (Ready to go)\n"
#define OK "200 OK\n"
#define START "START\n"
#define STOP "STOP\n"
#define QUIT "QUIT\n"
#define ERROR "400 ERROR\n"
#define MAPPINGS "MAPPINGS\n"
#define REFRESH "REFRESH\n"

// TODO: NEED TO HAVE A BETTER WAY OF INJECTING THE MAPPINGS FROM THE SYSTEM DESCRIPTION
#define MAPPINGS_STR "dummy_prog: 0\ndummy_prog2: 14"

#define msg_match(msg, match) (strncmp(msg, match, strlen(match))==0)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define RES(x, y, z) "220 VALID DATA (Data to follow)\n"    \
    "Content-length: "STR(x)"\n"\
    ","STR(y)","STR(z)


struct bench *bench = (void *)(uintptr_t)0x5010000;

uint64_t start;
uint64_t idle_ccount_start;
uint64_t idle_overflow_start;


static err_t profiler_sent_callback(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    return ERR_OK;
}

static err_t profiler_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    if (p == NULL) {
        tcp_close(pcb);
        return ERR_OK;
    }

    char *data_packet_str = (char *)data_packet;

    pbuf_copy_partial(p, (void *)data_packet, p->tot_len, 0);
    err_t error;

    if (msg_match(data_packet_str, HELLO)) {
        error = tcp_write(pcb, OK_READY, strlen(OK_READY), TCP_WRITE_FLAG_COPY);
        if (error) {
            microkit_dbg_puts("Failed to send OK_READY message through profiler peer");
        }
    } else if (msg_match(data_packet_str, START)) {
        print(microkit_name);
        print(" measurement starting... \n");
        microkit_notify(START_PMU);
    } else if (msg_match(data_packet_str, STOP)) {
        print(microkit_name);
        print(" measurement finished \n");;
        microkit_notify(STOP_PMU);
    } else if (msg_match(data_packet_str, MAPPINGS)) {
        error = tcp_write(pcb, MAPPINGS_STR, strlen(MAPPINGS_STR), TCP_WRITE_FLAG_COPY);
        if (error) {
            microkit_dbg_puts("Failed to send OK message through profiler peer");
        }
    } else if (msg_match(data_packet_str, REFRESH)) {
        // This is just to refresh the socket from the linux client side
        return ERR_OK;
    } else {
        microkit_dbg_puts("Received a message that we can't handle ");
        microkit_dbg_puts(data_packet_str);
        microkit_dbg_puts("\n");
        error = tcp_write(pcb, ERROR, strlen(ERROR), TCP_WRITE_FLAG_COPY);
        if (error) {
            microkit_dbg_puts("Failed to send OK message through profiler peer");
        }
    }

    return ERR_OK;
}

static err_t profiler_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    print("profiler connection established!\n");
    err_t error = tcp_write(newpcb, WHOAMI, strlen(WHOAMI), TCP_WRITE_FLAG_COPY);
    if (error) {
        print("Failed to send WHOAMI message through profiler peer\n");
    }
    tcp_sent(newpcb, profiler_sent_callback);
    tcp_recv(newpcb, profiler_recv_callback);
    utiliz_socket = newpcb;
    return ERR_OK;
}

int send_tcp(void *buff) {

    err_t error = tcp_write(utiliz_socket, buff, strlen(buff), TCP_WRITE_FLAG_COPY);
    if (error) {
        print("Failed to send message through profiler peer: ");
        put8(error);
        print("\n");
        if (error == -1) {
            print("MEM ERROR\n");
        }
        return 1;
    }

    error = tcp_output(utiliz_socket);
    if (error) {
        print("Failed to output via tcp through profiler peer: ");
        put8(error);
        print("\n");
    }

    return 0;
}

void tcp_sent_callback(tcp_sent_fn callback) {
    tcp_sent(utiliz_socket, callback);
}  

void tcp_reset_callback() {
    tcp_sent(utiliz_socket, profiler_sent_callback);
}  

int setup_utilization_socket(void)
{
    utiliz_socket = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (utiliz_socket == NULL) {
        print("Failed to open a socket for listening!");
        return -1;
    }

    err_t error = tcp_bind(utiliz_socket, IP_ANY_TYPE, UTILIZATION_PORT);
    if (error) {
        print("Failed to bind the TCP socket");
        return -1;
    } else {
        // print("Utilisation port bound to port 1236\n");
    }

    utiliz_socket = tcp_listen_with_backlog_and_err(utiliz_socket, 1, &error);
    if (error != ERR_OK) {
        print("Failed to listen on the profiler socket");
        return -1;
    }
    tcp_accept(utiliz_socket, profiler_accept_callback);

    return 0;
}