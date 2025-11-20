/** @file picow_freertos_ping.c
 *  @brief TCP server example adapted for Pico W with FreeRTOS.
 *
 *  NOTE: Kept minimal; large commented original loops condensed.
 */

#include "picow_freertos_ping.h"
#include <stdio.h>
#include <string.h>
#include "hardware/adc.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

/* ==============================
 * Globals
 * ============================== */
MessageBufferHandle_t xControlMessageBuffer = NULL;
struct tcp_pcb *tcp_server_pcb = NULL;
static uint8_t receive_buffer[BUFFER_SIZE];

/* ==============================
 * Temperature Read
 * ============================== */
static float read_onboard_temperature_(void)
{
    const float conv = 3.3f / (1 << 12);
    float adc_v = (float)adc_read() * conv;
    return 27.0f - (adc_v - 0.706f) / 0.001721f;
}

/* ==============================
 * TCP Callbacks
 * ============================== */
static void tcp_server_error_(void *arg, err_t err)
{
    (void)arg;
    printf("[TCP] error %d\n", err);
}

static err_t tcp_server_recv_(void *arg, struct tcp_pcb *tpcb,
                              struct pbuf *p, err_t err)
{
    (void)arg;
    if (p == NULL)
    {
        printf("[TCP] closed\n");
        tcp_close(tpcb);
        return ERR_OK;
    }
    if (err != ERR_OK)
    {
        pbuf_free(p);
        return err;
    }

    tcp_recved(tpcb, p->tot_len);
    if (p->tot_len > 0)
    {
        uint16_t cpy = pbuf_copy_partial(p, receive_buffer,
                                         p->tot_len, 0);
        receive_buffer[cpy] = '\0';
        printf("[TCP] rx %u bytes: %s\n", cpy, receive_buffer);

        char ack[64];
        snprintf(ack, sizeof(ack), "ACK:%s", receive_buffer);
        if (tcp_write(tpcb, ack, strlen(ack), TCP_WRITE_FLAG_COPY) == ERR_OK)
        {
            tcp_output(tpcb);
        }

        float temp = read_onboard_temperature_();
        char tmsg[48];
        snprintf(tmsg, sizeof(tmsg), "Temp:%.2fC", temp);
        if (tcp_write(tpcb, tmsg, strlen(tmsg), TCP_WRITE_FLAG_COPY) == ERR_OK)
        {
            tcp_output(tpcb);
        }
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_accept_(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;
    if ((err != ERR_OK) || (newpcb == NULL))
    {
        return ERR_VAL;
    }
    printf("[TCP] conn from %s:%d\n",
           ip4addr_ntoa(&newpcb->remote_ip),
           newpcb->remote_port);
    tcp_arg(newpcb, NULL);
    tcp_err(newpcb, tcp_server_error_);
    tcp_recv(newpcb, tcp_server_recv_);
    return ERR_OK;
}

/* ==============================
 * TCP Server Init
 * ============================== */
static bool tcp_server_init_(void)
{
    tcp_server_pcb = tcp_new();
    if (!tcp_server_pcb)
    {
        return false;
    }
    err_t err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, TCP_PORT);
    if (err != ERR_OK)
    {
        tcp_close(tcp_server_pcb);
        return false;
    }
    tcp_server_pcb = tcp_listen(tcp_server_pcb);
    if (!tcp_server_pcb)
    {
        return false;
    }
    tcp_accept(tcp_server_pcb, tcp_server_accept_);
    printf("[TCP] server on port %d\n", TCP_PORT);
    printf("[TCP] IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    return true;
}

/*** end of file ***/
