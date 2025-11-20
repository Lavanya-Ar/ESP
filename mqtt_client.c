/** @file mqtt_client.c
 *  @brief Pico W Wi-Fi + MQTT helper (threadsafe lwIP usage).
 *
 *  NOTE: Legacy commented block removed; condensed publish debug.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "mqtt_client.h"
#include "FreeRTOS.h"
#include "task.h"

/* ==============================
 * User/Build Configuration
 * ============================== */
#ifndef WIFI_SSID
#define WIFI_SSID        "YOUR_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD    "YOUR_PASSWORD"
#endif
#ifndef BROKER_IP
#define BROKER_IP        "192.168.1.10"
#endif
#ifndef BASE_TOPIC
#define BASE_TOPIC       "robot/alpha"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT        (1883)
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID   "pico-car"
#endif
#ifndef MQTT_KEEPALIVE_S
#define MQTT_KEEPALIVE_S (60)
#endif

/* ==============================
 * Static State
 * ============================== */
static mqtt_client_t    *g_client          = NULL;
static volatile bool     g_mqtt_connected  = false;

/* ==============================
 * Private Prototypes
 * ============================== */
static void mqtt_conn_cb_(mqtt_client_t *client,
                          void *arg,
                          mqtt_connection_status_t status);
static void mqtt_incoming_publish_cb_(void *arg,
                                      const char *topic,
                                      u32_t tot_len);
static void mqtt_incoming_data_cb_(void *arg,
                                   const u8_t *data,
                                   u16_t len,
                                   u8_t flags);
static void log_ip_(const char *label, const ip_addr_t *ip);

/* ==============================
 * Helpers
 * ============================== */
static void log_ip_(const char *label, const ip_addr_t *ip)
{
    printf("%s %s\n", label, ipaddr_ntoa(ip));
}

/* ==============================
 * Incoming Handlers
 * ============================== */
static void mqtt_incoming_publish_cb_(void *arg,
                                      const char *topic,
                                      u32_t tot_len)
{
    (void)arg;
    printf("[MQTT<-] topic=%s len=%" PRIu32 "\n",
           topic ? topic : "(null)", tot_len);
}

static void mqtt_incoming_data_cb_(void *arg,
                                   const u8_t *data,
                                   u16_t len,
                                   u8_t flags)
{
    (void)arg;
    static char buf[384];
    size_t n = (len < (sizeof buf - 1U)) ? len : (sizeof buf - 1U);
    memcpy(buf, data, n);
    buf[n] = '\0';
    printf("[MQTT<-] data:%s%s\n",
           buf,
           (flags & MQTT_DATA_FLAG_LAST) ? " [LAST]" : "");
}

/* ==============================
 * Connection Callback
 * ============================== */
static void mqtt_conn_cb_(mqtt_client_t *client,
                          void *arg,
                          mqtt_connection_status_t status)
{
    (void)arg;
    printf("[MQTT] conn status=%d\n", status);
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        g_mqtt_connected = true;
        printf("[MQTT] connected keepalive=%d id=%s\n",
               MQTT_KEEPALIVE_S, MQTT_CLIENT_ID);

        char topic[128];
        snprintf(topic, sizeof(topic), "%s/cmd", BASE_TOPIC);
        err_t e = mqtt_sub_unsub(client, topic, 1, NULL, NULL, 1);
        printf("[MQTT] subscribe %s => %d\n", topic, (int)e);

        const char *diag = "{\"hello\":\"pico-online\"}";
        char dtopic[128];
        snprintf(dtopic, sizeof(dtopic), "%s/diag", BASE_TOPIC);
        cyw43_arch_lwip_begin();
        err_t de = mqtt_publish(client, dtopic, diag,
                                (u16_t)strlen(diag),
                                0, 1, NULL, NULL);
        cyw43_arch_lwip_end();
        printf("[MQTT] diag retained err=%d\n", (int)de);

        mqtt_set_inpub_callback(client,
                                mqtt_incoming_publish_cb_,
                                mqtt_incoming_data_cb_,
                                NULL);
    }
    else
    {
        g_mqtt_connected = false;
    }
}

/* ==============================
 * Public Status
 * ============================== */
bool mqtt_is_connected(void)
{
    return g_mqtt_connected;
}

/* ==============================
 * Connect Wi-Fi + MQTT
 * ============================== */
bool wifi_and_mqtt_start(void)
{
    if (cyw43_arch_init())
    {
        printf("[NET] cyw43 init fail\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    printf("[NET] WiFi SSID=%s\n", WIFI_SSID);
    int r = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID,
                                               WIFI_PASSWORD,
                                               CYW43_AUTH_WPA2_AES_PSK,
                                               30000);
    if (r)
    {
        printf("[NET] WiFi connect failed err=%d\n", r);
        return false;
    }
    printf("[NET] WiFi connected\n");

    ip_addr_t broker_ip;
    if (!ipaddr_aton(BROKER_IP, &broker_ip))
    {
        printf("[MQTT] invalid BROKER_IP %s\n", BROKER_IP);
        return false;
    }
    log_ip_("[MQTT] broker:", &broker_ip);

    cyw43_arch_lwip_begin();
    g_client = mqtt_client_new();
    cyw43_arch_lwip_end();
    if (!g_client)
    {
        printf("[MQTT] client_new failed\n");
        return false;
    }

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id  = MQTT_CLIENT_ID;
    ci.keep_alive = MQTT_KEEPALIVE_S;

    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(g_client, &broker_ip,
                                    MQTT_PORT,
                                    mqtt_conn_cb_,
                                    NULL,
                                    &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK)
    {
        printf("[MQTT] connect err=%d\n", err);
        return false;
    }
    return true;
}

/* ==============================
 * Publish Telemetry
 * ============================== */
void mqtt_publish_telemetry(float speed,
                            float distance_cm,
                            float yaw_deg,
                            float ultra_cm,
                            const char *state)
{
    if ((!g_client) || (!g_mqtt_connected))
    {
        return;
    }

    char topic[128];
    char payload[256];

    snprintf(payload, sizeof(payload),
             "{\"speed\":%.2f,\"distance\":%.2f,\"imu\":{\"yaw\":%.2f},"
             "\"ultra_cm\":%.2f,\"state\":\"%s\"}",
             speed, distance_cm, yaw_deg, ultra_cm,
             state ? state : "idle");

    snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);

    cyw43_arch_lwip_begin();
    err_t e = mqtt_publish(g_client,
                           topic,
                           payload,
                           (u16_t)strlen(payload),
                           1, 0,
                           NULL, NULL);
    cyw43_arch_lwip_end();

    if (e != ERR_OK)
    {
        printf("[MQTT] publish err=%d\n", (int)e);
    }
}

/* ==============================
 * Optional Poll (no-op under sys_freertos)
 * ============================== */
void mqtt_loop_poll(void)
{
    sleep_ms(1);
}

/*** end of file ***/
