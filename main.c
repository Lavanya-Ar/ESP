/** @file main.c
 *  @brief Integrated FreeRTOS task orchestration (line following, obstacle avoidance, barcode).
 *
 *  NOTE: Barr-C style, condensed telemetry, preserved original state machine logic.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motor_encoder_demo.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "barcode.h"
#include "Obstacle_Avoidance.h"
#include "PID_Line_Follow.h"
#include "mqtt_client.h"
#include "encoder.h"
#include "imu_raw_demo.h"

/* ==============================
 * Robot States
 * ============================== */
typedef enum
{
    STATE_LINE_FOLLOWING,
    STATE_OBSTACLE_AVOIDANCE,
    STATE_BARCODE_SCANNING,
    STATE_WAITING_FOR_JUNCTION,
    STATE_EXECUTING_TURN
} robot_state_t;

/* ==============================
 * Shared State
 * ============================== */
static volatile robot_state_t g_state           = STATE_LINE_FOLLOWING;
static volatile bool          g_system_active   = true;
static volatile bool          g_obstacle_flag   = false;
static volatile char          g_pending_turn[10]= "";

static SemaphoreHandle_t g_obstacle_mutex;
static SemaphoreHandle_t g_state_mutex;
static SemaphoreHandle_t g_turn_mutex;

/* ==============================
 * Prototypes
 * ============================== */
static void obstacle_detection_task_(void *pv);
static void barcode_detection_task_(void *pv);
static void junction_detection_task_(void *pv);
static void robot_control_task_(void *pv);
static void wifi_connection_task_(void *pv);
static void init_task_(void *pv);

static void execute_turn_(const char *dir);
static void initialize_all_systems_(void);
static void snapshot_publish_(const char *state_str);

/* ==============================
 * Snapshot Telemetry
 * ============================== */
static void snapshot_publish_(const char *state_str)
{
    if (!mqtt_is_connected()) return;
    float speed    = get_current_speed_cm_s();
    float distance = get_total_distance_cm();
    float ultra    = ultrasonic_get_distance_cm();
    float direction= 0.0f;
    float yaw      = get_heading_fast(&direction);
    mqtt_publish_telemetry(speed, distance, yaw, ultra, state_str);
}

/* ==============================
 * Tasks
 * ============================== */
static void obstacle_detection_task_(void *pv)
{
    (void)pv;
    const TickType_t interval = pdMS_TO_TICKS(300);
    while (1)
    {
        if (ultrasonic_detect_obstacle_fast())
        {
            xSemaphoreTake(g_obstacle_mutex, portMAX_DELAY);
            g_obstacle_flag = true;
            xSemaphoreGive(g_obstacle_mutex);
        }
        vTaskDelay(interval);
    }
}

static void barcode_detection_task_(void *pv)
{
    (void)pv;
    const TickType_t interval = pdMS_TO_TICKS(100);
    while (1)
    {
        if (barcode_capture_ready())
        {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            if (g_state == STATE_LINE_FOLLOWING)
            {
                g_state = STATE_BARCODE_SCANNING;
                all_stop();
                sleep_ms(500);
                drive_signed(-30.0f * 1.25f, -30.0f);
                sleep_ms(400);
                all_stop();
            }
            xSemaphoreGive(g_state_mutex);
        }
        vTaskDelay(interval);
    }
}

static void junction_detection_task_(void *pv)
{
    (void)pv;
    const TickType_t interval = pdMS_TO_TICKS(50);
    while (1)
    {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool check_junction = (g_state == STATE_WAITING_FOR_JUNCTION);
        xSemaphoreGive(g_state_mutex);
        if (check_junction && barcode_capture_ready())
        {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_state = STATE_EXECUTING_TURN;
            xSemaphoreGive(g_state_mutex);
        }
        vTaskDelay(interval);
    }
}

/* ==============================
 * Turn Execution
 * ============================== */
static void execute_turn_(const char *dir)
{
    all_stop();
    sleep_ms(500);

    if (strcmp(dir, "RIGHT") == 0)
    {
        drive_signed(40.0f, 40.0f);
        sleep_ms(400);
        drive_signed(-40.0f, 60.0f);
        sleep_ms(400);
    }
    else
    {
        drive_signed(40.0f, -40.0f);
        sleep_ms(400);
    }
    all_stop();
    sleep_ms(800);
}

/* ==============================
 * Control Task
 * ============================== */
static void robot_control_task_(void *pv)
{
    (void)pv;
    uint32_t last_telemetry_ms     = 0;
    uint32_t last_speed_update_ms  = 0;
    bool obstacle_done             = false;

    speed_calc_init();
    reset_total_distance();

    while (g_system_active)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if ((now - last_speed_update_ms) >= 100)
        {
            update_speed_and_distance_();
            last_speed_update_ms = now;
        }

        if (mqtt_is_connected() && (now - last_telemetry_ms >= 2000))
        {
            const char *st = "";
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            robot_state_t local = g_state;
            xSemaphoreGive(g_state_mutex);

            switch (local)
            {
                case STATE_LINE_FOLLOWING:     st = "LINE"; break;
                case STATE_OBSTACLE_AVOIDANCE: st = "AVOID"; break;
                case STATE_BARCODE_SCANNING:   st = "SCAN"; break;
                case STATE_WAITING_FOR_JUNCTION: st = "WAIT"; break;
                case STATE_EXECUTING_TURN:     st = "TURN"; break;
                default:                       st = "UNK";  break;
            }
            snapshot_publish_(st);
            last_telemetry_ms = now;
        }

        robot_state_t local_state;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        local_state = g_state;
        xSemaphoreGive(g_state_mutex);

        switch (local_state)
        {
            case STATE_LINE_FOLLOWING:
            {
                follow_line_simple();
                bool obs_found = false;
                xSemaphoreTake(g_obstacle_mutex, portMAX_DELAY);
                obs_found = g_obstacle_flag;
                g_obstacle_flag = false;
                xSemaphoreGive(g_obstacle_mutex);

                if (!obstacle_done && obs_found)
                {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state = STATE_OBSTACLE_AVOIDANCE;
                    xSemaphoreGive(g_state_mutex);
                    all_stop();
                    sleep_ms(300);
                    snapshot_publish_("OBS_DET");
                }
                break;
            }
            case STATE_OBSTACLE_AVOIDANCE:
            {
                avoid_obstacle_only();
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state = STATE_LINE_FOLLOWING;
                xSemaphoreGive(g_state_mutex);
                obstacle_done = true;
                snapshot_publish_("AVOID_DONE");
                break;
            }
            case STATE_BARCODE_SCANNING:
            {
                snapshot_publish_("SCANNING");
                barcode_result_t scan;
                if (barcode_decode_captured(&scan) && scan.valid)
                {
                    char dir[8] = "RIGHT"; /* default */
                    /* Example: decode direction from data (simple heuristic) */
                    if ((scan.length > 0) && (scan.data[0] == 'L'))
                    {
                        strcpy(dir, "LEFT");
                    }
                    xSemaphoreTake(g_turn_mutex, portMAX_DELAY);
                    strncpy(g_pending_turn, dir, sizeof(g_pending_turn) - 1);
                    xSemaphoreGive(g_turn_mutex);

                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state = STATE_WAITING_FOR_JUNCTION;
                    xSemaphoreGive(g_state_mutex);

                    snapshot_publish_("BC_DONE");
                }
                else
                {
                    /* Fallback default RIGHT */
                    xSemaphoreTake(g_turn_mutex, portMAX_DELAY);
                    strncpy(g_pending_turn, "RIGHT", sizeof(g_pending_turn) - 1);
                    xSemaphoreGive(g_turn_mutex);

                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state = STATE_WAITING_FOR_JUNCTION;
                    xSemaphoreGive(g_state_mutex);

                    snapshot_publish_("BC_FAIL");
                }
                break;
            }
            case STATE_WAITING_FOR_JUNCTION:
            {
                follow_line_simple();
                break;
            }
            case STATE_EXECUTING_TURN:
            {
                char turn_dir[10];
                xSemaphoreTake(g_turn_mutex, portMAX_DELAY);
                strncpy(turn_dir, g_pending_turn, sizeof(turn_dir));
                xSemaphoreGive(g_turn_mutex);

                execute_turn_(turn_dir);

                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state = STATE_LINE_FOLLOWING;
                xSemaphoreGive(g_state_mutex);

                snapshot_publish_("TURN_DONE");
                sleep_ms(400);
                break;
            }
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

/* ==============================
 * System Init
 * ============================== */
static void initialize_all_systems_(void)
{
    stdio_init_all();
    sleep_ms(1500);

    g_obstacle_mutex = xSemaphoreCreateMutex();
    g_state_mutex    = xSemaphoreCreateMutex();
    g_turn_mutex     = xSemaphoreCreateMutex();

    motor_encoder_init();
    ultrasonic_init();
    ir_init(NULL);
    barcode_init();
    speed_calc_init();
    imu_init();
    servo_init_();
}

/* ==============================
 * WiFi Task
 * ============================== */
static void wifi_connection_task_(void *pv)
{
    (void)pv;
    if (!wifi_and_mqtt_start())
    {
        printf("[NET] WiFi/MQTT failed\n");
    }
    xTaskCreate(obstacle_detection_task_, "obs_det",
                1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(barcode_detection_task_, "bc_det",
                1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(junction_detection_task_, "jn_det",
                1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(robot_control_task_, "robot_ctl",
                4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskDelete(NULL);
}

/* ==============================
 * Init Task
 * ============================== */
static void init_task_(void *pv)
{
    (void)pv;
    vTaskDelay(pdMS_TO_TICKS(100));
    initialize_all_systems_();
    xTaskCreate(wifi_connection_task_, "wifi_conn",
                2048, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskDelete(NULL);
}

/* ==============================
 * main()
 * ============================== */
int main(void)
{
    xTaskCreate(init_task_, "init",
                2048, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();
    while (1)
    {
        tight_loop_contents();
    }
    return 0;
}

/*** end of file ***/
