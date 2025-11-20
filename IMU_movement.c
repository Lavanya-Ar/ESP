/** @file IMU_movement.c
 *  @brief Yaw PID control and IMU telemetry publishing (accelerometer + magnetometer).
 *
 *  NOTE: Progressive PID retained; integral limited. Barr-C formatting applied.
 *  WARNING: Gains and setpoint tuned for specific robot orientation.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#include "IMU_movement.h"
#include "imu_raw_demo.h"
#include "mqtt_client.h"
#include "motor_encoder_demo.h"
#include "encoder.h"

/* ==============================
 * Filter Configuration
 * ============================== */
#define FILTER_SIZE              (10u)
#define PID_PROGRESS_SMALL_ERR   (10.0f)
#define PID_PROGRESS_MED_ERR     (25.0f)
#define PID_PROGRESS_LARGE_ERR   (45.0f)

/* ==============================
 * PID Config (Public Accessors unchanged)
 * ============================== */
static pid_config_t pid_config =
{
    .kp              = 0.38f,
    .ki              = 0.005f,
    .kd              = 0.20f,
    .setpoint        = 140.0f,
    .base_speed_left = 30.0f * 1.25f,
    .base_speed_right= 30.0f,
    .max_output      = 15.0f,
    .integral_max    = 30.0f
};

static pid_state_t pid_state =
{
    .prev_error = 0.0f,
    .integral   = 0.0f,
    .enabled    = false
};

/* ==============================
 * Speed / Distance (IMU Mode)
 * ============================== */
static volatile uint32_t g_imu_last_speed_calc_time = 0;
static volatile float    g_imu_current_speed_cm_s   = 0.0f;
static volatile float    g_imu_last_distance_cm     = 0.0f;
static volatile uint32_t g_imu_last_pub_ms          = 0;

/* ==============================
 * Filter Histories
 * ============================== */
static int filter_index = 0;
static int16_t ax_hist[FILTER_SIZE] = {0};
static int16_t ay_hist[FILTER_SIZE] = {0};
static int16_t az_hist[FILTER_SIZE] = {0};

/* ==============================
 * Private Prototypes
 * ============================== */
static int16_t apply_filter_(int16_t raw, int16_t *history, int *idx);
static void    calc_orientation_(int16_t ax, int16_t ay, int16_t az,
                                 float *pitch, float *roll);
static float   calc_yaw_(int16_t mx, int16_t my, int16_t mz,
                         float pitch_deg, float roll_deg);
static float   normalize_angle_(float angle);
static float   angle_error_(float current, float target);
static void    publish_imu_telemetry_(const imu_data_t *data,
                                      float error, float pid_out,
                                      float ls, float rs,
                                      int16_t ax_raw, int16_t ay_raw,
                                      int16_t az_raw,
                                      int16_t ax_f, int16_t ay_f,
                                      int16_t az_f);
static float   progressive_pid_(float current_yaw,
                                pid_config_t *cfg,
                                pid_state_t *st);
static void    drive_to_setpoint_(float current_yaw,
                                  pid_config_t *cfg,
                                  pid_state_t *st,
                                  float *pid_out,
                                  float *ls,
                                  float *rs);

/* ==============================
 * Speed Calculation Helpers
 * ============================== */
void imu_speed_calc_init(void)
{
    encoders_init(true);
    g_imu_last_speed_calc_time = to_ms_since_boot(get_absolute_time());
    g_imu_current_speed_cm_s   = 0.0f;
    g_imu_last_distance_cm     = 0.0f;
    printf("[IMU_SPEED] init\n");
}

static void imu_update_speed_and_distance_(void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t dt_ms = now - g_imu_last_speed_calc_time;
    if (dt_ms < 100) return;

    float left  = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float current_dist = (left + right) * 0.5f;

    float dist_travel = current_dist - g_imu_last_distance_cm;
    float dt_s = dt_ms / 1000.0f;
    if (dt_s > 0.0f)
    {
        g_imu_current_speed_cm_s = dist_travel / dt_s;
        if (g_imu_current_speed_cm_s < 0.0f)    g_imu_current_speed_cm_s = 0.0f;
        if (g_imu_current_speed_cm_s > 200.0f)  g_imu_current_speed_cm_s = 200.0f;
    }
    else
    {
        g_imu_current_speed_cm_s = 0.0f;
    }

    g_imu_last_distance_cm     = current_dist;
    g_imu_last_speed_calc_time = now;
}

float imu_get_current_speed_cm_s(void)
{
    return g_imu_current_speed_cm_s;
}

float imu_get_total_distance_cm(void)
{
    float left  = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    return (left + right) * 0.5f;
}

void imu_reset_total_distance(void)
{
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    g_imu_last_distance_cm = 0.0f;
}

/* ==============================
 * Filtering / Orientation
 * ============================== */
static int16_t apply_filter_(int16_t raw, int16_t *history, int *idx)
{
    history[*idx] = raw;
    *idx = (*idx + 1) % FILTER_SIZE;
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += history[i];
    return (int16_t)(sum / FILTER_SIZE);
}

static void calc_orientation_(int16_t ax, int16_t ay, int16_t az,
                              float *pitch, float *roll)
{
    *pitch = atan2f(-ax, sqrtf((float)(ay * ay + az * az))) * 180.0f / (float)M_PI;
    *roll  = atan2f((float)ay, (float)az) * 180.0f / (float)M_PI;
}

static float calc_yaw_(int16_t mx, int16_t my, int16_t mz,
                       float pitch_deg, float roll_deg)
{
    float pitch = pitch_deg * M_PI / 180.0f;
    float roll  = roll_deg  * M_PI / 180.0f;

    float Mx_comp = mx * cosf(pitch) + mz * sinf(pitch);
    float My_comp = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) -
                    mz * sinf(roll) * cosf(pitch);

    float yaw = atan2f(-My_comp, Mx_comp) * 180.0f / M_PI;
    if (yaw < 0.0f) yaw += 360.0f;
    return yaw;
}

static float normalize_angle_(float angle)
{
    while (angle < 0.0f)   angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static float angle_error_(float current, float target)
{
    float err = target - current;
    if (err > 180.0f)  err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

/* ==============================
 * Telemetry Publishing
 * ============================== */
static void publish_imu_telemetry_(const imu_data_t *data,
                                   float error, float pid_out,
                                   float ls, float rs,
                                   int16_t ax_raw, int16_t ay_raw,
                                   int16_t az_raw,
                                   int16_t ax_f, int16_t ay_f,
                                   int16_t az_f)
{
    if (!mqtt_is_connected()) return;

    float speed    = imu_get_current_speed_cm_s();
    float distance = imu_get_total_distance_cm();
    float yaw_deg  = data->yaw;

    char state[160];
    snprintf(state, sizeof(state),
             "Y:%.1f SP:%.1f E:%.1f PID:%.1f L:%.1f R:%.1f AR:(%d,%d,%d) AF:(%d,%d,%d)",
             yaw_deg, pid_config.setpoint, error, pid_out, ls, rs,
             ax_raw, ay_raw, az_raw, ax_f, ay_f, az_f);

    mqtt_publish_telemetry(speed, distance, yaw_deg, 0.0f, state);
}

/* ==============================
 * Data Acquisition
 * ============================== */
bool read_imu_data(imu_data_t *data)
{
    if (data == NULL) return false;
    memset(data, 0, sizeof(*data));

    int16_t ax, ay, az;
    if (!read_accel_raw(&ax, &ay, &az)) return false;

    int16_t ax_f = apply_filter_(ax, ax_hist, &filter_index);
    int16_t ay_f = apply_filter_(ay, ay_hist, &filter_index);
    int16_t az_f = apply_filter_(az, az_hist, &filter_index);

    int16_t mx, my, mz;
    if (!read_mag_raw(&mx, &my, &mz)) return false;

    float pitch, roll;
    calc_orientation_(ax_f, ay_f, az_f, &pitch, &roll);
    float yaw = calc_yaw_(mx, my, mz, pitch, roll);

    data->pitch   = normalize_angle_(pitch);
    data->roll    = normalize_angle_(roll);
    data->yaw     = normalize_angle_(yaw);
    data->accel_x = ax_f / 1000.0f;
    data->accel_y = ay_f / 1000.0f;
    data->accel_z = az_f / 1000.0f;
    data->mag_x   = (float)mx;
    data->mag_y   = (float)my;
    data->mag_z   = (float)mz;
    return true;
}

void print_imu_data(const imu_data_t *data)
{
    if (!data) return;
    printf("[IMU] Pitch:%6.1f Roll:%6.1f Yaw:%6.1f\n",
           data->pitch, data->roll, data->yaw);
}

/* ==============================
 * PID / Control
 * ============================== */
void pid_controller_init(pid_config_t *cfg, pid_state_t *st)
{
    if (cfg == NULL) cfg = &pid_config;
    if (st == NULL)  st  = &pid_state;
    st->prev_error = 0.0f;
    st->integral   = 0.0f;
    st->enabled    = true;
    printf("[PID] init KP:%.3f KI:%.3f KD:%.3f SP:%.1f\n",
           cfg->kp, cfg->ki, cfg->kd, cfg->setpoint);
}

static float progressive_pid_(float current_yaw,
                              pid_config_t *cfg,
                              pid_state_t *st)
{
    if (!st->enabled || (cfg == NULL)) return 0.0f;

    float error = angle_error_(current_yaw, cfg->setpoint);
    float kp, kd;
    if (fabsf(error) < PID_PROGRESS_SMALL_ERR)
    {
        kp = 0.30f; kd = 0.25f;
    }
    else if (fabsf(error) < PID_PROGRESS_MED_ERR)
    {
        kp = 0.20f; kd = 0.15f;
    }
    else if (fabsf(error) < PID_PROGRESS_LARGE_ERR)
    {
        kp = 0.15f; kd = 0.10f;
    }
    else
    {
        kp = 0.12f; kd = 0.08f;
    }

    if (fabsf(error) < PID_PROGRESS_LARGE_ERR)
    {
        st->integral += error;
    }
    else
    {
        st->integral = 0.0f;
    }

    if (st->integral > cfg->integral_max)  st->integral = cfg->integral_max;
    if (st->integral < -cfg->integral_max) st->integral = -cfg->integral_max;

    float proportional = kp * error;
    float integral     = cfg->ki * st->integral;
    float derivative   = kd * (error - st->prev_error);
    st->prev_error     = error;

    float out = proportional + integral + derivative;
    if (out >  cfg->max_output) out =  cfg->max_output;
    if (out < -cfg->max_output) out = -cfg->max_output;
    return out;
}

static void drive_to_setpoint_(float current_yaw,
                               pid_config_t *cfg,
                               pid_state_t *st,
                               float *pid_out,
                               float *ls,
                               float *rs)
{
    if (!st->enabled)
    {
        *pid_out = 0.0f;
        *ls      = 0.0f;
        *rs      = 0.0f;
        return;
    }
    *pid_out = progressive_pid_(current_yaw, cfg, st);
    *ls = cfg->base_speed_left  + *pid_out;
    *rs = cfg->base_speed_right - *pid_out;

    if (*ls > 100.0f)  *ls = 100.0f;
    if (*ls < -100.0f) *ls = -100.0f;
    if (*rs > 100.0f)  *rs = 100.0f;
    if (*rs < -100.0f) *rs = -100.0f;

    drive_signed(*ls, *rs);
}

pid_config_t *get_pid_config(void) { return &pid_config; }
pid_state_t  *get_pid_state(void)  { return &pid_state; }

bool detect_initial_setpoint(int samples)
{
    float sum_yaw = 0.0f;
    int ok_samples = 0;
    for (int i = 0; i < samples; i++)
    {
        imu_data_t data;
        if (read_imu_data(&data))
        {
            sum_yaw += data.yaw;
            ok_samples++;
        }
        sleep_ms(100);
    }
    if (ok_samples > 0)
    {
        pid_config.setpoint = sum_yaw / ok_samples;
        return true;
    }
    return false;
}

void set_yaw_setpoint(float target_yaw)
{
    pid_config.setpoint = normalize_angle_(target_yaw);
}

/* ==============================
 * Task Loop
 * ============================== */
void imu_movement_task(__unused void *params)
{
    imu_movement_init();
    imu_data_t current;
    int16_t ax_raw=0, ay_raw=0, az_raw=0;
    int16_t ax_f=0, ay_f=0, az_f=0;

    while (true)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (read_imu_data(&current))
        {
            if (read_accel_raw(&ax_raw, &ay_raw, &az_raw))
            {
                ax_f = apply_filter_(ax_raw, ax_hist, &filter_index);
                ay_f = apply_filter_(ay_raw, ay_hist, &filter_index);
                az_f = apply_filter_(az_raw, az_hist, &filter_index);
            }

            float pid_out, ls, rs;
            drive_to_setpoint_(current.yaw, &pid_config, &pid_state,
                               &pid_out, &ls, &rs);
            float error = angle_error_(current.yaw, pid_config.setpoint);

            imu_update_speed_and_distance_();

            if (mqtt_is_connected() && (now - g_imu_last_pub_ms >= 500))
            {
                g_imu_last_pub_ms = now;
                publish_imu_telemetry_(&current, error, pid_out,
                                       ls, rs,
                                       ax_raw, ay_raw, az_raw,
                                       ax_f, ay_f, az_f);
            }
        }
        mqtt_loop_poll();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ==============================
 * Initialization
 * ============================== */
void imu_movement_init(void)
{
    imu_init();
    imu_speed_calc_init();
}

static void imu_robot_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!wifi_and_mqtt_start())
    {
        printf("[NET] no MQTT\n");
    }
    imu_movement_init();
    motor_encoder_init();

    if (!detect_initial_setpoint(10))
    {
        pid_config.setpoint = 140.0f;
    }
    pid_controller_init(NULL, NULL);
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}

/*** end of file ***/
