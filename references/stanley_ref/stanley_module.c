#include "stanley_module.h"
#include "microcontroller.h"
#include <math.h>
#include <string.h>

/* ── Path references ────────────────────────────────────────────── */
extern const WayPoint ZIGZAG_PATH[];
extern const WayPoint SQUARE_V1_PATH[];
#define ZIGZAG_PATH_LEN      429
#define SQUARE_V1_PATH_LEN   193

/* ── Vehicle geometry ───────────────────────────────────────────── */
#define WHEELBASE    1.685
#define TRACK_WIDTH  2.200
#define WHEEL_RADIUS 0.350
#define HALF_B       1.100

#define WP_WINDOW    30   /* look-ahead window size [waypoints] */

/* ── Module-private state ───────────────────────────────────────── */
static StanleyParams s_sparams;
static TrackParams   s_tparams;
static int           s_nearest_idx;
static int           s_active_path_id;

void stanley_module_init(void)
{
    s_nearest_idx    = 0;
    s_active_path_id = 0;

    stanley_params_default(&s_sparams);
    s_sparams.k             = 0.5;
    s_sparams.k_soft        = 1.0;
    s_sparams.max_steer_rad = atan(WHEELBASE / HALF_B);

    track_params_default(&s_tparams);
    s_tparams.wheelbase       = WHEELBASE;
    s_tparams.track_width     = TRACK_WIDTH;
    s_tparams.max_track_speed = 4.0;
    s_tparams.min_radius      = HALF_B;
}

void stanley_module_update(
    double gps_pos_x,
    double gps_pos_y,
    double imu_yaw,
    double ekf_vel_vx,
    double ref_vx,
    double path_select,
    StanleyModuleOut *out)
{
    /* ── Active path selection ──────────────────────────────────── */
    int new_path_id = (path_select > 0.5) ? 1 : 0;
    if (new_path_id != s_active_path_id) {
        s_nearest_idx    = 0;
        s_active_path_id = new_path_id;
    }

    const WayPoint *active_path = (s_active_path_id == 1)
                                  ? SQUARE_V1_PATH : ZIGZAG_PATH;
    int active_len              = (s_active_path_id == 1)
                                  ? SQUARE_V1_PATH_LEN : ZIGZAG_PATH_LEN;

    /* ── Vehicle state at front axle ───────────────────────────── */
    VehicleState state;
    state.x     = gps_pos_x + WHEELBASE * cos(imu_yaw);
    state.y     = gps_pos_y + WHEELBASE * sin(imu_yaw);
    state.yaw   = imu_yaw;
    state.speed = ekf_vel_vx;

    /* ── Windowed nearest search (circular) ────────────────────── */
    int            win_start = s_nearest_idx;
    const WayPoint *search_ptr;
    size_t          win_len;
    WayPoint        win_buf[WP_WINDOW + 1];

    if (win_start + WP_WINDOW < active_len) {
        search_ptr = &active_path[win_start];
        win_len    = WP_WINDOW + 1;
    } else {
        int tail = active_len - win_start;
        int head = (WP_WINDOW + 1) - tail;
        if (head > active_len) head = active_len;
        memcpy(win_buf,        &active_path[win_start], (size_t)tail * sizeof(WayPoint));
        memcpy(&win_buf[tail], active_path,             (size_t)head * sizeof(WayPoint));
        search_ptr = win_buf;
        win_len    = (size_t)(tail + head);
    }

    /* ── Zero output by default ─────────────────────────────────── */
    out->omega_L_cmd       = 0.0;
    out->omega_R_cmd       = 0.0;
    out->omega_motor_L_cmd = 0.0;
    out->omega_motor_R_cmd = 0.0;
    out->omega_z_cmd       = 0.0;
    out->ref_wp_x          = active_path[s_nearest_idx].x;
    out->ref_wp_y          = active_path[s_nearest_idx].y;

    /* ── Stanley steering ───────────────────────────────────────── */
    StanleyResult sres;
    if (stanley_compute(&state, search_ptr, win_len,
                        &s_sparams, &sres) != STANLEY_OK) {
        return;
    }

    s_nearest_idx = (win_start + sres.nearest_idx) % active_len;

    /* ── Track mapping ──────────────────────────────────────────── */
    TrackCommand tcmd;
    if (stanley_to_track(sres.steer_rad, ref_vx,
                         &s_tparams, &tcmd) != TRACK_OK) {
        out->ref_wp_x = active_path[s_nearest_idx].x;
        out->ref_wp_y = active_path[s_nearest_idx].y;
        return;
    }

    /* ── Linear speed → wheel angular velocity ──────────────────── */
    out->omega_L_cmd       = tcmd.v_left  / WHEEL_RADIUS;
    out->omega_R_cmd       = tcmd.v_right / WHEEL_RADIUS;
    out->omega_motor_L_cmd = out->omega_L_cmd * N_GEAR;
    out->omega_motor_R_cmd = out->omega_R_cmd * N_GEAR;
    out->omega_z_cmd       = (tcmd.v_right - tcmd.v_left) / TRACK_WIDTH;
    out->ref_wp_x          = active_path[s_nearest_idx].x;
    out->ref_wp_y          = active_path[s_nearest_idx].y;
}
