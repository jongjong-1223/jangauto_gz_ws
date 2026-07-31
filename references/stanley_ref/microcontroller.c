
#include "microcontroller.h"
#include "stanley_module.h"
#include "pi_module.h"
#include "ekf_module.h"

//// Global variables //// start
double gps_pos_x;
double gps_pos_y;
double imu_yaw;
double imu_pitch;
double imu_accel_x;
double imu_accel_y;
double imu_omega_yaw;
double ekf_vel_vx;
double enable;
double path_select;
double theta_motor_L;
double theta_motor_R;
double ref_vx;

double alpha_motor_L_meas;
double alpha_motor_R_meas;
//// Global variables //// end

/* ── Sensor preprocessing static state ─────────────────────────── */
static double s_prev_theta_motor_L;
static double s_prev_theta_motor_R;
static double s_prev_omega_meas_L;
static double s_prev_omega_meas_R;

/* Derived sensor quantities (LPF state) */
static double s_omega_motor_L_meas;
static double s_omega_motor_R_meas;
static double s_ref_vx_lpf;

/* ── Module output structs ──────────────────────────────────────── */
static StanleyModuleOut s_s_out;
static PIModuleOut      s_p_out;
static EKFModuleOut     s_e_out;


void init_software(void)
{
    gps_pos_x = 0.0;
    gps_pos_y = 0.0;
    imu_yaw = 0.0;
    imu_pitch = 0.0;
    imu_accel_x = 0.0;
    imu_accel_y = 0.0;
    imu_omega_yaw = 0.0;
    ekf_vel_vx = 0.0;
    enable = 0.0;
    path_select = 0.0;
    theta_motor_L = 0.0;
    theta_motor_R = 0.0;
    ref_vx = 0.0;

    alpha_motor_L_meas = alpha_motor_R_meas = 0.0;

    s_prev_theta_motor_L = 0.0;
    s_prev_theta_motor_R = 0.0;
    s_prev_omega_meas_L  = 0.0;
    s_prev_omega_meas_R  = 0.0;
    s_omega_motor_L_meas = 0.0;
    s_omega_motor_R_meas = 0.0;
    s_ref_vx_lpf         = 0.0;

    stanley_module_init();
    pi_module_init(10.5, -10.0);
    ekf_module_init(25000.0);
}


void microcontroller(const double * adc, double * dac)
{
    int i;

    /* ── Inputs ─────────────────────────────────────────────────── */
    gps_pos_x     = *(adc);
    gps_pos_y     = *(adc+1);
    imu_yaw       = *(adc+2);
    imu_pitch     = *(adc+3);
    imu_accel_x   = *(adc+4);
    imu_accel_y   = *(adc+5);
    imu_omega_yaw = *(adc+6);
    ekf_vel_vx    = *(adc+7);
    enable        = *(adc+8);
    path_select   = *(adc+9);
    theta_motor_L = *(adc+10);
    theta_motor_R = *(adc+11);
    ref_vx        = *(adc+12);

    /* ── Encoder-based angular velocity ─────────────────────────── */
    s_omega_motor_L_meas = (theta_motor_L - s_prev_theta_motor_L) * FS;
    s_omega_motor_R_meas = (theta_motor_R - s_prev_theta_motor_R) * FS;
    s_prev_theta_motor_L = theta_motor_L;
    s_prev_theta_motor_R = theta_motor_R;

    /* ── Encoder-based alpha (finite difference + LPF) ─────────── */
    {
        double alpha_raw_L  = (s_omega_motor_L_meas - s_prev_omega_meas_L) * FS;
        double alpha_raw_R  = (s_omega_motor_R_meas - s_prev_omega_meas_R) * FS;
        s_prev_omega_meas_L = s_omega_motor_L_meas;
        s_prev_omega_meas_R = s_omega_motor_R_meas;
        alpha_motor_L_meas  = (1.0 - LPF_ALPHA_ONEOFONETAU) * alpha_motor_L_meas
                            + LPF_ALPHA_ONEOFONETAU * alpha_raw_L;
        alpha_motor_R_meas  = (1.0 - LPF_ALPHA_ONEOFONETAU) * alpha_motor_R_meas
                            + LPF_ALPHA_ONEOFONETAU * alpha_raw_R;
    }

    /* ── Enable guard ────────────────────────────────────────────── */
    if (enable <= 0.5) {
        s_ref_vx_lpf = 0.0;
        s_s_out.omega_L_cmd       = 0.0;
        s_s_out.omega_R_cmd       = 0.0;
        s_s_out.omega_motor_L_cmd = 0.0;
        s_s_out.omega_motor_R_cmd = 0.0;
        s_s_out.omega_z_cmd       = 0.0;
        pi_module_reset();
        s_p_out.torque_e_L  = 0.0;
        s_p_out.torque_e_R  = 0.0;
        s_p_out.torque_ff_z = 0.0;
        for (i = 0; i < 10; i++) { s_e_out.ekf_out[i] = 0.0; }
    } else {

        s_ref_vx_lpf = (1.0 - LPF_ALPHA_ONETAU) * s_ref_vx_lpf
                     + LPF_ALPHA_ONETAU * ref_vx;

        stanley_module_update(gps_pos_x, gps_pos_y, imu_yaw,
                              ekf_vel_vx, s_ref_vx_lpf, path_select, &s_s_out);

        pi_module_update(s_s_out.omega_motor_L_cmd, s_s_out.omega_motor_R_cmd,
                         s_omega_motor_L_meas, s_omega_motor_R_meas,
                         s_s_out.omega_z_cmd, &s_p_out);

        ekf_module_update(s_p_out.torque_e_L, s_p_out.torque_e_R,
                          s_p_out.torque_ff_z,
                          theta_motor_L, theta_motor_R,
                          s_omega_motor_L_meas, s_omega_motor_R_meas,
                          alpha_motor_L_meas, alpha_motor_R_meas,
                          ekf_vel_vx, imu_omega_yaw, &s_e_out);
    }

    /* ── DAC output ─────────────────────────────────────────────── */
    dac[0]  = s_s_out.omega_L_cmd;          /* omega_L_cmd        [rad/s]    */
    dac[1]  = s_s_out.omega_R_cmd;          /* omega_R_cmd        [rad/s]    */
    dac[2]  = s_p_out.torque_e_L - s_p_out.torque_ff_z; /* torque_e_L [N*m] */
    dac[3]  = s_p_out.torque_e_R + s_p_out.torque_ff_z; /* torque_e_R [N*m] */
    // if (enable <= 0.5) {
    //     dac[2]  = 0.0; /* torque_e_L [N*m] */
    //     dac[3]  = 0.0; /* torque_e_R [N*m] */
    // }
    // else {
    //     dac[2]  = -3700.0; /* torque_e_L [N*m] */
    //     dac[3]  = 3700.0; /* torque_e_R [N*m] */
    // }
    dac[4]  = s_s_out.ref_wp_x;             /* ref_waypoint_x     [m]        */
    dac[5]  = s_s_out.ref_wp_y;             /* ref_waypoint_y     [m]        */
    dac[6]  = s_omega_motor_L_meas;         /* encoder omega_L    [rad/s]    */
    dac[7]  = s_omega_motor_R_meas;         /* encoder omega_R    [rad/s]    */
    dac[8]  = s_e_out.ekf_out[0];           /* EKF_L: Omega       [rad/s]    */
    dac[9]  = s_e_out.ekf_out[1];           /* EKF_L: J           [kg*m^2]   */
    dac[10] = s_e_out.ekf_out[2];           /* EKF_L: TL          [N*m]      */
    dac[11] = s_e_out.ekf_out[3];           /* EKF_L: B           [N*m*s/rad]*/
    dac[12] = s_e_out.ekf_out[4];           /* EKF_R: Omega       [rad/s]    */
    dac[13] = s_e_out.ekf_out[5];           /* EKF_R: J           [kg*m^2]   */
    dac[14] = s_e_out.ekf_out[6];           /* EKF_R: TL          [N*m]      */
    dac[15] = s_e_out.ekf_out[7];           /* EKF_R: B           [N*m*s/rad]*/
    dac[16] = s_e_out.ekf_out[8];           /* EKF_L: theta                  */
    dac[17] = s_e_out.ekf_out[9];           /* EKF_R: theta                  */
    dac[18] = alpha_motor_L_meas;           /* 엔코더 각가속도 (좌) [rad/s^2] */
    dac[19] = alpha_motor_R_meas;           /* 엔코더 각가속도 (우) [rad/s^2] */
    dac[20] = s_e_out.T_load_L;             /* T_load_L          [N*m]       */
    dac[21] = s_e_out.T_load_R;             /* T_load_R          [N*m]       */
    dac[22] = s_e_out.T_M_inv_2;            /* T_M_inv_2         [N*m]       */
    dac[23] = s_ref_vx_lpf;                /* x방향 선속도 지령 스무딩        */
    dac[24] = s_e_out.vx_hat;              /* x방향 선속도 옵저버 추정        */
    dac[25] = s_e_out.accel_vx_hat;        /* x방향 가속도 옵저버 추정        */
}
