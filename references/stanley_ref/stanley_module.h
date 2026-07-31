#ifndef STANLEY_MODULE_H
#define STANLEY_MODULE_H

typedef struct {
    double omega_L_cmd;       /* left  wheel angular velocity command  [rad/s] */
    double omega_R_cmd;       /* right wheel angular velocity command  [rad/s] */
    double omega_motor_L_cmd; /* left  motor angular velocity command  [rad/s] */
    double omega_motor_R_cmd; /* right motor angular velocity command  [rad/s] */
    double omega_z_cmd;       /* yaw rate command (v_R - v_L) / B      [rad/s] */
    double ref_wp_x;          /* current reference waypoint x           [m]    */
    double ref_wp_y;          /* current reference waypoint y           [m]    */
} StanleyModuleOut;

void stanley_module_init(void);

void stanley_module_update(
    double gps_pos_x,
    double gps_pos_y,
    double imu_yaw,
    double ekf_vel_vx,
    double ref_vx,
    double path_select,
    StanleyModuleOut *out
);

#endif /* STANLEY_MODULE_H */
