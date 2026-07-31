#ifndef STANLEY_H
#define STANLEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * stanley.h  –  Stanley Path Tracking Controller
 *
 * Reference:
 *   Thrun et al., "Stanley: The robot that won the DARPA
 *   Grand Challenge" (2006)
 *
 * Coordinate convention
 *   - x : forward (East in global frame)
 *   - y : left    (North in global frame)
 *   - yaw: counter-clockwise positive, radians
 * ========================================================= */

#include <stddef.h>   /* size_t */

/* ── Types ─────────────────────────────────────────────── */

/** A single waypoint on the reference path. */
typedef struct {
    double x;   /**< Global x-coordinate [m]   */
    double y;   /**< Global y-coordinate [m]   */
    double yaw; /**< Path tangent direction [rad] */
} WayPoint;

/** Vehicle state at the front axle. */
typedef struct {
    double x;        /**< Front-axle x  [m]   */
    double y;        /**< Front-axle y  [m]   */
    double yaw;      /**< Vehicle heading [rad] */
    double speed;    /**< Longitudinal speed [m/s] */
} VehicleState;

/** Tuning parameters for the Stanley controller. */
typedef struct {
    double k;              /**< Cross-track gain (> 0)               */
    double k_soft;         /**< Softening constant to avoid div/0    */
    double max_steer_rad;  /**< Steering saturation limit [rad]      */
} StanleyParams;

/** Result returned from the controller. */
typedef struct {
    double steer_rad;      /**< Commanded steering angle [rad]       */
    double heading_error;  /**< ψe  [rad]                            */
    double cross_track_error; /**< e   [m]  (positive = left of path) */
    int    nearest_idx;    /**< Index of the nearest waypoint        */
} StanleyResult;

/** Return codes. */
typedef enum {
    STANLEY_OK            =  0,
    STANLEY_ERR_NULL_PTR  = -1,
    STANLEY_ERR_NO_PATH   = -2,
    STANLEY_ERR_LOW_SPEED = -3
} StanleyStatus;

/* ── Track (differential-drive) types ──────────────────── */

/**
 * Physical geometry of a tracked / differential-drive vehicle.
 *
 *   wheelbase   (L)  – longitudinal distance between front and rear axles [m].
 *                      For a pure tracked vehicle use the equivalent wheelbase
 *                      that matches your empirical turning behaviour.
 *   track_width (B)  – lateral distance between the two track contact centres [m].
 *   max_track_speed  – absolute speed limit for either individual track [m/s].
 *   min_radius       – hard lower bound on |R|; MUST satisfy min_radius >= B/2.
 *                      Setting min_radius == B/2 allows pivot turns (one track
 *                      reversed).  Larger values forbid sharp turns.
 */
typedef struct {
    double wheelbase;        /**< L  [m]                 */
    double track_width;      /**< B  [m]                 */
    double max_track_speed;  /**< v_max per track [m/s]  */
    double min_radius;       /**< R_min >= B/2    [m]    */
} TrackParams;

/**
 * Left / right track velocity commands produced by the mapping layer.
 *
 * Positive values = forward motion of that track.
 * The diagnostic flags help callers detect when physical limits were hit.
 */
typedef struct {
    double v_left;          /**< Left  track target speed [m/s]           */
    double v_right;         /**< Right track target speed [m/s]           */
    double turning_radius;  /**< Actual |R| used after clamping [m]       */
    int    radius_clamped;  /**< 1 if R was raised to min_radius          */
    int    speed_scaled;    /**< 1 if speeds were scaled down for sat.    */
} TrackCommand;

/** Return codes for the track-mapping layer. */
typedef enum {
    TRACK_OK             =  0,
    TRACK_ERR_NULL_PTR   = -1,
    TRACK_ERR_BAD_PARAMS = -2   /**< e.g. track_width <= 0 or min_radius < B/2 */
} TrackStatus;

/* ── Public API ─────────────────────────────────────────── */

/**
 * @brief  Initialise default parameters.
 *
 * Safe to call before any other function.
 * Sets  k = 0.5,  k_soft = 1.0,  max_steer = 30° (~0.524 rad).
 *
 * @param params  Output parameter struct (must not be NULL).
 */
void stanley_params_default(StanleyParams *params);

/**
 * @brief  Find the index of the nearest waypoint.
 *
 * Performs a linear search over all waypoints and returns
 * the index of the point closest (Euclidean) to (qx, qy).
 *
 * @param path      Array of waypoints.
 * @param path_len  Number of waypoints.
 * @param qx        Query x-coordinate.
 * @param qy        Query y-coordinate.
 * @return          Index in [0, path_len-1], or -1 on error.
 */
int stanley_nearest_index(const WayPoint *path, size_t path_len,
                          double qx, double qy);

/**
 * @brief  Compute signed cross-track error.
 *
 * Positive when the vehicle is to the LEFT of the path
 * (path looks to the right from the vehicle's perspective).
 *
 * @param path_x    Nearest waypoint x.
 * @param path_y    Nearest waypoint y.
 * @param path_yaw  Path tangent at that waypoint [rad].
 * @param veh_x     Front-axle x.
 * @param veh_y     Front-axle y.
 * @return          Signed cross-track error [m].
 */
double stanley_cross_track_error(double path_x,  double path_y,
                                 double path_yaw,
                                 double veh_x,   double veh_y);

/**
 * @brief  Normalise an angle to the interval [-π, π].
 *
 * @param angle  Input angle [rad].
 * @return       Normalised angle [rad].
 */
double stanley_normalize_angle(double angle);

/**
 * @brief  Run one control step of the Stanley algorithm.
 *
 * δ = ψe + arctan( k·e / (v + k_soft) )
 *
 * The result is clamped to ±max_steer_rad.
 *
 * @param state   Current vehicle state (front-axle position + heading + speed).
 * @param path    Reference path array.
 * @param path_len Number of waypoints.
 * @param params  Controller parameters.
 * @param result  Output struct (steering angle + diagnostics).
 * @return        STANLEY_OK on success, negative error code otherwise.
 */
StanleyStatus stanley_compute(const VehicleState *state,
                              const WayPoint     *path,
                              size_t              path_len,
                              const StanleyParams *params,
                              StanleyResult       *result);

/* ── Track-mapping API ──────────────────────────────────── */

/**
 * @brief  Initialise default TrackParams for a mid-size tracked robot.
 *
 * Defaults:  L = 0.5 m,  B = 0.4 m,  v_max = 2.0 m/s,
 *            min_radius = B/2 = 0.2 m  (pivot turns allowed).
 *
 * @param tp  Output struct (must not be NULL).
 */
void track_params_default(TrackParams *tp);

/**
 * @brief  Map a Stanley steering angle + forward speed → left/right track speeds.
 *
 * Algorithm
 * ---------
 * 1. Straight-line guard:  |δ| < STRAIGHT_THRESH → v_L = v_R = v_fwd.
 * 2. Compute nominal radius:  R_nom = L / tan(δ).
 * 3. Numerical stability clamp:  |R| = max(|R_nom|, min_radius).
 *    This enforces R >= B/2, preventing the inner track from reversing
 *    when min_radius == B/2 is used, or forbidding tighter turns otherwise.
 *    Sets TrackCommand.radius_clamped = 1 if the clamp was applied.
 * 4. Compute raw track speeds from the differential-drive kinematic model:
 *      v_outer = v_fwd * (|R| + B/2) / |R|
 *      v_inner = v_fwd * (|R| - B/2) / |R|
 *    Sign of δ determines which side is outer / inner.
 * 5. Speed saturation:  if max(|v_L|, |v_R|) > max_track_speed,
 *    scale both tracks proportionally so the faster track equals v_max.
 *    This preserves the turning ratio while respecting hardware limits.
 *    Sets TrackCommand.speed_scaled = 1 if scaling was applied.
 *
 * @param steer_rad   Steering angle from stanley_compute() [rad].
 * @param v_fwd       Desired forward (centre-of-vehicle) speed [m/s].
 *                    Negative values command reverse motion.
 * @param tp          Track geometry and limits (must not be NULL).
 * @param cmd         Output track command (must not be NULL).
 * @return            TRACK_OK on success, negative error code otherwise.
 */
TrackStatus stanley_to_track(double steer_rad, double v_fwd,
                              const TrackParams *tp,
                              TrackCommand      *cmd);

#ifdef __cplusplus
}
#endif

#endif /* STANLEY_H */
