/* =========================================================
 * stanley.c  –  Stanley Path Tracking Controller
 *
 * δ(t) = ψe(t) + arctan( k·e(t) / (v(t) + k_soft) )
 *
 *   ψe  = heading error   (path_yaw – vehicle_yaw)
 *   e   = signed cross-track error (left > 0)
 *   v   = vehicle speed   [m/s]
 *   k   = cross-track gain
 *   k_soft = softening term (prevents divide-by-zero at rest)
 * ========================================================= */

#include "stanley.h"

#include <math.h>    /* atan2, sqrt, fabs, M_PI */
#include <float.h>   /* DBL_MAX                 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Internal helpers ───────────────────────────────────── */

static double clamp(double val, double lo, double hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ── Public API ─────────────────────────────────────────── */

void stanley_params_default(StanleyParams *params)
{
    if (!params) return;

    params->k             = 0.5;
    params->k_soft        = 1.0;
    params->max_steer_rad = 30.0 * M_PI / 180.0;   /* 30 degrees */
}

/* --------------------------------------------------------- */

double stanley_normalize_angle(double angle)
{
    /* Wrap to (-π, π] */
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/* --------------------------------------------------------- */

int stanley_nearest_index(const WayPoint *path, size_t path_len,
                          double qx, double qy)
{
    if (!path || path_len == 0) return -1;

    int    best_idx  = 0;
    double best_dist = DBL_MAX;

    for (size_t i = 0; i < path_len; i++) {
        double dx   = path[i].x - qx;
        double dy   = path[i].y - qy;
        double dist = dx * dx + dy * dy;   /* squared distance – no sqrt needed */
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = (int)i;
        }
    }

    return best_idx;
}

/* --------------------------------------------------------- */

double stanley_cross_track_error(double path_x,  double path_y,
                                 double path_yaw,
                                 double veh_x,   double veh_y)
{
    /*
     * Vector from the nearest path point to the front axle:
     *   d = [veh_x - path_x,  veh_y - path_y]
     *
     * Path tangent unit vector (pointing forward along path):
     *   t = [cos(path_yaw),  sin(path_yaw)]
     *
     * Path normal unit vector (pointing left of the path):
     *   n = [-sin(path_yaw),  cos(path_yaw)]
     *
     * Signed cross-track error = d · n
     *   e > 0  →  vehicle is LEFT  of the path
     *   e < 0  →  vehicle is RIGHT of the path
     */
    double dx = path_x - veh_x;
    double dy = path_y - veh_y;

    double nx = -sin(path_yaw);   /* normal x */
    double ny =  cos(path_yaw);   /* normal y */

    return dx * nx + dy * ny;
}

/* --------------------------------------------------------- */

StanleyStatus stanley_compute(const VehicleState  *state,
                              const WayPoint      *path,
                              size_t               path_len,
                              const StanleyParams *params,
                              StanleyResult       *result)
{
    /* ── Validate inputs ───────────────────────────────── */
    if (!state || !path || !params || !result)
        return STANLEY_ERR_NULL_PTR;

    if (path_len == 0)
        return STANLEY_ERR_NO_PATH;

    /* ── Step 1: find nearest waypoint ────────────────── */
    int idx = stanley_nearest_index(path, path_len, state->x, state->y);
    result->nearest_idx = idx;

    /* ── Step 2: heading error  ψe ────────────────────── */
    /*
     * ψe = path_yaw – vehicle_yaw
     * Positive ψe means the path is turning left relative
     * to the vehicle's current heading → steer left.
     */
    double psi_e = stanley_normalize_angle(path[idx].yaw - state->yaw);
    result->heading_error = psi_e;

    /* ── Step 3: signed cross-track error  e ──────────── */
    double e = stanley_cross_track_error(path[idx].x,  path[idx].y,
                                         path[idx].yaw,
                                         state->x,     state->y);
    result->cross_track_error = e;

    /* ── Step 4: Stanley steering law ─────────────────── */
    /*
     *  δ = ψe + arctan( k · e / (v + k_soft) )
     *
     *  The arctan term handles the cross-track correction:
     *    - large |e| → large correction
     *    - large v   → gentle correction (stability)
     *    - k_soft    → finite correction at v ≈ 0
     */
    double speed          = fabs(state->speed);   /* use magnitude */
    double cte_correction = atan2(params->k * e,
                                  speed + params->k_soft);

    double steer = psi_e + cte_correction;

    /* ── Step 5: Saturate ──────────────────────────────── */
    steer = clamp(steer,
                  -params->max_steer_rad,
                   params->max_steer_rad);

    result->steer_rad = steer;

    return STANLEY_OK;
}

/* =========================================================
 * Track-mapping layer
 * ========================================================= */

/*
 * Straight-line threshold [rad].
 * Below this |δ|, tan(δ) ≈ δ is tiny enough that R would overflow
 * double precision; treat as pure straight-line motion instead.
 */
#define STRAIGHT_THRESH (1e-4)

void track_params_default(TrackParams *tp)
{
    if (!tp) return;
    tp->wheelbase       = 0.50;   /* 50 cm                        */
    tp->track_width     = 0.40;   /* 40 cm                        */
    tp->max_track_speed = 2.00;   /* 2 m/s per track              */
    tp->min_radius      = 0.20;   /* B/2 – pivot turns allowed    */
}

/* --------------------------------------------------------- */

TrackStatus stanley_to_track(double steer_rad, double v_fwd,
                              const TrackParams *tp,
                              TrackCommand      *cmd)
{
    /* ── Validate ──────────────────────────────────────── */
    if (!tp || !cmd)
        return TRACK_ERR_NULL_PTR;

    if (tp->track_width <= 0.0 || tp->wheelbase <= 0.0 ||
        tp->max_track_speed <= 0.0)
        return TRACK_ERR_BAD_PARAMS;

    /*
     * Enforce min_radius >= B/2.
     * B/2 is the geometric minimum for a tracked vehicle: at R = B/2 the
     * inner track has zero speed (pure skid-steer pivot about inner track).
     * Values smaller than B/2 would require the inner track to reverse,
     * which may be undesirable or mechanically forbidden.
     */
    double half_B = tp->track_width / 2.0;
    double r_min  = (tp->min_radius >= half_B) ? tp->min_radius : half_B;

    cmd->radius_clamped = 0;
    cmd->speed_scaled   = 0;

    /* ── Case 1: straight line ──────────────────────────── */
    if (fabs(steer_rad) < STRAIGHT_THRESH) {
        cmd->v_left        = v_fwd;
        cmd->v_right       = v_fwd;
        cmd->turning_radius = DBL_MAX;   /* conceptually infinite */
        return TRACK_OK;
    }

    /* ── Case 2: turning ────────────────────────────────── */

    /*
     * Nominal turning radius from the bicycle kinematic model:
     *   R_nom = L / tan(δ)
     *
     * sign(R_nom) == sign(δ):
     *   δ > 0  →  turning left   →  R > 0 (ICR to the left)
     *   δ < 0  →  turning right  →  R < 0 (ICR to the right)
     */
    double tan_delta = tan(steer_rad);
    double R_nom     = tp->wheelbase / tan_delta;   /* signed */
    double R_abs     = fabs(R_nom);
    int    turn_left = (R_nom > 0.0);               /* 1 = left, 0 = right */

    /* ── Step 3: clamp to min_radius ────────────────────── */
    if (R_abs < r_min) {
        R_abs               = r_min;
        cmd->radius_clamped = 1;
    }
    cmd->turning_radius = R_abs;

    /*
     * Differential-drive kinematic model:
     *
     *   v_outer = v_fwd * (R + B/2) / R
     *   v_inner = v_fwd * (R - B/2) / R
     *
     * When turning LEFT:  left  track = inner,  right track = outer.
     * When turning RIGHT: right track = inner,  left  track = outer.
     *
     * v_fwd may be negative (reverse).  The ratio (R ± B/2)/R is applied
     * to the signed v_fwd, so reverse turning works correctly.
     */
    double ratio_outer = (R_abs + half_B) / R_abs;
    double ratio_inner = (R_abs - half_B) / R_abs;   /* >= 0 when R >= B/2 */

    double v_outer = v_fwd * ratio_outer;
    double v_inner = v_fwd * ratio_inner;

    if (turn_left) {
        cmd->v_right = v_outer;
        cmd->v_left  = v_inner;
    } else {
        cmd->v_left  = v_outer;
        cmd->v_right = v_inner;
    }

    /* ── Step 5: proportional speed saturation ──────────── */
    double max_abs = fabs(cmd->v_left);
    if (fabs(cmd->v_right) > max_abs)
        max_abs = fabs(cmd->v_right);

    if (max_abs > tp->max_track_speed && max_abs > 0.0) {
        double scale    = tp->max_track_speed / max_abs;
        cmd->v_left    *= scale;
        cmd->v_right   *= scale;
        cmd->speed_scaled = 1;
    }

    return TRACK_OK;
}
