/*
 * GaitEngine.h — Hexapod Gait Generator
 * ========================================
 * Generates foot trajectories for walking, turning, and transitioning.
 *
 * Gait: Tripod (alternating triangle groups)
 *   Group A: LFront, RMiddle, LBack  (move together)
 *   Group B: RFront, LMiddle, RBack  (move together)
 *
 *   Walk cycle (4 phases):
 *     Phase 0: Lift Group A legs (swing preparation)
 *     Phase 1: Swing A forward + push B backward (body advances)
 *     Phase 2: Lift Group B legs (swing preparation)
 *     Phase 3: Swing B forward + push A backward (body advances)
 *
 * Interpolation uses cubic easing for smooth acceleration/deceleration,
 * following the approach from JakobLeander/hexapod.
 *
 * Coordinate system:
 *   X: forward, Y: left, Z: up (negative = toward ground)
 *
 * References:
 *   - github.com/JakobLeander/hexapod — tripod gait implementation
 *   - github.com/petercorke/robotics-toolbox-python — trajectory planning
 */

#ifndef GAIT_ENGINE_H
#define GAIT_ENGINE_H

#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "Kinematics.h"

// ────── Data Structures ──────

// Full pose for all 6 legs (foot positions in body frame)
struct HexPose {
    Vec3 lf, lm, lb;  // Left front, middle, back
    Vec3 rf, rm, rb;  // Right front, middle, back
};

// ────── Interpolation Functions ──────

/*
 * Linear interpolation between two Vec3 points.
 *   t ∈ [0, 1]: 0 = start, 1 = end
 */
inline Vec3 lerpVec3(Vec3 a, Vec3 b, float t) {
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

/*
 * Smooth-step easing (cubic Hermite):
 *   t ∈ [0, 1] → smoothed t ∈ [0, 1]
 *   Provides smooth acceleration at start and deceleration at end.
 *   f(t) = 3t² - 2t³
 */
inline float smoothStep(float t) {
    t = constrain(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/*
 * Interpolate between two full poses with optional smoothing.
 *   t ∈ [0, 1], smooth = true applies cubic easing
 */
inline HexPose lerpPose(HexPose from, HexPose to, float t, bool smooth = false) {
    if (smooth) t = smoothStep(t);
    return {
        lerpVec3(from.lf, to.lf, t),
        lerpVec3(from.lm, to.lm, t),
        lerpVec3(from.lb, to.lb, t),
        lerpVec3(from.rf, to.rf, t),
        lerpVec3(from.rm, to.rm, t),
        lerpVec3(from.rb, to.rb, t)
    };
}

// ────── Gait Engine ──────

class GaitEngine {
public:

    // ─── Static Poses ───

    /*
     * homePose — All feet on ground at default standing positions.
     *
     * This is the neutral standing pose. Foot positions are defined
     * by FOOT_FB_DIST, FOOT_WIDTH_FB, FOOT_WIDTH_M in Config.h.
     *
     * Feet are placed symmetrically:
     *   Front/Back: at ±FOOT_FB_DIST along X, ±FOOT_WIDTH_FB along Y
     *   Middle: at X=0, ±FOOT_WIDTH_M along Y
     */
    HexPose homePose(float zHeight = DEFAULT_STAND_Z) {
        return {
            .lf = { FOOT_FB_DIST,     FOOT_WIDTH_FB,  zHeight},
            .lm = { 0,                FOOT_WIDTH_M,   zHeight},
            .lb = {-FOOT_FB_DIST,     FOOT_WIDTH_FB,  zHeight},
            .rf = { FOOT_FB_DIST,    -FOOT_WIDTH_FB,  zHeight},
            .rm = { 0,               -FOOT_WIDTH_M,   zHeight},
            .rb = {-FOOT_FB_DIST,    -FOOT_WIDTH_FB,  zHeight}
        };
    }

    /*
     * sleepPose — Legs relaxed, not supporting weight.
     * Feet at body height (z=0). Robot should be supported externally.
     */
    HexPose sleepPose() {
        return {
            .lf = { FOOT_FB_DIST,     FOOT_WIDTH_FB,  0},
            .lm = { 0,                FOOT_WIDTH_M,   0},
            .lb = {-FOOT_FB_DIST,     FOOT_WIDTH_FB,  0},
            .rf = { FOOT_FB_DIST,    -FOOT_WIDTH_FB,  0},
            .rm = { 0,               -FOOT_WIDTH_M,   0},
            .rb = {-FOOT_FB_DIST,    -FOOT_WIDTH_FB,  0}
        };
    }

    // ─── Walk Gait ───

    /*
     * walkKeyframe — Get keyframe pose for forward/backward walking.
     *
     * Tripod gait: Groups A and B alternate lift-swing-push cycles.
     *   Group A: LF, RM, LB
     *   Group B: RF, LM, RB
     *
     * Phase sequence:
     *   0: Group A lifts (feet rise to swing height)
     *   1: Group A swings forward, Group B pushes backward
     *   2: Group B lifts
     *   3: Group B swings forward, Group A pushes backward
     *
     * Parameters:
     *   phase: 0-3 (cycle wraps around)
     *   zHeight: standing height (negative, mm below body)
     *   direction: +1.0 = forward, -1.0 = backward
     */
    HexPose walkKeyframe(uint8_t phase, float zHeight = DEFAULT_STAND_Z, float direction = 1.0f) {
        const float zUp = zHeight + STEP_HEIGHT;  // Lifted foot height
        const float sx = STEP_LENGTH * direction;  // Step distance (signed)

        switch (phase) {
        case 0:
            // ── Group A lifts ──
            // A legs (LF, RM, LB) rise to swing height
            // B legs (RF, LM, RB) stay on ground
            return {
                .lf = { FOOT_FB_DIST,     FOOT_WIDTH_FB,  zUp},       // A: lift
                .lm = { 0,                FOOT_WIDTH_M,   zHeight},    // B: ground
                .lb = {-FOOT_FB_DIST,     FOOT_WIDTH_FB,  zUp},       // A: lift
                .rf = { FOOT_FB_DIST,    -FOOT_WIDTH_FB,  zHeight},    // B: ground
                .rm = { 0,               -FOOT_WIDTH_M,   zUp},       // A: lift
                .rb = {-FOOT_FB_DIST,    -FOOT_WIDTH_FB,  zHeight}     // B: ground
            };

        case 1:
            // ── A swung forward, B pushed backward ──
            // A lands at forward position, B has pushed body forward
            return {
                .lf = { FOOT_FB_DIST + sx,   FOOT_WIDTH_FB,  zHeight},   // A: forward
                .lm = { 0 - sx,              FOOT_WIDTH_M,   zHeight},   // B: backward
                .lb = {-FOOT_FB_DIST + sx,   FOOT_WIDTH_FB,  zHeight},   // A: forward
                .rf = { FOOT_FB_DIST - sx,  -FOOT_WIDTH_FB,  zHeight},   // B: backward
                .rm = { 0 + sx,             -FOOT_WIDTH_M,   zHeight},   // A: forward
                .rb = {-FOOT_FB_DIST - sx,  -FOOT_WIDTH_FB,  zHeight}    // B: backward
            };

        case 2:
            // ── Group B lifts ──
            return {
                .lf = { FOOT_FB_DIST,     FOOT_WIDTH_FB,  zHeight},    // A: ground
                .lm = { 0,                FOOT_WIDTH_M,   zUp},        // B: lift
                .lb = {-FOOT_FB_DIST,     FOOT_WIDTH_FB,  zHeight},    // A: ground
                .rf = { FOOT_FB_DIST,    -FOOT_WIDTH_FB,  zUp},        // B: lift
                .rm = { 0,               -FOOT_WIDTH_M,   zHeight},    // A: ground
                .rb = {-FOOT_FB_DIST,    -FOOT_WIDTH_FB,  zUp}         // B: lift
            };

        case 3:
            // ── B swung forward, A pushed backward ──
            return {
                .lf = { FOOT_FB_DIST - sx,   FOOT_WIDTH_FB,  zHeight},   // A: backward
                .lm = { 0 + sx,              FOOT_WIDTH_M,   zHeight},   // B: forward
                .lb = {-FOOT_FB_DIST - sx,   FOOT_WIDTH_FB,  zHeight},   // A: backward
                .rf = { FOOT_FB_DIST + sx,  -FOOT_WIDTH_FB,  zHeight},   // B: forward
                .rm = { 0 - sx,             -FOOT_WIDTH_M,   zHeight},   // A: backward
                .rb = {-FOOT_FB_DIST + sx,  -FOOT_WIDTH_FB,  zHeight}    // B: forward
            };

        default:
            return homePose(zHeight);
        }
    }

    // ─── Rotation Gait ───

    /*
     * rotateKeyframe — Keyframe for turning in place.
     *
     * Rotation works by moving feet along tangent arcs around the body
     * center. Each foot shifts perpendicular to its radius from center,
     * creating a pure rotational torque.
     *
     * Mathematically, for a small rotation angle θ, foot at (x,y) moves to:
     *   x' = x·cos(θ) - y·sin(θ)
     *   y' = x·sin(θ) + y·cos(θ)
     *
     * For small angles (used here): cos(θ)≈1, sin(θ)≈θ
     *   Δx ≈ -y·θ,  Δy ≈ x·θ
     *
     * Parameters:
     *   phase: 0-3 cycle
     *   direction: +1.0 = rotate left (CCW from above), -1.0 = right
     */
    HexPose rotateKeyframe(uint8_t phase, float zHeight = DEFAULT_STAND_Z, float direction = 1.0f) {
        const float zUp = zHeight + STEP_HEIGHT;

        // Rotation angle (radians) per half-step
        // Tuned for stable turning: ~5° per half-step
        const float theta = 0.09f * direction;  // ~5.2° per half-step

        // Home positions for each foot (these are the "rest" positions)
        const float lfx =  FOOT_FB_DIST, lfy =  FOOT_WIDTH_FB;
        const float lmx =  0,            lmy =  FOOT_WIDTH_M;
        const float lbx = -FOOT_FB_DIST, lby =  FOOT_WIDTH_FB;
        const float rfx =  FOOT_FB_DIST, rfy = -FOOT_WIDTH_FB;
        const float rmx =  0,            rmy = -FOOT_WIDTH_M;
        const float rbx = -FOOT_FB_DIST, rby = -FOOT_WIDTH_FB;

        // Compute rotated positions (exact 2D rotation, not small-angle approx)
        const float ct = cosf(theta);
        const float st = sinf(theta);

        // Rotated positions (+ direction = CW displacement for Group A)
        // Group A rotated forward, Group B rotated backward
        // "Forward rotation" for Group A:
        float lfx_r = lfx * ct - lfy * st;  float lfy_r = lfx * st + lfy * ct;
        float rmx_r = rmx * ct - rmy * st;  float rmy_r = rmx * st + rmy * ct;
        float lbx_r = lbx * ct - lby * st;  float lby_r = lbx * st + lby * ct;

        // Group B rotated (opposite direction)
        const float ct_b = cosf(-theta);
        const float st_b = sinf(-theta);
        float rfx_r = rfx * ct_b - rfy * st_b;  float rfy_r = rfx * st_b + rfy * ct_b;
        float lmx_r = lmx * ct_b - lmy * st_b;  float lmy_r = lmx * st_b + lmy * ct_b;
        float rbx_r = rbx * ct_b - rby * st_b;  float rby_r = rbx * st_b + rby * ct_b;

        switch (phase) {
        case 0:
            // Group A lifts
            return {
                .lf = {lfx,  lfy,  zUp},
                .lm = {lmx,  lmy,  zHeight},
                .lb = {lbx,  lby,  zUp},
                .rf = {rfx,  rfy,  zHeight},
                .rm = {rmx,  rmy,  zUp},
                .rb = {rbx,  rby,  zHeight}
            };

        case 1:
            // A swung to rotated position (landed), B pushed opposite
            return {
                .lf = {lfx_r, lfy_r, zHeight},  // A: rotated
                .lm = {lmx_r, lmy_r, zHeight},  // B: counter-rotated
                .lb = {lbx_r, lby_r, zHeight},  // A: rotated
                .rf = {rfx_r, rfy_r, zHeight},  // B: counter-rotated
                .rm = {rmx_r, rmy_r, zHeight},  // A: rotated
                .rb = {rbx_r, rby_r, zHeight}   // B: counter-rotated
            };

        case 2:
            // Group B lifts
            return {
                .lf = {lfx,  lfy,  zHeight},
                .lm = {lmx,  lmy,  zUp},
                .lb = {lbx,  lby,  zHeight},
                .rf = {rfx,  rfy,  zUp},
                .rm = {rmx,  rmy,  zHeight},
                .rb = {rbx,  rby,  zUp}
            };

        case 3:
            // B swung to rotated position, A pushed opposite
            // (mirror of phase 1: A counter-rotated, B rotated)
            {
                float lfx_b = lfx * ct_b - lfy * st_b;  float lfy_b = lfx * st_b + lfy * ct_b;
                float rmx_b = rmx * ct_b - rmy * st_b;  float rmy_b = rmx * st_b + rmy * ct_b;
                float lbx_b = lbx * ct_b - lby * st_b;  float lby_b = lbx * st_b + lby * ct_b;
                float rfx_f = rfx * ct - rfy * st;       float rfy_f = rfx * st + rfy * ct;
                float lmx_f = lmx * ct - lmy * st;       float lmy_f = lmx * st + lmy * ct;
                float rbx_f = rbx * ct - rby * st;       float rby_f = rbx * st + rby * ct;

                return {
                    .lf = {lfx_b, lfy_b, zHeight},  // A: counter-rotated
                    .lm = {lmx_f, lmy_f, zHeight},  // B: rotated
                    .lb = {lbx_b, lby_b, zHeight},  // A: counter-rotated
                    .rf = {rfx_f, rfy_f, zHeight},  // B: rotated
                    .rm = {rmx_b, rmy_b, zHeight},  // A: counter-rotated
                    .rb = {rbx_f, rby_f, zHeight}   // B: rotated
                };
            }

        default:
            return homePose(zHeight);
        }
    }

    // ─── Phase Management ───

    uint8_t nextPhase() {
        m_phase = (m_phase + 1) % 4;
        return m_phase;
    }

    uint8_t getPhase() const { return m_phase; }
    void resetPhase() { m_phase = 0; }

private:
    uint8_t m_phase = 0;
};

#endif // GAIT_ENGINE_H
