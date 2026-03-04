/*
 * CPGEngine.h — Central Pattern Generator for Hexapod Locomotion
 * ===============================================================
 * Implements 6 Kuramoto coupled phase oscillators, one per leg.
 * Gait patterns emerge automatically from the coupling dynamics —
 * no hard-coded keyframes or phases.
 *
 * The governing equation for each leg i:
 *
 *   dφᵢ/dt = ω + Σⱼ Kᵢⱼ · sin(φⱼ − φᵢ − θᵢⱼ)
 *
 *   φᵢ   = phase of leg i ∈ [0, 2π)
 *   ω    = natural frequency (controls speed AND gait type)
 *   Kᵢⱼ  = coupling strength between legs i and j
 *   θᵢⱼ  = target phase offset (encodes the desired gait)
 *
 * Gait emergence vs. ω:
 *   ω ≈ 1.0 rad/s  →  Wave gait   (one leg swings at a time)
 *   ω ≈ 3.0 rad/s  →  Ripple gait (two legs swing, staggered)
 *   ω ≈ 5.0 rad/s  →  Tripod gait (three legs swing at once)
 *
 * The θᵢⱼ matrix is interpolated between wave and tripod targets
 * as ω increases, so the gait transitions smoothly and automatically.
 * You change one number (ω); the rest falls out of the math.
 *
 * Coupling topology (which legs talk to each other):
 *   Ipsilateral  (same side, adjacent):   LF-LM, LM-LB, RF-RM, RM-RB
 *   Contralateral (across body, matching): LF-RF, LM-RM, LB-RB
 *   Diagonal     (cross-body, offset):     LF-RM, LB-RM, RF-LM, RB-LM
 *
 * Leg indexing (matches HexLeg array in hexapod_walker.ino):
 *   0=LF  1=LM  2=LB  3=RF  4=RM  5=RB
 *
 * Timing performance on Arduino Mega (16MHz, no FPU):
 *   step() call:       ~2.5 ms  (11 sin() calls, Euler integration)
 *   6× IK solve:       ~3.0 ms  (in hexapod_walker.ino, not here)
 *   Total per frame:   ~6.0 ms  → safe at 40Hz (25ms budget)
 *
 * References:
 *   - Kuramoto (1984): "Chemical Oscillations, Waves, and Turbulence"
 *   - Ijspeert (2008): "Central pattern generators for locomotion control
 *                       in animals and robots", Neural Networks 21(4)
 *   - Our Python simulation (hexapod_cpg.py) — identical math, validated
 */

#ifndef CPG_ENGINE_H
#define CPG_ENGINE_H

#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "Kinematics.h"

// ── Leg indexing ──────────────────────────────────────────────
// Must match the legs[] array in hexapod_walker.ino
#define LEG_LF  0
#define LEG_LM  1
#define LEG_LB  2
#define LEG_RF  3
#define LEG_RM  4
#define LEG_RB  5
#define CPG_N   6

// ── Gait parameters ──────────────────────────────────────────
// Duty factor: fraction of cycle each foot is on the ground.
// Interpolated between these bounds as ω increases.
// High ω → lower duty (faster swing, less time on ground).
const float CPG_DUTY_SLOW  = 0.70f;   // Wave regime  (β≈0.70, very stable)
const float CPG_DUTY_FAST  = 0.50f;   // Tripod regime (β≈0.50, fastest)

// Foot lift height during swing (mm)
const float CPG_STEP_HEIGHT = STEP_HEIGHT;

// Foot neutral/home positions (body frame, mm)
// These match the Python simulation NEUTRAL dict exactly.
const float CPG_NEUTRAL_X[CPG_N] = {
     FOOT_FB_DIST,   0.0f,  -FOOT_FB_DIST,
     FOOT_FB_DIST,   0.0f,  -FOOT_FB_DIST
};
const float CPG_NEUTRAL_Y[CPG_N] = {
    FOOT_WIDTH_FB,  FOOT_WIDTH_M,  FOOT_WIDTH_FB,
   -FOOT_WIDTH_FB, -FOOT_WIDTH_M, -FOOT_WIDTH_FB
};

// ── Coupling architecture ─────────────────────────────────────
// 11 bidirectional edges. Stored as (i,j) pairs; coupling is symmetric.
// Validated against the Python simulation.
#define CPG_NUM_EDGES 11
static const uint8_t CPG_EDGES[CPG_NUM_EDGES][2] = {
    {0,1},  // LF-LM  (ipsilateral left,  adjacent)
    {1,2},  // LM-LB  (ipsilateral left,  adjacent)
    {3,4},  // RF-RM  (ipsilateral right, adjacent)
    {4,5},  // RM-RB  (ipsilateral right, adjacent)
    {0,3},  // LF-RF  (contralateral, front)
    {1,4},  // LM-RM  (contralateral, middle)
    {2,5},  // LB-RB  (contralateral, back)
    {0,4},  // LF-RM  (diagonal)
    {2,4},  // LB-RM  (diagonal)
    {3,1},  // RF-LM  (diagonal)
    {5,1},  // RB-LM  (diagonal)
};

// ── Pre-computed phase-offset matrices ───────────────────────
// Computed once in Python and hardcoded here to avoid startup trig.
// theta[i][j] = desired phase of leg j RELATIVE to leg i.

// TRIPOD targets: Group A={LF,LB,RM}=0°, Group B={LM,RF,RB}=180°
static const float THETA_TRIPOD[CPG_N][CPG_N] = {
    {0.0000f, 3.1416f, 0.0000f, 3.1416f, 0.0000f, 3.1416f},  // LF
    {3.1416f, 0.0000f, 3.1416f, 0.0000f, 3.1416f, 0.0000f},  // LM
    {0.0000f, 3.1416f, 0.0000f, 3.1416f, 0.0000f, 3.1416f},  // LB
    {3.1416f, 0.0000f, 3.1416f, 0.0000f, 3.1416f, 0.0000f},  // RF
    {0.0000f, 3.1416f, 0.0000f, 3.1416f, 0.0000f, 3.1416f},  // RM
    {3.1416f, 0.0000f, 3.1416f, 0.0000f, 3.1416f, 0.0000f},  // RB
};

// WAVE targets: 60° spacing front-to-back on each side.
// LF=0°, LM=60°, LB=120°, RF=180°, RM=240°, RB=300°
static const float THETA_WAVE[CPG_N][CPG_N] = {
    { 0.0000f,  1.0472f,  2.0944f, -3.1416f, -2.0944f, -1.0472f},  // LF
    {-1.0472f,  0.0000f,  1.0472f,  2.0944f, -3.1416f, -2.0944f},  // LM
    {-2.0944f, -1.0472f,  0.0000f,  1.0472f,  2.0944f, -3.1416f},  // LB
    {-3.1416f, -2.0944f, -1.0472f,  0.0000f,  1.0472f,  2.0944f},  // RF
    { 2.0944f, -3.1416f, -2.0944f, -1.0472f,  0.0000f,  1.0472f},  // RM
    { 1.0472f,  2.0944f, -3.1416f, -2.0944f, -1.0472f,  0.0000f},  // RB
};

// ─────────────────────────────────────────────────────────────────
// CPGEngine class
// ─────────────────────────────────────────────────────────────────

class CPGEngine {
public:

    // ── Public configuration (tune while running) ──────────────

    float omega;          // Natural frequency (rad/s). Controls speed + gait.
                          // Range [0.5, 6.0]. Default 2.0 (ripple-ish).
    float K;              // Coupling strength. Default 3.5.
                          // Higher = faster synchronisation, stiffer gait.
    float stride;         // Half-stride length (mm). Default = STEP_LENGTH.
    float walkDirRad;     // Walk direction in body frame (0=fwd, PI=bwd)
    float turnRate;       // Tangential mm per full cycle for rotation.
                          // Positive = CCW (left), negative = CW (right).
    bool  moving;         // true = phases advancing, false = frozen

    // Current oscillator phases (read-only; modified by step())
    float phi[CPG_N];

    // ── Constructor ────────────────────────────────────────────

    CPGEngine() :
        omega(2.0f), K(3.5f), stride(STEP_LENGTH),
        walkDirRad(0.0f), turnRate(0.0f), moving(false)
    {
        // Seed phases to tripod positions directly so the robot
        // starts in a known stable gait without needing to converge.
        // Group A (LF=0, LB=2, RM=4) at phase 0
        // Group B (LM=1, RF=3, RB=5) at phase π
        phi[LEG_LF] = 0.0f;
        phi[LEG_LM] = M_PI;
        phi[LEG_LB] = 0.0f;
        phi[LEG_RF] = M_PI;
        phi[LEG_RM] = 0.0f;
        phi[LEG_RB] = M_PI;
    }

    // ── step(dt) ─────────────────────────────────────────────
    /*
     * Advance all oscillator phases by dt seconds.
     * Uses Euler integration (sufficient at 40Hz; RK2 not needed here).
     *
     * Call this once per control loop, then call getFootPosition()
     * for each leg to get the target foot coordinates.
     */
    void step(float dt)
    {
        if (!moving || dt <= 0.0f) return;

        // Interpolate coupling targets between wave and tripod
        // based on current ω. Same smoothstep as Python sim.
        float alpha = (omega - 1.0f) / 4.0f;
        alpha = constrain(alpha, 0.0f, 1.0f);
        alpha = alpha * alpha * (3.0f - 2.0f * alpha);   // smoothstep

        // Compute dφᵢ/dt for each oscillator
        float dphi[CPG_N];
        for (uint8_t i = 0; i < CPG_N; i++) {
            dphi[i] = omega;   // natural frequency term
        }

        // Add coupling terms (only iterate over defined edges, not full NxN)
        for (uint8_t e = 0; e < CPG_NUM_EDGES; e++) {
            uint8_t i = CPG_EDGES[e][0];
            uint8_t j = CPG_EDGES[e][1];

            // Interpolate target offset: theta = lerp(wave, tripod, alpha)
            float theta_ij = THETA_WAVE[i][j] + alpha * (THETA_TRIPOD[i][j] - THETA_WAVE[i][j]);
            float theta_ji = THETA_WAVE[j][i] + alpha * (THETA_TRIPOD[j][i] - THETA_WAVE[j][i]);

            // Kuramoto coupling: K * sin(φⱼ - φᵢ - θᵢⱼ)
            float coupling_ij = K * sinf(phi[j] - phi[i] - theta_ij);
            float coupling_ji = K * sinf(phi[i] - phi[j] - theta_ji);

            dphi[i] += coupling_ij;
            dphi[j] += coupling_ji;
        }

        // Euler integrate and wrap to [0, 2π)
        for (uint8_t i = 0; i < CPG_N; i++) {
            phi[i] += dphi[i] * dt;
            // Wrap: faster than fmod on AVR
            while (phi[i] >= TWO_PI) phi[i] -= TWO_PI;
            while (phi[i] <  0.0f)   phi[i] += TWO_PI;
        }
    }

    // ── getFootPosition() ─────────────────────────────────────
    /*
     * Convert a leg's current oscillator phase to a 3D foot position.
     *
     * Stance phase (φ < duty·2π):
     *   Foot on ground, moves linearly backward — pushes body forward.
     *
     * Swing phase (φ ≥ duty·2π):
     *   Foot in air, arcs forward with sine-curve lift profile.
     *
     * This is identical to the Python simulation's _feet_from_phases().
     */
    Vec3 getFootPosition(uint8_t legIdx) const
    {
        const float phiLeg = phi[legIdx];
        const float nx     = CPG_NEUTRAL_X[legIdx];
        const float ny     = CPG_NEUTRAL_Y[legIdx];

        // Duty factor: decreases with ω (less stance time at high speed)
        float alpha = (omega - 1.0f) / 4.0f;
        alpha = constrain(alpha, 0.0f, 1.0f);
        const float duty    = CPG_DUTY_SLOW + alpha * (CPG_DUTY_FAST - CPG_DUTY_SLOW);
        const float dutyRad = TWO_PI * duty;

        // Stride vector in walk direction
        float sdx = stride * cosf(walkDirRad);
        float sdy = stride * sinf(walkDirRad);

        // Rotation contribution: tangential arc around body centre
        // Each leg's foot sweeps a small arc perpendicular to its radius
        const float legR = sqrtf(nx*nx + ny*ny);
        if (legR > 1.0f) {
            sdx += (-ny / legR) * turnRate;
            sdy += ( nx / legR) * turnRate;
        }

        Vec3 foot;

        if (phiLeg < dutyRad) {
            // ── STANCE: linear push-back ──────────────────────
            // t ∈ [0,1] across stance. Foot goes from +stride → -stride.
            const float t = phiLeg / dutyRad;
            foot.x = nx + sdx * (0.5f - t);
            foot.y = ny + sdy * (0.5f - t);
            foot.z = DEFAULT_STAND_Z;

        } else {
            // ── SWING: parabolic arc ──────────────────────────
            const float swingRad = TWO_PI - dutyRad;
            const float t        = (phiLeg - dutyRad) / swingRad;  // 0→1

            // Cubic ease for smooth horizontal travel (no jerk at lift-off)
            const float te = t * t * (3.0f - 2.0f * t);
            foot.x = nx + sdx * (-0.5f + te);
            foot.y = ny + sdy * (-0.5f + te);

            // Sine arch: zero at lift-off, peak at mid-swing, zero at land
            foot.z = DEFAULT_STAND_Z + CPG_STEP_HEIGHT * sinf(M_PI * t);
        }

        return foot;
    }

    // ── Control interface ──────────────────────────────────────

    /*
     * walkForward / walkBackward — Set walk direction and start moving.
     */
    void walkForward()  { walkDirRad = 0.0f;  turnRate = 0.0f; moving = true; }
    void walkBackward() { walkDirRad = M_PI;   turnRate = 0.0f; moving = true; }

    /*
     * turnLeft / turnRight — Rotate in place.
     * Feet sweep tangential arcs around body centre.
     * direction: +1=CCW/left, -1=CW/right
     */
    void turnLeft()  { walkDirRad = 0.0f; turnRate =  55.0f; moving = true; }
    void turnRight() { walkDirRad = 0.0f; turnRate = -55.0f; moving = true; }

    /*
     * stop — Freeze phase advancement. Feet hold their last position.
     */
    void stop() { moving = false; }

    /*
     * setSpeed — Set ω and thereby both speed and gait type.
     * omega = 1.0 → wave, 3.0 → ripple, 5.0 → tripod
     */
    void setSpeed(float newOmega) {
        omega = constrain(newOmega, 0.5f, 6.0f);
    }

    /*
     * gaitName — Returns a short string describing current gait.
     * Useful for serial diagnostics.
     */
    const char* gaitName() const {
        float alpha = (omega - 1.0f) / 4.0f;
        alpha = constrain(alpha, 0.0f, 1.0f);
        alpha = alpha * alpha * (3.0f - 2.0f * alpha);
        if (alpha < 0.25f) return "Wave";
        if (alpha < 0.65f) return "Ripple";
        return "Tripod";
    }

    /*
     * phaseCoherence — Measures how well the network has synchronised.
     * Returns a value in [0, 1]. Close to 1 = locked gait, near 0 = transitioning.
     *
     * Uses pattern coherence (mean phase-difference error vs. target),
     * not global Kuramoto r, which is misleading for antiphase gaits.
     */
    float phaseCoherence() const {
        if (!moving) return 0.0f;

        // Compute interpolated theta targets at current omega
        float alpha = (omega - 1.0f) / 4.0f;
        alpha = constrain(alpha, 0.0f, 1.0f);
        alpha = alpha * alpha * (3.0f - 2.0f * alpha);

        float totalErr = 0.0f;
        uint8_t count  = 0;

        for (uint8_t e = 0; e < CPG_NUM_EDGES; e++) {
            uint8_t i = CPG_EDGES[e][0];
            uint8_t j = CPG_EDGES[e][1];
            float theta_ij = THETA_WAVE[i][j] + alpha * (THETA_TRIPOD[i][j] - THETA_WAVE[i][j]);

            float actual = phi[j] - phi[i];
            // Wrap to (-π, π]
            while (actual >  M_PI) actual -= TWO_PI;
            while (actual < -M_PI) actual += TWO_PI;

            float err = fabsf(actual - theta_ij);
            if (err > M_PI) err = TWO_PI - err;
            totalErr += err;
            count++;
        }

        if (count == 0) return 0.0f;
        return 1.0f - (totalErr / count) / M_PI;
    }

    /*
     * printStatus — Print CPG state to Serial (for diagnostics).
     */
    void printStatus() const {
        Serial.print(F("CPG: omega="));    Serial.print(omega, 2);
        Serial.print(F(" gait="));         Serial.print(gaitName());
        Serial.print(F(" r="));            Serial.print(phaseCoherence(), 2);
        Serial.print(F(" moving="));       Serial.println(moving ? F("yes") : F("no"));
        Serial.print(F("     phases(deg): "));
        const char* names[] = {"LF","LM","LB","RF","RM","RB"};
        for (uint8_t i = 0; i < CPG_N; i++) {
            Serial.print(names[i]); Serial.print(F("="));
            Serial.print(phi[i] * RAD_TO_DEG, 0);
            if (i < CPG_N-1) Serial.print(F(" "));
        }
        Serial.println();
    }
};

#endif // CPG_ENGINE_H
