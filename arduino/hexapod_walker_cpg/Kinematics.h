/*
 * Kinematics.h — 3-DOF Hexapod Leg Inverse Kinematics
 * =====================================================
 * Pure math — no hardware dependencies. Can be tested independently.
 *
 * IK approach: Geometric / Law of Cosines
 * Same method used by:
 *   - github.com/JakobLeander/hexapod (proven on this exact kit)
 *   - github.com/petercorke/robotics-toolbox-python (general DH/geometric IK)
 *
 * Coordinate System (body frame, right-hand rule):
 *   X: forward  (positive = front of robot)
 *   Y: sideways (positive = left side of robot)
 *   Z: vertical (positive = up, negative = down toward ground)
 *
 * Joint model (DH-equivalent for 3-DOF leg):
 *   ┌─ Body Frame ─┐
 *   │               │
 *   └───[Mount M]───┘
 *         │
 *    Coxa servo (rotation about Z-axis)
 *         │
 *    ═══ Coxa link (L_c) ═══
 *         │
 *    Femur servo (rotation about local Y-axis)
 *         │
 *    ═══ Femur link (L_f) ═══
 *         │
 *    Tibia servo (rotation about local Y-axis)
 *         │
 *    ═══ Tibia link (L_t) ═══
 *         │
 *      [Foot Tip]
 *
 * All angles are in DEGREES, centered at 0°:
 *   Coxa:  0° = leg points in mount's natural direction
 *   Femur: 0° = femur points horizontally outward
 *   Tibia: 0° = tibia perpendicular to femur (adjusted by TIBIA_OFFSET_DEG)
 *
 * Mathematical references:
 *   - Law of cosines: c² = a² + b² - 2ab·cos(C)
 *   - Rearranged: cos(C) = (a² + b² - c²) / (2ab)
 *   - atan2(y, x) for quadrant-correct angle computation
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <math.h>
#include "Config.h"

// ────── Data Structures ──────

struct JointAngles {
    float coxa;    // Hip rotation (degrees, 0° = centered)
    float femur;   // Knee/thigh angle (degrees, 0° = horizontal)
    float tibia;   // Ankle/foot angle (degrees, 0° = ~perpendicular to femur)
    bool  valid;   // true if position is reachable within servo limits
};

struct Vec3 {
    float x, y, z;
};

// ────── Inverse Kinematics ──────

/*
 * computeLegIK — Compute joint angles for one leg
 *
 * Parameters:
 *   footPos    — Desired foot tip position in BODY frame (mm)
 *   mountX, mountY — Coxa mount position on body (mm from center)
 *
 * Returns:
 *   JointAngles struct with coxa, femur, tibia in degrees
 *   .valid = false if position is unreachable or out of servo range
 *
 * ────── Full Mathematical Derivation ──────
 *
 * GIVEN:
 *   Foot target  P = (fx, fy, fz) in body frame
 *   Mount point  M = (mx, my, 0) on body
 *   Segment lengths: L_c (coxa), L_f (femur), L_t (tibia)
 *
 * STEP 1: Direction from mount to foot
 *   dx = fx - mx
 *   dy = fy - my
 *   dz = fz        (mount Z assumed = 0)
 *
 * STEP 2: Coxa angle (horizontal rotation about Z)
 *   The mount has a "natural" direction: θ_mount = atan2(my, mx)
 *   The foot direction from mount: θ_foot = atan2(dy, dx)
 *   Coxa rotation: α = θ_foot - θ_mount
 *
 *   This means at α = 0°, the leg points directly away from body center
 *   through its mount point. Positive α rotates forward (CCW from above
 *   for left legs, CW for right legs — handled by reversal flags).
 *
 * STEP 3: Project into 2D leg plane
 *   The coxa rotates the entire leg in the horizontal plane.
 *   After coxa rotation, the femur and tibia operate in a vertical 2D plane.
 *
 *   Horizontal distance from mount to foot:
 *     r = √(dx² + dy²)
 *
 *   Distance from femur pivot to foot (in leg plane):
 *     h = r - L_c     (horizontal component)
 *     v = dz           (vertical component, negative = below body)
 *
 * STEP 4: Distance from femur pivot to foot tip
 *   D = √(h² + v²)
 *
 *   This is the direct line distance that the femur-tibia linkage
 *   must span. It must satisfy: |L_f - L_t| ≤ D ≤ L_f + L_t
 *
 * STEP 5: Femur angle (law of cosines + atan2)
 *   Two sub-angles sum to give femur:
 *
 *   φ_a = atan2(v, h)
 *     This is the angle of the line from femur pivot to foot,
 *     measured from horizontal. When standing (v < 0), this is negative.
 *
 *   φ_b = acos((D² + L_f² - L_t²) / (2·D·L_f))
 *     This is the angle between the femur-to-foot line and the femur
 *     segment itself (law of cosines at the femur pivot).
 *
 *   Femur angle = φ_b + φ_a
 *     Combined: femur angle from horizontal to actual femur direction.
 *     When standing with default height, this typically gives ~20-35°.
 *
 * STEP 6: Tibia angle (law of cosines)
 *   γ_internal = acos((L_f² + L_t² - D²) / (2·L_f·L_t))
 *     This is the internal angle at the tibia joint (between femur
 *     and tibia segments). Range: 0° (fully folded) to 180° (straight).
 *
 *   Servo convention:
 *     At servo 0°, tibia is approximately perpendicular to femur (γ ≈ 90°)
 *     The physical mounting adds TIBIA_OFFSET_DEG of additional offset.
 *
 *   Tibia servo angle = γ_internal - 90° - TIBIA_OFFSET_DEG
 *
 * ────── Numerical Example (home position) ──────
 *
 *   LF foot at (150, 123, -60), mount at (74, 39):
 *   dx=76, dy=84, dz=-60
 *   θ_mount = atan2(39,74) = 27.8°, θ_foot = atan2(84,76) = 47.9°
 *   α (coxa) = 20.1°
 *   r = √(76²+84²) = 113.3mm, h = 113.3-28 = 85.3mm, v = -60mm
 *   D = √(85.3²+60²) = 104.3mm ∈ [43, 211] ✓
 *   φ_a = atan2(-60, 85.3) = -35.1°
 *   φ_b = acos((104.3²+84²-127²)/(2·104.3·84)) = acos(0.327) = 70.9°
 *   Femur = 70.9° + (-35.1°) = 35.8°
 *   γ = acos((84²+127²-104.3²)/(2·84·127)) = acos(0.372) = 68.2°
 *   Tibia = 68.2° - 90° - 14° = -35.8°
 *   All within [-90, 90] ✓
 */
inline JointAngles computeLegIK(Vec3 footPos, float mountX, float mountY)
{
    JointAngles result;
    result.valid = false;

    // ─── Step 1: Vector from coxa mount to foot ───
    const float dx = footPos.x - mountX;
    const float dy = footPos.y - mountY;
    const float dz = footPos.z;  // mount Z = 0 (body center plane)

    // ─── Step 2: Coxa angle ───
    // Mount's natural pointing direction
    const float mountAngleRad = atan2(mountY, mountX);
    // Direction from mount to desired foot
    const float footAngleRad = atan2(dy, dx);
    // Relative rotation (degrees)
    result.coxa = (footAngleRad - mountAngleRad) * RAD_TO_DEG;

    // Normalize to [-180, 180]
    while (result.coxa >  180.0f) result.coxa -= 360.0f;
    while (result.coxa < -180.0f) result.coxa += 360.0f;

    // ─── Step 3: Project into leg plane ───
    const float r = sqrtf(dx * dx + dy * dy);  // horizontal distance
    const float h = r - COXA_LENGTH;            // femur-pivot → foot (horizontal)
    const float v = dz;                          // vertical (negative = below)

    // ─── Step 4: Femur-pivot to foot-tip distance ───
    const float D = sqrtf(h * h + v * v);

    // ─── Step 5: Reachability check ───
    if (D > MAX_REACH || D < MIN_REACH || D < 0.001f) {
        Serial.print(F("IK UNREACHABLE: D="));
        Serial.print(D, 1);
        Serial.print(F("mm range=["));
        Serial.print(MIN_REACH, 1);
        Serial.print(F(","));
        Serial.print(MAX_REACH, 1);
        Serial.println(F("]"));
        return result;
    }

    // ─── Step 6: Femur angle ───
    // Sub-angle: line from femur-pivot to foot, relative to horizontal
    const float phi_a = atan2(v, h);  // radians

    // Sub-angle: law of cosines at femur pivot
    //   Triangle: sides D, L_f, L_t. Angle at femur pivot between D and L_f.
    float cos_phi_b = (D * D + FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH)
                    / (2.0f * D * FEMUR_LENGTH);
    cos_phi_b = constrain(cos_phi_b, -1.0f, 1.0f);  // numerical safety
    const float phi_b = acosf(cos_phi_b);

    result.femur = (phi_b + phi_a) * RAD_TO_DEG;

    // ─── Step 7: Tibia angle ───
    //   Internal angle at tibia joint (law of cosines)
    //   Triangle: sides L_f, L_t, D. Angle between L_f and L_t.
    float cos_gamma = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - D * D)
                    / (2.0f * FEMUR_LENGTH * TIBIA_LENGTH);
    cos_gamma = constrain(cos_gamma, -1.0f, 1.0f);  // numerical safety
    const float gamma_deg = acosf(cos_gamma) * RAD_TO_DEG;

    // Convert internal angle to servo angle
    // Convention: servo 0° ≈ tibia perpendicular to femur (γ_internal = 90°)
    result.tibia = gamma_deg - 90.0f - TIBIA_OFFSET_DEG;

    // ─── Step 8: Validate all angles within servo limits ───
    if (result.coxa  < SERVO_ANGLE_MIN || result.coxa  > SERVO_ANGLE_MAX ||
        result.femur < SERVO_ANGLE_MIN || result.femur > SERVO_ANGLE_MAX ||
        result.tibia < SERVO_ANGLE_MIN || result.tibia > SERVO_ANGLE_MAX) {

        Serial.print(F("IK OUT OF RANGE: C="));
        Serial.print(result.coxa, 1);
        Serial.print(F(" F="));
        Serial.print(result.femur, 1);
        Serial.print(F(" T="));
        Serial.println(result.tibia, 1);
        return result;
    }

    // Check for NaN (can happen with degenerate geometry)
    if (isnan(result.coxa) || isnan(result.femur) || isnan(result.tibia)) {
        Serial.println(F("IK ERROR: NaN in computed angles"));
        return result;
    }

    result.valid = true;
    return result;
}

// ────── Forward Kinematics ──────

/*
 * computeFK — Compute foot position from joint angles
 *
 * Used for:
 *   1. IK validation (compute IK → FK → compare to original target)
 *   2. Debugging (what position do current angles produce?)
 *   3. Workspace analysis (sweep angles, plot reachable positions)
 *
 * Math:
 *   1. Coxa rotation defines the leg direction in the horizontal plane
 *   2. In the leg's vertical plane:
 *      - Femur endpoint = L_f at angle femurRad from horizontal
 *      - Tibia extends from femur end at absolute angle determined by
 *        the internal angle between femur and tibia
 *   3. Total horizontal = coxa_length + femur_horizontal + tibia_horizontal
 *   4. Project back to body frame using coxa direction
 */
inline Vec3 computeFK(float coxaDeg, float femurDeg, float tibiaDeg,
                       float mountX, float mountY)
{
    const float mountAngleRad = atan2(mountY, mountX);

    // Coxa absolute angle in body frame
    const float coxaAbsRad = (coxaDeg * DEG_TO_RAD) + mountAngleRad;

    // Femur angle from horizontal (in the leg's vertical plane)
    const float femurRad = femurDeg * DEG_TO_RAD;

    // Tibia: convert servo angle back to internal angle
    const float tibiaInternalDeg = tibiaDeg + 90.0f + TIBIA_OFFSET_DEG;
    const float tibiaInternalRad = tibiaInternalDeg * DEG_TO_RAD;

    // Femur endpoint in leg plane (h = horizontal, v = vertical)
    const float femurEndH = FEMUR_LENGTH * cosf(femurRad);
    const float femurEndV = FEMUR_LENGTH * sinf(femurRad);

    // Tibia absolute angle = femurAngle - (π - tibiaInternal)
    // This gives the tibia direction relative to horizontal
    const float tibiaAbsRad = femurRad - (M_PI - tibiaInternalRad);
    const float tipH = femurEndH + TIBIA_LENGTH * cosf(tibiaAbsRad);
    const float tipV = femurEndV + TIBIA_LENGTH * sinf(tibiaAbsRad);

    // Total horizontal distance from mount in leg direction
    const float totalH = COXA_LENGTH + tipH;

    // Project back to 3D body frame
    Vec3 foot;
    foot.x = mountX + totalH * cosf(coxaAbsRad);
    foot.y = mountY + totalH * sinf(coxaAbsRad);
    foot.z = tipV;

    return foot;
}

// ────── Validation Utility ──────

/*
 * validateIK — Round-trip test: IK → FK → compare to target
 *
 * Returns the Euclidean error in mm between the target and the
 * FK-reconstructed position. Should be < 0.1mm for valid positions.
 * Returns -1.0 if IK fails (unreachable).
 */
inline float validateIK(Vec3 target, float mountX, float mountY)
{
    JointAngles angles = computeLegIK(target, mountX, mountY);
    if (!angles.valid) return -1.0f;

    Vec3 reconstructed = computeFK(angles.coxa, angles.femur, angles.tibia,
                                    mountX, mountY);

    float ex = target.x - reconstructed.x;
    float ey = target.y - reconstructed.y;
    float ez = target.z - reconstructed.z;

    return sqrtf(ex * ex + ey * ey + ez * ez);
}

/*
 * printIKValidation — Detailed validation output for debugging
 */
inline void printIKValidation(Vec3 target, float mountX, float mountY, const char* label)
{
    Serial.print(F("  Validate "));
    Serial.print(label);
    Serial.print(F(": target=("));
    Serial.print(target.x, 1); Serial.print(F(","));
    Serial.print(target.y, 1); Serial.print(F(","));
    Serial.print(target.z, 1); Serial.print(F(") "));

    JointAngles angles = computeLegIK(target, mountX, mountY);
    if (!angles.valid) {
        Serial.println(F("→ UNREACHABLE"));
        return;
    }

    Vec3 fk = computeFK(angles.coxa, angles.femur, angles.tibia, mountX, mountY);

    float error = sqrtf(
        (target.x - fk.x) * (target.x - fk.x) +
        (target.y - fk.y) * (target.y - fk.y) +
        (target.z - fk.z) * (target.z - fk.z)
    );

    Serial.print(F("→ angles=("));
    Serial.print(angles.coxa, 1); Serial.print(F(","));
    Serial.print(angles.femur, 1); Serial.print(F(","));
    Serial.print(angles.tibia, 1); Serial.print(F(") FK=("));
    Serial.print(fk.x, 1); Serial.print(F(","));
    Serial.print(fk.y, 1); Serial.print(F(","));
    Serial.print(fk.z, 1); Serial.print(F(") err="));
    Serial.print(error, 2);
    Serial.println(error < 0.5f ? F("mm ✓") : F("mm ✗ CHECK DIMENSIONS!"));
}

#endif // KINEMATICS_H
