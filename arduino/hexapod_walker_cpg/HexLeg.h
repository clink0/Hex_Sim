/*
 * HexLeg.h — Single Hexapod Leg Controller
 * ==========================================
 * Manages three servos (coxa/femur/tibia) for one leg.
 * Uses Arduino's Servo library for direct PWM control.
 *
 * Features:
 *   - Inverse kinematics via Kinematics.h
 *   - Servo angle rate limiting (prevents violent jerks)
 *   - Pulse width calculation with direction reversal
 *   - State tracking for debugging
 *
 * Servo angle convention:
 *   angle ∈ [-90°, +90°] → pulse ∈ [PULSE_MIN, PULSE_MAX]
 *   0° = center position
 *   Direction reversal handled per-servo for left/right mounting.
 *
 * References:
 *   - github.com/JakobLeander/hexapod (servo setup for this kit)
 *   - github.com/petercorke/robotics-toolbox-python (IK methodology)
 */

#ifndef HEXLEG_H
#define HEXLEG_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "Kinematics.h"

class HexLeg {
public:
    // ─── Public State (read-only externally) ───

    const char* name;         // Leg identifier ("LF", "LM", etc.)
    float mountX, mountY;     // Coxa mount on body (mm from center)
    Vec3 footPos;             // Current foot position (body frame, mm)
    JointAngles currentAngles; // Current servo angles (degrees)
    bool attached;            // true if servos are active

    // ─── Initialization ───

    /*
     * init — Set up one leg with its hardware connections.
     *
     * Must be called once in setup() for each leg.
     * Immediately centers all three servos.
     *
     * Parameters:
     *   legName — Human-readable identifier (e.g., "LF")
     *   mx, my  — Mount position on body (mm from center)
     *   pinHip, pinKnee, pinFoot — Arduino pin numbers
     *   revHip, revKnee, revFoot — Direction reversal flags
     */
    void init(const char* legName,
              float mx, float my,
              uint8_t pinHip, uint8_t pinKnee, uint8_t pinFoot,
              bool revHip, bool revKnee, bool revFoot)
    {
        name   = legName;
        mountX = mx;
        mountY = my;

        m_pinHip  = pinHip;
        m_pinKnee = pinKnee;
        m_pinFoot = pinFoot;

        m_revHip  = revHip;
        m_revKnee = revKnee;
        m_revFoot = revFoot;

        // Attach servos with calibrated pulse range
        m_servoHip.attach(pinHip, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        m_servoKnee.attach(pinKnee, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        m_servoFoot.attach(pinFoot, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        attached = true;

        // Initialize to center
        currentAngles = {0.0f, 0.0f, 0.0f, true};
        m_targetAngles = currentAngles;
        writeServos(0, 0, 0);

        // Set a reasonable default foot position
        // (will be overwritten when first pose is applied)
        footPos = {mx + 100.0f, my, DEFAULT_STAND_Z};
    }

    // ─── Position Control ───

    /*
     * setFootPosition — Move foot to target position using IK.
     *
     * Computes inverse kinematics for the target position,
     * validates reachability, and writes to servos.
     *
     * Returns true if successful, false if position is unreachable.
     */
    bool setFootPosition(Vec3 target)
    {
        JointAngles angles = computeLegIK(target, mountX, mountY);

        if (!angles.valid) {
            // IK failed — don't move, keep current position
            return false;
        }

        // Apply rate limiting to prevent sudden jumps
        angles.coxa  = rateLimitAngle(currentAngles.coxa,  angles.coxa);
        angles.femur = rateLimitAngle(currentAngles.femur, angles.femur);
        angles.tibia = rateLimitAngle(currentAngles.tibia, angles.tibia);

        // Update state
        footPos = target;
        currentAngles = angles;

        // Command servos
        writeServos(angles.coxa, angles.femur, angles.tibia);
        return true;
    }

    /*
     * setFootPositionDirect — Move foot without rate limiting.
     *
     * Use for interpolated motion where the caller handles smoothness.
     * Faster than setFootPosition since it skips rate limiting.
     */
    bool setFootPositionDirect(Vec3 target)
    {
        JointAngles angles = computeLegIK(target, mountX, mountY);
        if (!angles.valid) return false;

        footPos = target;
        currentAngles = angles;
        writeServos(angles.coxa, angles.femur, angles.tibia);
        return true;
    }

    // ─── Direct Angle Control ───

    /*
     * setAngles — Set servo angles directly (bypasses IK).
     * Useful for calibration, testing, and wiggle functions.
     */
    void setAngles(float coxa, float femur, float tibia)
    {
        coxa  = constrain(coxa,  SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        femur = constrain(femur, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        tibia = constrain(tibia, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

        currentAngles.coxa  = coxa;
        currentAngles.femur = femur;
        currentAngles.tibia = tibia;
        currentAngles.valid = true;
        writeServos(coxa, femur, tibia);
    }

    /*
     * center — Move all joints to 0° (center pulse).
     * Essential first step in calibration.
     */
    void center()
    {
        setAngles(0, 0, 0);
    }

    // ─── Servo Power Management ───

    /*
     * detach — Release servo PWM signals (servos go limp).
     * Use in sleep mode to save power and reduce noise.
     */
    void detach()
    {
        m_servoHip.detach();
        m_servoKnee.detach();
        m_servoFoot.detach();
        attached = false;
    }

    /*
     * reattach — Re-enable servo PWM after detach.
     */
    void reattach()
    {
        m_servoHip.attach(m_pinHip, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        m_servoKnee.attach(m_pinKnee, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        m_servoFoot.attach(m_pinFoot, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        attached = true;
        // Re-write current angles to servos
        writeServos(currentAngles.coxa, currentAngles.femur, currentAngles.tibia);
    }

    // ─── Diagnostics ───

    /*
     * printState — Output current state to Serial.
     */
    void printState()
    {
        Serial.print(name);
        Serial.print(F(" foot=("));
        Serial.print(footPos.x, 1); Serial.print(F(","));
        Serial.print(footPos.y, 1); Serial.print(F(","));
        Serial.print(footPos.z, 1); Serial.print(F(") deg=("));
        Serial.print(currentAngles.coxa, 1);  Serial.print(F(","));
        Serial.print(currentAngles.femur, 1);  Serial.print(F(","));
        Serial.print(currentAngles.tibia, 1);  Serial.print(F(") µs=("));
        Serial.print(angleToPulse(currentAngles.coxa,  m_revHip));  Serial.print(F(","));
        Serial.print(angleToPulse(currentAngles.femur, m_revKnee)); Serial.print(F(","));
        Serial.print(angleToPulse(currentAngles.tibia, m_revFoot)); Serial.println(F(")"));
    }

    /*
     * runIKValidation — Test IK round-trip accuracy for this leg.
     * Tests 5 positions and prints error for each.
     */
    void runIKValidation()
    {
        Serial.print(F("=== IK Validation for "));
        Serial.print(name);
        Serial.println(F(" ==="));

        Vec3 tests[] = {
            {mountX + 100.0f, mountY, DEFAULT_STAND_Z},
            {mountX + 120.0f, mountY, DEFAULT_STAND_Z},
            {mountX +  80.0f, mountY, DEFAULT_STAND_Z + 30},
            {mountX + 100.0f, mountY + 20, DEFAULT_STAND_Z},
            {mountX + 100.0f, mountY, DEFAULT_STAND_Z - 20},
        };
        const char* labels[] = {"Home", "Reach", "Lifted", "Sideways", "Lower"};

        for (int i = 0; i < 5; i++) {
            printIKValidation(tests[i], mountX, mountY, labels[i]);
        }
    }

private:
    Servo m_servoHip, m_servoKnee, m_servoFoot;
    uint8_t m_pinHip, m_pinKnee, m_pinFoot;
    bool m_revHip, m_revKnee, m_revFoot;
    JointAngles m_targetAngles;

    /*
     * rateLimitAngle — Clamp angle change to MAX_ANGLE_CHANGE_PER_STEP.
     *
     * Prevents violent servo jerks that could damage the robot or
     * strip servo gears if IK produces a large angle jump.
     */
    float rateLimitAngle(float current, float target)
    {
        float delta = target - current;
        if (delta > MAX_ANGLE_CHANGE_PER_STEP)
            return current + MAX_ANGLE_CHANGE_PER_STEP;
        if (delta < -MAX_ANGLE_CHANGE_PER_STEP)
            return current - MAX_ANGLE_CHANGE_PER_STEP;
        return target;
    }

    /*
     * angleToPulse — Convert angle to servo pulse width.
     *
     * Math (linear interpolation):
     *   angle ∈ [-90°, +90°] → pulse ∈ [PULSE_MIN, PULSE_MAX]
     *   pulse = PULSE_MIN + (angle + 90) / 180 × (PULSE_MAX - PULSE_MIN)
     *
     * If reversed: negate angle before mapping (mirrors the servo direction).
     */
    uint16_t angleToPulse(float angleDeg, bool reversed)
    {
        angleDeg = constrain(angleDeg, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        if (reversed) angleDeg = -angleDeg;

        float pulse = SERVO_PULSE_MIN +
            (angleDeg - SERVO_ANGLE_MIN) *
            (float)(SERVO_PULSE_MAX - SERVO_PULSE_MIN) /
            (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN);

        return (uint16_t)constrain(pulse, (float)SERVO_PULSE_MIN, (float)SERVO_PULSE_MAX);
    }

    /*
     * writeServos — Send angles to all three physical servos.
     */
    void writeServos(float coxa, float femur, float tibia)
    {
        if (!attached) return;
        m_servoHip.writeMicroseconds(angleToPulse(coxa,  m_revHip));
        m_servoKnee.writeMicroseconds(angleToPulse(femur, m_revKnee));
        m_servoFoot.writeMicroseconds(angleToPulse(tibia, m_revFoot));
    }
};

#endif // HEXLEG_H
