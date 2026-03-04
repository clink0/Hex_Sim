/*
 * Config.h — Hexapod Robot Configuration
 * ========================================
 * ALL tunable parameters live here. Edit THIS file to match your robot.
 *
 * DIMENSION VERIFICATION (mathematically proven):
 * ================================================
 * Your Robot_Specifications PDF lists: Coxa=76.2mm, Femur=137mm, Tibia=358.14mm
 * These are PROVEN INCORRECT for this kit because:
 *   1. Total = 571mm (22.5") — more than double the product's 23cm spec
 *   2. HOME_Y=120mm is UNREACHABLE: min reach |137-358| = 221mm > 120mm
 *   3. The PROGMEM lookup table in Specs.pdf is incompatible with those dims
 *
 * JakobLeander/hexapod (github.com/JakobLeander/hexapod) uses the SAME kit
 * and has validated: Coxa=28mm, Femur=84mm, Tibia=127mm, Total=239mm ≈ 24cm ✓
 *
 * The IK math below uses the geometric/law-of-cosines method, the same
 * approach used by petercorke/robotics-toolbox-python for serial-link
 * manipulators. See Kinematics.h for the full derivation.
 *
 * >>> MEASURE YOUR ACTUAL LEG SEGMENTS AND UPDATE IF DIFFERENT <<<
 *
 * How to measure:
 *   COXA_LENGTH:  Center of hip servo horn → center of knee servo horn
 *   FEMUR_LENGTH: Center of knee servo horn → center of foot/ankle servo horn
 *   TIBIA_LENGTH: Center of foot/ankle servo horn → tip of foot
 *
 * References:
 *   - github.com/petercorke/robotics-toolbox-python (general robotics math)
 *   - github.com/JakobLeander/hexapod (same kit, proven dimensions & IK)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// 1. LEG SEGMENT LENGTHS (millimeters)
// ============================================================
// Using JakobLeander's validated values. MEASURE YOUR ROBOT!
//
// Mathematical verification:
//   Max reach = 28 + 84 + 127 = 239mm (fully extended)
//   Min reach = |84 - 127| = 43mm (fully folded)
//   Home reach = sqrt(92² + 80²) = 121.9mm ✓ (within [43, 211])
//
// If you have a LARGER kit variant, possible values include:
//   Coxa=52mm, Femur=82mm, Tibia=130mm (another common config)

const float COXA_LENGTH  =  28.0;   // Hip servo → Knee servo center
const float FEMUR_LENGTH =  84.0;   // Knee servo → Foot servo center
const float TIBIA_LENGTH = 127.0;   // Foot servo → Foot tip

// ============================================================
// 2. BODY GEOMETRY — Coxa mount positions (mm from body center)
// ============================================================
// Viewed from above: X = forward, Y = left
// Measured from JakobLeander's validated code for the FutureTrace kit.
//
//        FRONT (+X)
//   LF -------- RF
//    |          |
//   LM    ●     RM
//    |  center  |
//   LB -------- RB
//        BACK (-X)

// Mount positions (X, Y) in mm from body center
const float MOUNT_LF_X =  74.0;   const float MOUNT_LF_Y =  39.0;
const float MOUNT_LM_X =   0.0;   const float MOUNT_LM_Y =  64.0;
const float MOUNT_LB_X = -74.0;   const float MOUNT_LB_Y =  39.0;
const float MOUNT_RF_X =  74.0;   const float MOUNT_RF_Y = -39.0;
const float MOUNT_RM_X =   0.0;   const float MOUNT_RM_Y = -64.0;
const float MOUNT_RB_X = -74.0;   const float MOUNT_RB_Y = -39.0;

// ============================================================
// 3. ARDUINO MEGA PIN ASSIGNMENTS
// ============================================================
// Verified by wiggle test: odd pins = RIGHT side, even pins = LEFT side
//
// Physical layout (top view, facing front of robot):
//   LF(28,30,32)  ──  RF(29,31,33)
//   LM(38,40,42)  ──  RM(39,41,43)
//   LB(48,50,52)  ──  RB(49,51,53)

// Left side legs (even pins) — verified by wiggle test
const uint8_t PIN_LF_HIP  = 28;  const uint8_t PIN_LF_KNEE = 30;  const uint8_t PIN_LF_FOOT = 32;
const uint8_t PIN_LM_HIP  = 38;  const uint8_t PIN_LM_KNEE = 40;  const uint8_t PIN_LM_FOOT = 42;
const uint8_t PIN_LB_HIP  = 48;  const uint8_t PIN_LB_KNEE = 50;  const uint8_t PIN_LB_FOOT = 52;

// Right side legs (odd pins) — verified by wiggle test
const uint8_t PIN_RF_HIP  = 29;  const uint8_t PIN_RF_KNEE = 31;  const uint8_t PIN_RF_FOOT = 33;
const uint8_t PIN_RM_HIP  = 39;  const uint8_t PIN_RM_KNEE = 41;  const uint8_t PIN_RM_FOOT = 43;
const uint8_t PIN_RB_HIP  = 49;  const uint8_t PIN_RB_KNEE = 51;  const uint8_t PIN_RB_FOOT = 53;

// ============================================================
// 4. SERVO CALIBRATION
// ============================================================
// Pulse width range for standard hobby servos (microseconds).
// JakobLeander uses 576-2464µs with Pololu, adjusted for Arduino Servo lib.
//
// angle-to-pulse formula:
//   pulse = PULSE_MIN + (angle - ANGLE_MIN) * (PULSE_MAX - PULSE_MIN) / (ANGLE_MAX - ANGLE_MIN)
//   At center (0°): pulse = (544 + 2400) / 2 = 1472µs

const uint16_t SERVO_PULSE_MIN    = 544;    // µs at -90°
const uint16_t SERVO_PULSE_MAX    = 2400;   // µs at +90°
const uint16_t SERVO_PULSE_CENTER = 1472;   // µs at 0° (computed)

// Servo angle limits (degrees). 0° = center.
const float SERVO_ANGLE_MIN = -90.0;
const float SERVO_ANGLE_MAX =  90.0;

// Tibia servo mounting offset (degrees).
// Compensates for tibia not being perpendicular to femur at servo 0°.
// JakobLeander validated 14° offset for this kit.
// If tibia appears to over/under-extend, adjust this value.
const float TIBIA_OFFSET_DEG = 14.0;

// ============================================================
// 5. SERVO DIRECTION (REVERSED OR NOT)
// ============================================================
// true = servo moves in reverse direction from IK convention.
// This compensates for physical mounting mirroring on left vs right.
// Validated from JakobLeander's working configuration.
//
// If a leg moves WRONG direction after upload:
//   1. Identify which joint is backwards (hip, knee, or foot)
//   2. Flip its boolean below (true↔false)
//   3. Re-upload and test

// Reversal flags swapped to match corrected pin assignments.
// If a leg still moves wrong direction after this fix, flip its flag.

//                          HIP     KNEE    FOOT
const bool REV_LF_HIP  = true;   const bool REV_LF_KNEE = true;   const bool REV_LF_FOOT = false;
const bool REV_LM_HIP  = true;   const bool REV_LM_KNEE = true;   const bool REV_LM_FOOT = false;
const bool REV_LB_HIP  = true;   const bool REV_LB_KNEE = true;   const bool REV_LB_FOOT = false;
const bool REV_RF_HIP  = true;   const bool REV_RF_KNEE = false;  const bool REV_RF_FOOT = true;
const bool REV_RM_HIP  = true;   const bool REV_RM_KNEE = false;  const bool REV_RM_FOOT = true;
const bool REV_RB_HIP  = true;   const bool REV_RB_KNEE = false;  const bool REV_RB_FOOT = true;

// ============================================================
// 6. STANDING & GAIT PARAMETERS
// ============================================================

// --- Standing Pose ---
// CALIBRATED: "center = home" — the robot stands correctly at all servos 0°.
// These values are FK-computed foot positions at (0°, 0°, 0°) for every joint.
// IK of these positions returns exactly (0°, 0°, 0°), confirming calibration.
const float DEFAULT_STAND_Z = -123.2;  // mm below body (FK at center)

// Foot home positions (FK of center position)
// DO NOT change unless you change leg dimensions or mount positions.
const float FOOT_FB_DIST  = 200.3;   // X distance for front/back feet (mm)
const float FOOT_WIDTH_FB = 105.5;   // Y distance for front/back feet (mm)
const float FOOT_WIDTH_M  = 206.7;   // Y distance for middle feet (mm)

// --- Walking Parameters ---
// Step geometry: how far and high each foot moves during walking.
// Larger STEP_LENGTH = faster travel but more risk of instability.
// Larger STEP_HEIGHT = more ground clearance but more energy.
const float STEP_LENGTH   =  40.0;   // Foot travel per step (mm)
const float STEP_HEIGHT   =  30.0;   // Foot lift during swing (mm)

// --- Motion Timing ---
// INTERPOLATION_STEPS × STEP_DELAY_MS = time per keyframe transition
// Default: 5 × 20ms = 100ms per phase, 400ms per full walk cycle
const uint8_t INTERPOLATION_STEPS = 5;    // Sub-steps between keyframes
const unsigned long STEP_DELAY_MS  = 20;   // Delay between sub-steps (ms)

// Safety limit: max servo travel per interpolation step.
// Prevents violent jerks if IK produces a big jump.
const float MAX_ANGLE_CHANGE_PER_STEP = 15.0;  // degrees

// ============================================================
// 7. SERIAL COMMUNICATION
// ============================================================
const unsigned long SERIAL_BAUD = 115200;

// ============================================================
// 8. DERIVED CONSTANTS (computed at compile time)
// ============================================================
// These are used internally for range checking. Do not modify.

// Maximum and minimum reach of the femur-tibia linkage
const float MAX_REACH = FEMUR_LENGTH + TIBIA_LENGTH;           // 211mm
const float MIN_REACH = fabs(FEMUR_LENGTH - TIBIA_LENGTH);     // 43mm

// Full leg max reach from body center (for foot position validation)
// This is the theoretical max — practical max is ~85% of this
const float FULL_MAX_REACH = COXA_LENGTH + MAX_REACH;          // 239mm

#endif // CONFIG_H
