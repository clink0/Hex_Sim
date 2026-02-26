/*
 * hexapod_walker.ino — Main Hexapod Robot Controller
 * ====================================================
 *
 * Hardware: Arduino Mega 2560 + 18× hobby servos (direct PWM)
 * Kit:     FutureTrace 18-DOF Hexapod Spider Robot Frame
 *
 * Architecture:
 *   hexapod_walker.ino  → Main loop, serial commands, state machine
 *       ├── Config.h     → All tunable parameters
 *       ├── Kinematics.h → Pure math: inverse & forward kinematics
 *       ├── HexLeg.h     → Leg controller: IK + servo output
 *       └── GaitEngine.h → Tripod gait keyframe generator
 *
 * IK Method: Geometric / Law of Cosines
 *   Same approach used by petercorke/robotics-toolbox-python for
 *   serial-link manipulators, validated on this kit by JakobLeander/hexapod.
 *
 * References:
 *   - github.com/JakobLeander/hexapod   — Same kit, proven IK & gait
 *   - github.com/petercorke/robotics-toolbox-python — General robotics math
 *
 * Serial Commands (115200 baud):
 *   Movement:   h=home  s=sleep  w=walk  b=backward  l=left  r=right  x=stop
 *   Calibration: c=center  1-6=wiggle leg  +=raise  -=lower
 *   Diagnostics: t=test IK  d=dump state  v=validate IK  p=print config
 */

#include "Config.h"
#include "Kinematics.h"
#include "HexLeg.h"
#include "GaitEngine.h"

// ═══════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════

HexLeg legs[6];    // 0=LF, 1=LM, 2=LB, 3=RF, 4=RM, 5=RB
GaitEngine gait;

// ─── State Machine ─────────────────────────────────────

enum RobotState {
    STATE_SLEEP,
    STATE_HOME,
    STATE_WALK_FWD,
    STATE_WALK_BWD,
    STATE_ROTATE_LEFT,
    STATE_ROTATE_RIGHT,
    STATE_TRANSITIONING
};

RobotState currentState = STATE_SLEEP;
RobotState desiredState = STATE_SLEEP;

// ─── Motion Interpolation ──────────────────────────────

HexPose currentPose;     // Where the robot IS
HexPose targetPose;      // Where the robot is GOING
int8_t  interpStep = -1; // Countdown: INTERPOLATION_STEPS → 0 (-1 = idle)
unsigned long lastStepTime = 0;

// ─── Walk Cycle ────────────────────────────────────────

uint8_t walkPhase  = 0;
bool    walkActive = false;

// ─── Height Control ────────────────────────────────────

float bodyHeight = DEFAULT_STAND_Z;
const float HEIGHT_STEP = 10.0f;
const float MIN_HEIGHT  = -180.0f;     // Lowest (near full extension)
const float MAX_HEIGHT  = -80.0f;      // Highest (legs more retracted)

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println(F(""));
    Serial.println(F("========================================="));
    Serial.println(F("  HEXAPOD WALKER — Booting..."));
    Serial.println(F("  IK: Geometric / Law of Cosines"));
    Serial.println(F("  Refs: JakobLeander/hexapod"));
    Serial.println(F("        petercorke/robotics-toolbox-python"));
    Serial.println(F("========================================="));
    Serial.println(F(""));

    // Print configuration
    printConfig();

    // Initialize all 6 legs with staggered delays
    // (prevents current surge from all servos moving at once)
    Serial.println(F("Initializing legs..."));

    legs[0].init("LF", MOUNT_LF_X, MOUNT_LF_Y,
                 PIN_LF_HIP, PIN_LF_KNEE, PIN_LF_FOOT,
                 REV_LF_HIP, REV_LF_KNEE, REV_LF_FOOT);
    delay(50);

    legs[1].init("LM", MOUNT_LM_X, MOUNT_LM_Y,
                 PIN_LM_HIP, PIN_LM_KNEE, PIN_LM_FOOT,
                 REV_LM_HIP, REV_LM_KNEE, REV_LM_FOOT);
    delay(50);

    legs[2].init("LB", MOUNT_LB_X, MOUNT_LB_Y,
                 PIN_LB_HIP, PIN_LB_KNEE, PIN_LB_FOOT,
                 REV_LB_HIP, REV_LB_KNEE, REV_LB_FOOT);
    delay(50);

    legs[3].init("RF", MOUNT_RF_X, MOUNT_RF_Y,
                 PIN_RF_HIP, PIN_RF_KNEE, PIN_RF_FOOT,
                 REV_RF_HIP, REV_RF_KNEE, REV_RF_FOOT);
    delay(50);

    legs[4].init("RM", MOUNT_RM_X, MOUNT_RM_Y,
                 PIN_RM_HIP, PIN_RM_KNEE, PIN_RM_FOOT,
                 REV_RM_HIP, REV_RM_KNEE, REV_RM_FOOT);
    delay(50);

    legs[5].init("RB", MOUNT_RB_X, MOUNT_RB_Y,
                 PIN_RB_HIP, PIN_RB_KNEE, PIN_RB_FOOT,
                 REV_RB_HIP, REV_RB_KNEE, REV_RB_FOOT);
    delay(50);

    // Start in sleep pose
    currentPose = gait.sleepPose();
    applyPose(currentPose);

    Serial.println(F(""));
    Serial.println(F("=== READY ==="));
    Serial.println(F("Movement: h=home s=sleep w=walk b=back l=left r=right x=stop"));
    Serial.println(F("Calibrate: c=center 1-6=wiggle +=up -=down"));
    Serial.println(F("Diagnostic: t=test d=dump v=validate p=config"));
    Serial.println(F(""));
    Serial.println(F(">>> FIRST: Hold robot in air, send 'c' to center <<<"));
    Serial.println(F(""));
}

// ═══════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════

void loop()
{
    // ─── Serial Command Handling ───
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    // ─── Motion Interpolation Engine ───
    // Smoothly transitions between poses using linear interpolation
    // with optional cubic easing (smoothStep in GaitEngine.h)
    if (interpStep >= 0 && (millis() - lastStepTime) >= STEP_DELAY_MS) {

        // Compute interpolation factor: 0.0 → 1.0 over INTERPOLATION_STEPS
        float t = (float)(INTERPOLATION_STEPS - interpStep) / (float)INTERPOLATION_STEPS;

        // Apply smooth easing for non-walk transitions
        bool useSmooth = !walkActive;
        HexPose intermediate = lerpPose(currentPose, targetPose, t, useSmooth);

        // Send to servos
        applyPose(intermediate);

        interpStep--;
        lastStepTime = millis();

        // Interpolation complete
        if (interpStep < 0) {
            currentPose = targetPose;

            // If walking, advance to next keyframe
            if (walkActive) {
                advanceWalk();
            }
        }
    }
}

// ═══════════════════════════════════════════════════════
//  COMMAND HANDLER
// ═══════════════════════════════════════════════════════

void handleCommand(char cmd)
{
    switch (cmd) {

    // ─── Movement Commands ───

    case 'h':  // Stand up (home position)
        Serial.println(F(">> HOME"));
        walkActive = false;
        desiredState = STATE_HOME;
        startTransition(gait.homePose(bodyHeight));
        break;

    case 's':  // Sleep (relax servos)
        Serial.println(F(">> SLEEP"));
        walkActive = false;
        desiredState = STATE_SLEEP;
        startTransition(gait.sleepPose());
        break;

    case 'w':  // Walk forward
        Serial.println(F(">> WALK FORWARD"));
        desiredState = STATE_WALK_FWD;
        walkActive = true;
        walkPhase = 0;
        gait.resetPhase();
        startTransition(gait.walkKeyframe(0, bodyHeight, 1.0f));
        break;

    case 'b':  // Walk backward
        Serial.println(F(">> WALK BACKWARD"));
        desiredState = STATE_WALK_BWD;
        walkActive = true;
        walkPhase = 0;
        gait.resetPhase();
        startTransition(gait.walkKeyframe(0, bodyHeight, -1.0f));
        break;

    case 'l':  // Rotate left
        Serial.println(F(">> ROTATE LEFT"));
        desiredState = STATE_ROTATE_LEFT;
        walkActive = true;
        walkPhase = 0;
        gait.resetPhase();
        startTransition(gait.rotateKeyframe(0, bodyHeight, 1.0f));
        break;

    case 'r':  // Rotate right
        Serial.println(F(">> ROTATE RIGHT"));
        desiredState = STATE_ROTATE_RIGHT;
        walkActive = true;
        walkPhase = 0;
        gait.resetPhase();
        startTransition(gait.rotateKeyframe(0, bodyHeight, -1.0f));
        break;

    case 'x':  // Stop walking, return to home
        Serial.println(F(">> STOP"));
        walkActive = false;
        desiredState = STATE_HOME;
        startTransition(gait.homePose(bodyHeight));
        break;

    // ─── Height Control ───

    case '+':
        bodyHeight += HEIGHT_STEP;
        if (bodyHeight > MAX_HEIGHT) bodyHeight = MAX_HEIGHT;
        Serial.print(F(">> HEIGHT: ")); Serial.print(bodyHeight); Serial.println(F("mm"));
        if (!walkActive) startTransition(gait.homePose(bodyHeight));
        break;

    case '-':
        bodyHeight -= HEIGHT_STEP;
        if (bodyHeight < MIN_HEIGHT) bodyHeight = MIN_HEIGHT;
        Serial.print(F(">> HEIGHT: ")); Serial.print(bodyHeight); Serial.println(F("mm"));
        if (!walkActive) startTransition(gait.homePose(bodyHeight));
        break;

    // ─── Calibration ───

    case 'c':  // Center all servos
        Serial.println(F(">> CENTER ALL SERVOS"));
        walkActive = false;
        for (int i = 0; i < 6; i++) {
            legs[i].center();
            delay(20);
        }
        Serial.println(F("   All servos at 0deg (center pulse)"));
        Serial.println(F("   Verify horns are mechanically centered."));
        break;

    case '1': case '2': case '3': case '4': case '5': case '6':
        wiggleLeg(cmd - '1');
        break;

    // ─── Diagnostics ───

    case 't':  // Test single leg IK
        testSingleLeg();
        break;

    case 'd':  // Dump all state
        dumpState();
        break;

    case 'v':  // Validate IK (round-trip test)
        validateAllLegs();
        break;

    case 'p':  // Print configuration
        printConfig();
        break;

    case '\n': case '\r':
        break;  // Ignore newlines

    default:
        Serial.print(F("Unknown: '"));
        Serial.print(cmd);
        Serial.println(F("' | h=home w=walk x=stop c=center t=test v=validate"));
        break;
    }
}

// ═══════════════════════════════════════════════════════
//  MOTION FUNCTIONS
// ═══════════════════════════════════════════════════════

/*
 * Begin smooth transition to a target pose.
 * The loop() interpolation engine handles the rest.
 */
void startTransition(HexPose target)
{
    targetPose = target;
    interpStep = INTERPOLATION_STEPS;
    lastStepTime = millis();
}

/*
 * Advance walk cycle to next phase.
 * Called automatically when interpolation completes during walking.
 */
void advanceWalk()
{
    walkPhase = (walkPhase + 1) % 4;

    HexPose next;
    switch (desiredState) {
    case STATE_WALK_FWD:
        next = gait.walkKeyframe(walkPhase, bodyHeight, 1.0f);
        break;
    case STATE_WALK_BWD:
        next = gait.walkKeyframe(walkPhase, bodyHeight, -1.0f);
        break;
    case STATE_ROTATE_LEFT:
        next = gait.rotateKeyframe(walkPhase, bodyHeight, 1.0f);
        break;
    case STATE_ROTATE_RIGHT:
        next = gait.rotateKeyframe(walkPhase, bodyHeight, -1.0f);
        break;
    default:
        walkActive = false;
        next = gait.homePose(bodyHeight);
        break;
    }

    startTransition(next);
}

/*
 * Apply a full pose to all 6 legs.
 * Uses direct positioning (no rate limiting) since the interpolation
 * engine provides smooth transitions between keyframes.
 */
void applyPose(HexPose pose)
{
    legs[0].setFootPositionDirect(pose.lf);
    legs[1].setFootPositionDirect(pose.lm);
    legs[2].setFootPositionDirect(pose.lb);
    legs[3].setFootPositionDirect(pose.rf);
    legs[4].setFootPositionDirect(pose.rm);
    legs[5].setFootPositionDirect(pose.rb);
}

// ═══════════════════════════════════════════════════════
//  TEST & CALIBRATION FUNCTIONS
// ═══════════════════════════════════════════════════════

/*
 * Test IK on the left front leg with a controlled movement sequence.
 * Moves through: home → lift → forward → place → return
 */
void testSingleLeg()
{
    Serial.println(F(">> TEST: LF leg IK sequence"));
    Serial.println(F("   (Other legs at home)"));

    // First stand at home
    HexPose home = gait.homePose(bodyHeight);
    applyPose(home);
    delay(500);

    // Test positions for LF leg
    Vec3 tests[] = {
        { FOOT_FB_DIST,      FOOT_WIDTH_FB, bodyHeight},          // Home
        { FOOT_FB_DIST,      FOOT_WIDTH_FB, bodyHeight + 30},     // Lift
        { FOOT_FB_DIST + 30, FOOT_WIDTH_FB, bodyHeight + 30},     // Forward+up
        { FOOT_FB_DIST + 30, FOOT_WIDTH_FB, bodyHeight},          // Forward+down
        { FOOT_FB_DIST,      FOOT_WIDTH_FB, bodyHeight},          // Home
    };
    const char* desc[] = {
        "Home", "Lift up", "Forward+up", "Place down", "Return"
    };

    for (int i = 0; i < 5; i++) {
        Serial.print(F("   ")); Serial.print(i);
        Serial.print(F(": ")); Serial.print(desc[i]);
        Serial.print(F(" ("));
        Serial.print(tests[i].x, 0); Serial.print(F(","));
        Serial.print(tests[i].y, 0); Serial.print(F(","));
        Serial.print(tests[i].z, 0); Serial.println(F(")"));

        bool ok = legs[0].setFootPosition(tests[i]);
        if (ok) {
            Serial.print(F("     "));
            legs[0].printState();
        } else {
            Serial.println(F("     >>> UNREACHABLE! Check dimensions in Config.h"));
        }
        delay(700);
    }

    Serial.println(F(">> TEST COMPLETE"));
    Serial.println(F("   Wrong direction? Flip REV_LF_xxx in Config.h"));
    Serial.println(F("   Wrong reach? Measure & update dimensions in Config.h"));
}

/*
 * Wiggle a leg's coxa servo to identify which physical leg it is.
 */
void wiggleLeg(uint8_t idx)
{
    if (idx >= 6) return;
    const char* names[] = {"LF", "LM", "LB", "RF", "RM", "RB"};
    Serial.print(F(">> WIGGLE ")); Serial.println(names[idx]);

    float savedC = legs[idx].currentAngles.coxa;
    float savedF = legs[idx].currentAngles.femur;
    float savedT = legs[idx].currentAngles.tibia;

    for (int i = 0; i < 3; i++) {
        legs[idx].setAngles( 10, savedF, savedT); delay(200);
        legs[idx].setAngles(-10, savedF, savedT); delay(200);
    }
    legs[idx].setAngles(savedC, savedF, savedT);
    Serial.println(F("   Which physical leg wiggled its hip?"));
}

/*
 * Run IK round-trip validation on all legs.
 * For each leg, tests 5 positions and reports accuracy.
 * Error should be < 0.5mm for all positions.
 */
void validateAllLegs()
{
    Serial.println(F(""));
    Serial.println(F("=== IK ROUND-TRIP VALIDATION ==="));
    Serial.println(F("Method: target → IK → FK → compare (error should be < 0.5mm)"));
    Serial.println(F(""));

    for (int i = 0; i < 6; i++) {
        legs[i].runIKValidation();
        Serial.println(F(""));
    }

    // Also test the home pose positions
    Serial.println(F("=== HOME POSE VALIDATION ==="));
    HexPose home = gait.homePose(bodyHeight);
    printIKValidation(home.lf, MOUNT_LF_X, MOUNT_LF_Y, "LF home");
    printIKValidation(home.lm, MOUNT_LM_X, MOUNT_LM_Y, "LM home");
    printIKValidation(home.lb, MOUNT_LB_X, MOUNT_LB_Y, "LB home");
    printIKValidation(home.rf, MOUNT_RF_X, MOUNT_RF_Y, "RF home");
    printIKValidation(home.rm, MOUNT_RM_X, MOUNT_RM_Y, "RM home");
    printIKValidation(home.rb, MOUNT_RB_X, MOUNT_RB_Y, "RB home");
    Serial.println(F(""));

    // Test walk extremes
    Serial.println(F("=== WALK EXTREME VALIDATION ==="));
    for (int phase = 0; phase < 4; phase++) {
        HexPose wp = gait.walkKeyframe(phase, bodyHeight, 1.0f);
        char label[16];
        snprintf(label, sizeof(label), "Walk P%d LF", phase);
        printIKValidation(wp.lf, MOUNT_LF_X, MOUNT_LF_Y, label);
    }
    Serial.println(F(""));
    Serial.println(F("=== VALIDATION COMPLETE ==="));
}

/*
 * Dump current state of all legs.
 */
void dumpState()
{
    Serial.println(F("=== ROBOT STATE ==="));
    for (int i = 0; i < 6; i++) {
        legs[i].printState();
    }
    Serial.print(F("Height: ")); Serial.print(bodyHeight); Serial.println(F("mm"));
    Serial.print(F("Walking: ")); Serial.println(walkActive ? F("yes") : F("no"));
    Serial.print(F("Phase: ")); Serial.println(walkPhase);
    Serial.println(F(""));
}

/*
 * Print current configuration for verification.
 */
void printConfig()
{
    Serial.println(F("=== CONFIGURATION ==="));
    Serial.print(F("Segments (mm): coxa=")); Serial.print(COXA_LENGTH);
    Serial.print(F(" femur=")); Serial.print(FEMUR_LENGTH);
    Serial.print(F(" tibia=")); Serial.println(TIBIA_LENGTH);
    Serial.print(F("Reach range: ")); Serial.print(MIN_REACH, 1);
    Serial.print(F(" — ")); Serial.print(MAX_REACH, 1); Serial.println(F("mm"));
    Serial.print(F("Full max reach: ")); Serial.print(FULL_MAX_REACH, 1); Serial.println(F("mm"));
    Serial.print(F("Standing height: ")); Serial.print(bodyHeight); Serial.println(F("mm"));
    Serial.print(F("Step: length=")); Serial.print(STEP_LENGTH);
    Serial.print(F("mm height=")); Serial.print(STEP_HEIGHT); Serial.println(F("mm"));
    Serial.print(F("Tibia offset: ")); Serial.print(TIBIA_OFFSET_DEG); Serial.println(F("deg"));
    Serial.print(F("Pulse range: ")); Serial.print(SERVO_PULSE_MIN);
    Serial.print(F(" — ")); Serial.print(SERVO_PULSE_MAX); Serial.println(F("us"));

    // Verify home position reachability
    float h_lf = sqrtf(
        (FOOT_FB_DIST - MOUNT_LF_X) * (FOOT_FB_DIST - MOUNT_LF_X) +
        (FOOT_WIDTH_FB - MOUNT_LF_Y) * (FOOT_WIDTH_FB - MOUNT_LF_Y)
    );
    float D_lf = sqrtf(
        (h_lf - COXA_LENGTH) * (h_lf - COXA_LENGTH) +
        bodyHeight * bodyHeight
    );
    Serial.print(F("LF home reach: D=")); Serial.print(D_lf, 1);
    Serial.print(F("mm ["));
    Serial.print(MIN_REACH, 1); Serial.print(F(","));
    Serial.print(MAX_REACH, 1); Serial.print(F("] "));
    Serial.println((D_lf >= MIN_REACH && D_lf <= MAX_REACH) ? F("OK") : F("OUT OF RANGE!"));
    Serial.println(F(""));
}
