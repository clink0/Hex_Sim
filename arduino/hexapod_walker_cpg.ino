/*
 * hexapod_walker.ino — Hexapod Robot Controller (CPG Edition)
 * ============================================================
 *
 * Hardware: Arduino Mega 2560 + 18× hobby servos (direct PWM)
 * Kit:      FutureTrace 18-DOF Hexapod Spider Robot Frame
 *
 * Architecture:
 *   hexapod_walker.ino  → Main loop, serial commands, state machine
 *       ├── Config.h     → All tunable parameters       (UNCHANGED)
 *       ├── Kinematics.h → IK / FK math                 (UNCHANGED)
 *       ├── HexLeg.h     → Leg controller + servo output (UNCHANGED)
 *       └── CPGEngine.h  → Kuramoto CPG gait controller  (NEW)
 *
 * What changed from the keyframe version:
 *   BEFORE: 4-keyframe state machine + linear interpolation.
 *           Hard-coded tripod only. Phase offsets fixed at startup.
 *
 *   AFTER:  6 Kuramoto coupled oscillators, updated at 40Hz.
 *           Gait (wave/ripple/tripod) emerges from coupling dynamics.
 *           Change ω with '+'/'-' → gait transitions automatically.
 *           Disturbing a leg? CPG re-synchronises within ~3 steps.
 *
 * Main loop timing (Mega 2560 @ 16MHz, no FPU):
 *   CPG step():       ~2.5ms  (11 sin() calls)
 *   6× IK + servos:  ~3.5ms  (atan2, acos, sqrt per leg)
 *   Serial + misc:   ~0.5ms
 *   Total:           ~6.5ms  → running at 40Hz (25ms period) ✓
 *
 * Serial Commands (115200 baud):
 *   Movement:    h=home  s=sleep  w=walk  b=back  l=left  r=right  x=stop
 *   Gait/Speed:  +=faster  -=slower  (shifts ω: wave ↔ ripple ↔ tripod)
 *   Calibration: c=center  1-6=wiggle leg  ^=raise  v=lower
 *   Diagnostics: d=dump  p=cpg status  t=test IK  q=validate IK
 *
 * References:
 *   - CPGEngine.h for the full mathematical description
 *   - hexapod_cpg.py (Python sim) — identical math, interactive visualisation
 *   - github.com/JakobLeander/hexapod — IK validation for this kit
 */

#include "Config.h"
#include "Kinematics.h"
#include "HexLeg.h"
#include "CPGEngine.h"

// ════════════════════════════════════════════════════════════
//  GLOBALS
// ════════════════════════════════════════════════════════════

HexLeg    legs[6];   // 0=LF  1=LM  2=LB  3=RF  4=RM  5=RB
CPGEngine cpg;

// ── State ───────────────────────────────────────────────────

enum RobotState {
    STATE_SLEEP,
    STATE_HOME,
    STATE_WALK_FWD,
    STATE_WALK_BWD,
    STATE_ROTATE_LEFT,
    STATE_ROTATE_RIGHT
};

RobotState currentState = STATE_SLEEP;

// ── Control timing ──────────────────────────────────────────

const unsigned long CONTROL_PERIOD_MS = 25;   // 40Hz update rate
unsigned long lastControlTime = 0;

// ── Height control ──────────────────────────────────────────

float bodyHeight     = DEFAULT_STAND_Z;
const float HEIGHT_STEP = 10.0f;
const float MIN_HEIGHT  = -180.0f;
const float MAX_HEIGHT  =  -80.0f;

// ── Startup transition ──────────────────────────────────────
// Simple linear interpolation used only at startup to smoothly
// move from sleep to home pose before CPG takes over.

bool  startupDone    = false;
Vec3  startupFrom[6];
Vec3  startupTo[6];
int   startupStep    = -1;
const int STARTUP_STEPS = 60;    // 60 × 25ms = 1.5 second rise

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════╗"));
    Serial.println(F("║   HEXAPOD CPG CONTROLLER           ║"));
    Serial.println(F("║   Kuramoto Coupled Oscillators     ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    Serial.println(F(""));

    printConfig();

    // ── Initialise legs ──────────────────────────────────────
    Serial.println(F("Initialising legs..."));

    legs[0].init("LF", MOUNT_LF_X, MOUNT_LF_Y,
                 PIN_LF_HIP,  PIN_LF_KNEE,  PIN_LF_FOOT,
                 REV_LF_HIP,  REV_LF_KNEE,  REV_LF_FOOT);  delay(50);

    legs[1].init("LM", MOUNT_LM_X, MOUNT_LM_Y,
                 PIN_LM_HIP,  PIN_LM_KNEE,  PIN_LM_FOOT,
                 REV_LM_HIP,  REV_LM_KNEE,  REV_LM_FOOT);  delay(50);

    legs[2].init("LB", MOUNT_LB_X, MOUNT_LB_Y,
                 PIN_LB_HIP,  PIN_LB_KNEE,  PIN_LB_FOOT,
                 REV_LB_HIP,  REV_LB_KNEE,  REV_LB_FOOT);  delay(50);

    legs[3].init("RF", MOUNT_RF_X, MOUNT_RF_Y,
                 PIN_RF_HIP,  PIN_RF_KNEE,  PIN_RF_FOOT,
                 REV_RF_HIP,  REV_RF_KNEE,  REV_RF_FOOT);  delay(50);

    legs[4].init("RM", MOUNT_RM_X, MOUNT_RM_Y,
                 PIN_RM_HIP,  PIN_RM_KNEE,  PIN_RM_FOOT,
                 REV_RM_HIP,  REV_RM_KNEE,  REV_RM_FOOT);  delay(50);

    legs[5].init("RB", MOUNT_RB_X, MOUNT_RB_Y,
                 PIN_RB_HIP,  PIN_RB_KNEE,  PIN_RB_FOOT,
                 REV_RB_HIP,  REV_RB_KNEE,  REV_RB_FOOT);  delay(50);

    Serial.println(F("Legs ready."));
    Serial.println(F(""));

    // ── Startup: begin smooth rise to home pose ───────────────
    beginStartupRise();

    Serial.println(F("═══════════════════════════════════════"));
    Serial.println(F("  READY  —  CPG Edition"));
    Serial.println(F("───────────────────────────────────────"));
    Serial.println(F("  w=walk  b=back  l=left  r=right  x=stop"));
    Serial.println(F("  h=home  s=sleep"));
    Serial.println(F("  +=faster  -=slower  (wave<->ripple<->tripod)"));
    Serial.println(F("  p=cpg_status  d=dump  c=center"));
    Serial.println(F("═══════════════════════════════════════"));
    Serial.println(F(""));
    Serial.println(F(">>> TIP: Try '+' several times while walking <<<"));
    Serial.println(F("        Watch the gait pattern change live!"));
    Serial.println(F(""));
}

// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════

void loop()
{
    // ── 1. Serial command handling (non-blocking) ──────────────
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    // ── 2. Fixed-rate control loop (40Hz) ─────────────────────
    unsigned long now = millis();
    if (now - lastControlTime < CONTROL_PERIOD_MS) return;
    float dt = (float)(now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    // Clamp dt in case something stalled (e.g. Serial flood)
    if (dt > 0.1f) dt = 0.025f;

    // ── 3. Handle startup rise (runs once at boot) ─────────────
    if (!startupDone) {
        doStartupStep();
        return;
    }

    // ── 4. Step CPG ────────────────────────────────────────────
    //  Advances all 6 oscillator phases by dt seconds.
    //  Gait coordination falls out of the coupling math.
    cpg.step(dt);

    // ── 5. Apply foot positions ────────────────────────────────
    //  Convert each phase to a foot target and solve IK.
    if (cpg.moving) {
        for (uint8_t i = 0; i < 6; i++) {
            Vec3 target = cpg.getFootPosition(i);
            legs[i].setFootPositionDirect(target);
        }
    }
}

// ════════════════════════════════════════════════════════════
//  STARTUP RISE
//  Smoothly moves from center (sleep) → home pose over 1.5s.
//  After this, the CPG takes over.
// ════════════════════════════════════════════════════════════

void beginStartupRise()
{
    // From: all servos centered (0°, which the HexLeg init() left them at)
    // To: home/standing pose using CPG neutral positions
    for (uint8_t i = 0; i < 6; i++) {
        Vec3 from;
        from.x = CPG_NEUTRAL_X[i] + MOUNTS_X(i);
        from.y = CPG_NEUTRAL_Y[i];
        from.z = 0.0f;   // legs up near body center height

        Vec3 to;
        to.x = CPG_NEUTRAL_X[i];
        to.y = CPG_NEUTRAL_Y[i];
        to.z = bodyHeight;

        startupFrom[i] = from;
        startupTo[i]   = to;
    }
    startupStep = 0;
    Serial.println(F("Rising to home pose..."));
}

/*
 * Returns the mount X offset for leg i.
 * Needed to build a valid starting foot position at startup.
 */
inline float MOUNTS_X(uint8_t i) {
    const float mx[] = {
        MOUNT_LF_X, MOUNT_LM_X, MOUNT_LB_X,
        MOUNT_RF_X, MOUNT_RM_X, MOUNT_RB_X
    };
    return mx[i];
}

void doStartupStep()
{
    if (startupStep < 0 || startupStep >= STARTUP_STEPS) {
        startupDone = true;
        currentState = STATE_HOME;
        Serial.println(F("Home. Ready for commands."));
        return;
    }

    float t  = (float)startupStep / (float)(STARTUP_STEPS - 1);
    float te = t * t * (3.0f - 2.0f * t);   // cubic ease

    for (uint8_t i = 0; i < 6; i++) {
        Vec3 foot;
        foot.x = startupFrom[i].x + (startupTo[i].x - startupFrom[i].x) * te;
        foot.y = startupFrom[i].y + (startupTo[i].y - startupFrom[i].y) * te;
        foot.z = startupFrom[i].z + (startupTo[i].z - startupFrom[i].z) * te;
        legs[i].setFootPositionDirect(foot);
    }

    startupStep++;
}

// ════════════════════════════════════════════════════════════
//  COMMAND HANDLER
// ════════════════════════════════════════════════════════════

void handleCommand(char cmd)
{
    switch (cmd) {

    // ── Movement ──────────────────────────────────────────────

    case 'w':
        Serial.print(F(">> WALK FWD  ["));
        Serial.print(cpg.gaitName()); Serial.println(F("]"));
        currentState = STATE_WALK_FWD;
        cpg.walkForward();
        break;

    case 'b':
        Serial.print(F(">> WALK BACK ["));
        Serial.print(cpg.gaitName()); Serial.println(F("]"));
        currentState = STATE_WALK_BWD;
        cpg.walkBackward();
        break;

    case 'l':
        Serial.println(F(">> ROTATE LEFT"));
        currentState = STATE_ROTATE_LEFT;
        cpg.turnLeft();
        break;

    case 'r':
        Serial.println(F(">> ROTATE RIGHT"));
        currentState = STATE_ROTATE_RIGHT;
        cpg.turnRight();
        break;

    case 'x':
        Serial.println(F(">> STOP"));
        cpg.stop();
        currentState = STATE_HOME;
        break;

    // ── Height adjust ────────────────────────────────────────

    case '^':   // Raise body (servos retract, body goes up)
        bodyHeight = constrain(bodyHeight + HEIGHT_STEP, MIN_HEIGHT, MAX_HEIGHT);
        Serial.print(F(">> HEIGHT: ")); Serial.println(bodyHeight);
        break;

    case 'v':   // Lower body
        bodyHeight = constrain(bodyHeight - HEIGHT_STEP, MIN_HEIGHT, MAX_HEIGHT);
        Serial.print(F(">> HEIGHT: ")); Serial.println(bodyHeight);
        break;

    // ── Home / sleep ─────────────────────────────────────────

    case 'h': {
        Serial.println(F(">> HOME"));
        cpg.stop();
        currentState = STATE_HOME;
        // Smoothly return all feet to neutral positions
        for (uint8_t i = 0; i < 6; i++) {
            Vec3 home;
            home.x = CPG_NEUTRAL_X[i];
            home.y = CPG_NEUTRAL_Y[i];
            home.z = bodyHeight;
            legs[i].setFootPosition(home);
        }
        break;
    }

    case 's': {
        Serial.println(F(">> SLEEP"));
        cpg.stop();
        currentState = STATE_SLEEP;
        // All legs draw up to near-body height
        for (uint8_t i = 0; i < 6; i++) {
            Vec3 sleep;
            sleep.x = CPG_NEUTRAL_X[i];
            sleep.y = CPG_NEUTRAL_Y[i];
            sleep.z = 0.0f;
            legs[i].setFootPosition(sleep);
        }
        delay(500);
        for (uint8_t i = 0; i < 6; i++) legs[i].detach();
        Serial.println(F("   Servos released."));
        break;
    }

    // ── Speed / gait control (ω) ─────────────────────────────
    //
    // This is the key CPG feature: changing ω changes both speed
    // AND gait type. The transition is smooth — no mode switching.

    case '+':
        cpg.setSpeed(cpg.omega + 0.5f);
        Serial.print(F(">> omega="));  Serial.print(cpg.omega, 2);
        Serial.print(F("  gait="));    Serial.println(cpg.gaitName());
        break;

    case '-':
        cpg.setSpeed(cpg.omega - 0.5f);
        Serial.print(F(">> omega="));  Serial.print(cpg.omega, 2);
        Serial.print(F("  gait="));    Serial.println(cpg.gaitName());
        break;

    // ── Diagnostics ──────────────────────────────────────────

    case 'p':
        cpg.printStatus();
        break;

    case 'd':
        dumpState();
        break;

    case 't':
        testSingleLeg();
        break;

    case 'q':
        validateAllLegs();
        break;

    case 'c':
        Serial.println(F(">> CENTER ALL SERVOS"));
        cpg.stop();
        for (uint8_t i = 0; i < 6; i++) legs[i].center();
        break;

    // ── Wiggle individual legs (for physical identification) ──

    case '1': wiggleLeg(0); break;   // LF
    case '2': wiggleLeg(1); break;   // LM
    case '3': wiggleLeg(2); break;   // LB
    case '4': wiggleLeg(3); break;   // RF
    case '5': wiggleLeg(4); break;   // RM
    case '6': wiggleLeg(5); break;   // RB

    case '\n': case '\r': break;     // ignore newlines

    default:
        Serial.print(F("? '"));
        Serial.print(cmd);
        Serial.println(F("'  |  w=walk b=back l=left r=right x=stop h=home s=sleep"));
        Serial.println(F("            +=faster -=slower  p=cpg d=dump c=center 1-6=wiggle"));
        break;
    }
}

// ════════════════════════════════════════════════════════════
//  DIAGNOSTIC FUNCTIONS
// ════════════════════════════════════════════════════════════

void dumpState()
{
    Serial.println(F(""));
    Serial.println(F("═══ ROBOT STATE ═══"));
    for (uint8_t i = 0; i < 6; i++) {
        legs[i].printState();
    }
    Serial.print(F("Height: "));  Serial.print(bodyHeight);  Serial.println(F("mm"));
    Serial.println(F(""));
    cpg.printStatus();
    Serial.println(F(""));
}

void testSingleLeg()
{
    Serial.println(F(">> TEST: LF leg IK sequence"));
    cpg.stop();

    Vec3 tests[] = {
        {FOOT_FB_DIST,      FOOT_WIDTH_FB, bodyHeight},
        {FOOT_FB_DIST,      FOOT_WIDTH_FB, bodyHeight + 30},
        {FOOT_FB_DIST + 40, FOOT_WIDTH_FB, bodyHeight + 30},
        {FOOT_FB_DIST + 40, FOOT_WIDTH_FB, bodyHeight},
        {FOOT_FB_DIST,      FOOT_WIDTH_FB, bodyHeight},
    };
    const char* desc[] = {"Home","Lift","Fwd+up","Place","Return"};

    for (int i = 0; i < 5; i++) {
        Serial.print(F("  ")); Serial.print(desc[i]); Serial.print(F(": "));
        bool ok = legs[0].setFootPosition(tests[i]);
        if (ok) {
            legs[0].printState();
        } else {
            Serial.println(F("UNREACHABLE — check Config.h dimensions"));
        }
        delay(600);
    }
    Serial.println(F(">> TEST COMPLETE"));
}

void wiggleLeg(uint8_t idx)
{
    if (idx >= 6) return;
    const char* names[] = {"LF","LM","LB","RF","RM","RB"};
    Serial.print(F(">> WIGGLE ")); Serial.println(names[idx]);

    float sc = legs[idx].currentAngles.coxa;
    float sf = legs[idx].currentAngles.femur;
    float st = legs[idx].currentAngles.tibia;

    for (int i = 0; i < 3; i++) {
        legs[idx].setAngles(sc + 15, sf, st); delay(200);
        legs[idx].setAngles(sc - 15, sf, st); delay(200);
    }
    legs[idx].setAngles(sc, sf, st);
}

void validateAllLegs()
{
    Serial.println(F(""));
    Serial.println(F("═══ IK ROUND-TRIP VALIDATION ═══"));
    Serial.println(F("Target → IK → FK → compare (should be < 0.5mm)"));
    Serial.println(F(""));
    for (uint8_t i = 0; i < 6; i++) {
        legs[i].runIKValidation();
    }
    Serial.println(F("═══ DONE ═══"));
}

void printConfig()
{
    Serial.println(F("═══ CONFIG ═══"));
    Serial.print(F("Segments: coxa="));  Serial.print(COXA_LENGTH);
    Serial.print(F(" femur="));          Serial.print(FEMUR_LENGTH);
    Serial.print(F(" tibia="));          Serial.println(TIBIA_LENGTH);
    Serial.print(F("Reach: "));          Serial.print(MIN_REACH,1);
    Serial.print(F("–"));               Serial.print(MAX_REACH,1);
    Serial.println(F("mm"));
    Serial.print(F("Stand Z: "));        Serial.println(bodyHeight);
    Serial.print(F("CPG omega: "));      Serial.print(cpg.omega,2);
    Serial.print(F("  gait: "));         Serial.println(cpg.gaitName());
    Serial.println(F(""));
}
