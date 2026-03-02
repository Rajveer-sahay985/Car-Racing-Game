// =============================================================================
//  Vortex Racing Engine — STEP 1: Bullet btRaycastVehicle physics
//  Camera, rendering, models: UNCHANGED from Step 0
//  Only the physics simulation is swapped from bicycle model → Bullet
//
//  Build:
//    clang++ Car_Physics_Engine.cpp -std=c++17 \
//      -I/opt/homebrew/include -I/opt/homebrew/include/bullet \
//      -L/opt/homebrew/lib \
//      -lraylib -lBulletDynamics -lBulletCollision -lLinearMath \
//      -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo \
//      -O2 -o car_main && ./car_main
// =============================================================================
#include "raylib.h"
#include "raymath.h"

// --- Bullet ---
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include <cmath>

// =============================================================================
//  BULLET SETUP  (global pointers — easier to clean up)
// =============================================================================
static btDefaultCollisionConfiguration*     gCollCfg    = nullptr;
static btCollisionDispatcher*               gDispatcher = nullptr;
static btBroadphaseInterface*               gBroadphase = nullptr;
static btSequentialImpulseConstraintSolver* gSolver     = nullptr;
static btDiscreteDynamicsWorld*             gWorld      = nullptr;

// Helper: create a rigid body and add it to the world
static btRigidBody* MakeRigidBody(float mass,
                                   const btTransform& startTrans,
                                   btCollisionShape* shape)
{
    btVector3 inertia(0, 0, 0);
    if (mass > 0.0f) shape->calculateLocalInertia(mass, inertia);
    btDefaultMotionState* ms = new btDefaultMotionState(startTrans);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, ms, shape, inertia);
    btRigidBody* body = new btRigidBody(ci);
    gWorld->addRigidBody(body);
    return body;
}

// =============================================================================
//  OBSTACLE — pairs a visible raylib Model with a Bullet static collider
// =============================================================================
struct Obstacle {
    Model         model;
    btRigidBody*  body        = nullptr;
    btBoxShape*   shape       = nullptr;
    Vector3       worldPos    = {0,0,0};  // world centre of the collider
    float         visualScale = 1.0f;
    Color         tint        = WHITE;
};

// Build a static box obstacle from an OBJ file.
// halfExtents: the Bullet box half-extents in world units.
// worldPos:    centre of the box in world space (y should be half-height to sit on ground).
static Obstacle MakeObstacle(const char* objFile, Shader sh,
                              btVector3 halfExtents, Vector3 worldPos,
                              float visualScale = 1.0f, Color tint = WHITE)
{
    Obstacle obs;
    obs.model       = LoadModel(objFile);
    obs.worldPos    = worldPos;
    obs.visualScale = visualScale;
    obs.tint        = tint;

    for (int m = 0; m < obs.model.materialCount; m++)
        obs.model.materials[m].shader = sh;

    obs.shape = new btBoxShape(halfExtents);

    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(worldPos.x, worldPos.y, worldPos.z));

    obs.body = MakeRigidBody(0.0f, t, obs.shape);  // mass=0 → static
    obs.body->setFriction(0.85f);
    obs.body->setRestitution(0.3f);
    return obs;
}

// =============================================================================
//  MAIN
// =============================================================================
int main()
{
    InitWindow(1280, 720, "Vortex Racing — Bullet Physics");
    SetTargetFPS(60);

    // =========================================================================
    // ██████████████████████████████████████████████████████████████████████
    //  TUNING PARAMETERS — change these freely to adjust feel!
    //  All physics, steering, drift and body-feel values live here.
    // ██████████████████████████████████████████████████████████████████████
    // =========================================================================

    // --- Engine / Brakes ---
    const float P_ENGINE_FORCE    = 3000.0f;  // N on rear wheels (higher = faster)
    const float P_BRAKE_FORCE     = 160.0f;   // N·m (higher = sharper brake)
    const float P_ENGINE_IN_RATE  = 6.0f;     // throttle ramp speed
    const float P_ENGINE_OUT_RATE = 10.0f;    // coast ramp speed

    // --- Steering ---
    const float P_MAX_STEER       = 35.0f * DEG2RAD; // max angle at low speed
    const float P_STEER_IN_RATE   = 3.5f;    // how fast steer locks in
    const float P_STEER_OUT_RATE  = 7.0f;    // how fast steer returns to 0
    const float P_STEER_SPEED_REDUCE = 0.77f;// how much high speed reduces steer

    // --- Suspension ---
    const float P_SUSP_STIFFNESS  = 25.0f;   // spring stiffness (higher = stiffer ride)
    const float P_SUSP_DAMPING    = 3.0f;    // oscillation damping
    const float P_SUSP_COMPRESS   = 4.0f;    // compression damping
    const float P_SUSP_TRAVEL_CM  = 20.0f;   // max travel in cm
    const float P_SUSP_MAX_FORCE  = 7000.0f; // max spring force (N)
    const float P_SUSP_LENGTH     = 0.45f;   // rest length (m)
    const float P_WHEEL_RADIUS    = 0.333f;  // visual + physics wheel radius (m)

    // --- Chassis ---
    const float P_CHASSIS_MASS    = 700.0f;  // kg (lower = lighter, easier to drift)
    const float P_LINEAR_DAMP     = 0.15f;   // air/roll resistance (0=float, 0.3=sluggish)
    const float P_ANGULAR_DAMP    = 0.75f;   // yaw drag
    const float P_ROLL_INFLUENCE  = 0.15f;   // anti-roll (0=tippy, 0.5=planted)

    // --- Traction / Front & Rear Grip ---
    const float P_FRONT_FRICTION  = 2.5f;    // front wheel grip
    const float P_REAR_FRICTION   = 2.8f;    // rear wheel grip (normal)

    // --- Drift ---
    const float P_HB_FRICTION      = 0.85f;  // rear grip: low-speed handbrake (launch feel)
    const float P_HB_HIGH_SPD_FRIC = 0.35f;  // rear grip: HIGH-SPEED handbrake (drift entry)
                                              //   lower = more snap, try 0.25-0.5
    const float P_HB_SPEED_THRESH  = 15.0f;  // km/h threshold between low/high-speed HB modes
    const float P_DRIFT_SUSTAIN    = 2.4f;   // how fast grip drops while sliding
    const float P_DRIFT_MIN_FRIC   = 1.2f;   // minimum rear grip during sustained slide
    const float P_MAX_YAW_RATE     = 1.5f;   // rad/s cap — main anti-spinout dial
                                              //   1.0=very controlled, 2.5=wild spins
    const float P_COUNTERSTEER     = 5000.0f;// countersteer torque assist

    // --- Arcade Fake Drift (Research-Based) ---
    const float P_DRIFT_MOMENTUM   = 0.55f;  // "imaginary wall" — resists lateral change
                                              //   higher = longer arcs, less spinout
    const float P_WEIGHT_TRANSFER  = 0.7f;   // braking rear-unload fraction (0-1)
    const float P_DRIFT_ANGLE_ENTRY= 8.0f;   // slip angle (°) to enter drift state
    const float P_DRIFT_ANGLE_EXIT = 4.0f;   // slip angle (°) to exit drift state
    const float P_DRIFT_POWER_FORCE= 2200.0f;// power oversteer: extra forward force during drift
                                              //   simulates rear wheels pushing car along heading
                                              //   higher = bigger arcs, more aggressive slide

    // --- RWD Launch / Wheel Spin ---
    const float P_LAUNCH_SPIN_FRIC = 0.18f;  // rear friction at ZERO speed (full wheel spin)
                                              //   MUST be < 0.68 to actually slip under 3000N
                                              //   lower = more burnout, try 0.1-0.3
    const float P_LAUNCH_SPEED_KMH = 22.0f;  // speed at which grip fully recovers
    const float P_LAUNCH_KICK      = 900.0f; // torque impulse on fresh throttle (N·m·s)
    const float P_LAUNCH_BOG_FORCE = 1800.0f;// backward force during wheel spin (N)
                                              //   keeps car in place while tires heat up
    const float P_DONUT_FORCE      = 3200.0f;// fake lateral rear force for donuts (N)
                                              //   W+SPACE+A→rear pushed right→car spins left
                                              //   W+SPACE+D→rear pushed left→car spins right
                                              //   higher = tighter/faster donut

    // --- Body Feel (visual dynamics) ---
    const float P_BODY_ROLL       = 0.06f;   // lean amount in corners (rad per m/s lateral)
    const float P_SUSP_BOB_SCALE  = 0.7f;   // suspension height bob multiplier
    const float P_VIB_AMP         = 0.003f;  // engine vibration amplitude
    const float P_VIB_FREQ        = 14.0f;   // engine vibration frequency (Hz)

    // =========================================================================

    // -------------------------------------------------------------------------
    const char *celVS =
        "#version 410 core\n"
        "in vec3 vertexPosition;\n"
        "in vec3 vertexNormal;\n"
        "in vec2 vertexTexCoord;\n"
        "in vec4 vertexColor;\n"
        "uniform mat4 mvp;\n"
        "uniform mat4 matModel;\n"
        "uniform mat4 matNormal;\n"
        "out vec3 fragNormal;\n"
        "out vec2 fragTexCoord;\n"
        "void main() {\n"
        "    fragNormal   = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));\n"
        "    fragTexCoord = vertexTexCoord;\n"
        "    gl_Position  = mvp * vec4(vertexPosition, 1.0);\n"
        "}\n";

    const char *celFS =
        "#version 410 core\n"
        "in vec3 fragNormal;\n"
        "in vec2 fragTexCoord;\n"
        "uniform sampler2D texture0;\n"
        "uniform vec4 colDiffuse;\n"
        "out vec4 finalColor;\n"
        "void main() {\n"
        "    vec3 lightDir = normalize(vec3(0.4, 1.0, 0.3));\n"
        "    float NdotL = dot(normalize(fragNormal), lightDir);\n"
        "    float intensity;\n"
        "    if      (NdotL > 0.6) intensity = 1.0;\n"
        "    else if (NdotL > 0.3) intensity = 0.75;\n"
        "    else if (NdotL > 0.0) intensity = 0.5;\n"
        "    else                  intensity = 0.25;\n"
        "    vec4 texColor = texture(texture0, fragTexCoord);\n"
        "    vec4 base = texColor * colDiffuse;\n"
        "    finalColor = vec4(base.rgb * intensity, base.a);\n"
        "}\n";

    Shader celShader = LoadShaderFromMemory(celVS, celFS);

    // -------------------------------------------------------------------------
    //  MODELS  (unchanged from Step 0)
    // -------------------------------------------------------------------------
    float wheelRadius = P_WHEEL_RADIUS;

    // 0=FL, 1=FR, 2=RL, 3=RR
    Model wheelModels[4] = {
        LoadModel("wheel_fr.obj"),
        LoadModel("wheel_fl.obj"),
        LoadModel("wheel_rr.obj"),
        LoadModel("wheel_rl.obj"),
    };
    for (int i = 0; i < 4; i++)
        for (int m = 0; m < wheelModels[i].materialCount; m++)
            wheelModels[i].materials[m].shader = celShader;

    Model carModel = LoadModel("car.obj");
    for (int m = 0; m < carModel.materialCount; m++)
        carModel.materials[m].shader = celShader;


    // -------------------------------------------------------------------------
    //  CAR DIMENSIONS  (unchanged from Step 0)
    // -------------------------------------------------------------------------
    float wheelBase  = 2.5f;
    float trackWidth = 1.6f;
    float frontZOffset = 0.35f;
    float rearZOffset  = 0.0f;

    Vector3 wheelLocal[4] = {
        { -trackWidth/2, wheelRadius,  wheelBase/2 + frontZOffset },  // FL
        {  trackWidth/2, wheelRadius,  wheelBase/2 + frontZOffset },  // FR
        { -trackWidth/2, wheelRadius, -wheelBase/2 - rearZOffset  },  // RL
        {  trackWidth/2, wheelRadius, -wheelBase/2 - rearZOffset  },  // RR
    };

    // =========================================================================
    //  BULLET WORLD SETUP
    // =========================================================================
    gCollCfg    = new btDefaultCollisionConfiguration();
    gDispatcher = new btCollisionDispatcher(gCollCfg);
    gBroadphase = new btDbvtBroadphase();
    gSolver     = new btSequentialImpulseConstraintSolver();
    gWorld      = new btDiscreteDynamicsWorld(gDispatcher, gBroadphase,
                                              gSolver, gCollCfg);
    gWorld->setGravity(btVector3(0, -20.0f, 0));

    // ---- Static ground plane ----
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0), 0);
    btTransform groundTrans;
    groundTrans.setIdentity();
    btRigidBody* ground = MakeRigidBody(0.0f, groundTrans, groundShape);
    ground->setFriction(0.85f);

    // ---- Chassis: simple box (stable, no ground penetration) ----
    // Half-extents: x=0.85 (half-width), y=0.35 (half-height), z=2.1 (half-length)
    // Bottom of box = origin.y - 0.35.  Origin.y = 0.80 → box bottom = 0.45m above ground.
    // Suspension raycasts reach down to ~y=0, so wheels always touch ground.
    btBoxShape* chassisShape = new btBoxShape(btVector3(0.85f, 0.35f, 2.1f));

    btTransform startTrans;
    startTrans.setIdentity();
    startTrans.setOrigin(btVector3(0, 0.80f, 0));

    btRigidBody* chassis = MakeRigidBody(P_CHASSIS_MASS, startTrans, chassisShape);
    chassis->setActivationState(DISABLE_DEACTIVATION);
    chassis->setDamping(P_LINEAR_DAMP, P_ANGULAR_DAMP);


    // ---- btRaycastVehicle ----
    btVehicleRaycaster* raycaster = new btDefaultVehicleRaycaster(gWorld);

    btRaycastVehicle::btVehicleTuning tuning;
    tuning.m_suspensionStiffness    = P_SUSP_STIFFNESS;
    tuning.m_suspensionDamping      = P_SUSP_DAMPING;
    tuning.m_suspensionCompression  = P_SUSP_COMPRESS;
    tuning.m_maxSuspensionTravelCm  = P_SUSP_TRAVEL_CM;
    tuning.m_frictionSlip           = P_FRONT_FRICTION;
    tuning.m_maxSuspensionForce     = P_SUSP_MAX_FORCE;

    btRaycastVehicle* vehicle = new btRaycastVehicle(tuning, chassis, raycaster);
    // Bullet coordinate system: right=X, up=Y, forward=Z
    vehicle->setCoordinateSystem(0, 1, 2);
    gWorld->addVehicle(vehicle);

    // ---- Add 4 wheels ----
    // Connection points in LOCAL chassis space.
    // Our chassis half-extents: x=0.85, y=0.35, z=2.1
    // Wheels connect at the 4 corners, slightly inside the edge.
    const btVector3 wheelDir(0, -1, 0);    // suspension casts downward
    const btVector3 axleDir (-1, 0, 0);    // axle points left
    const float     suspLen  = P_SUSP_LENGTH;

    // FL=0, FR=1, RL=2, RR=3
    const btVector3 connPts[4] = {
        btVector3(-0.85f, 0.0f,  1.55f),  // FL
        btVector3( 0.85f, 0.0f,  1.55f),  // FR
        btVector3(-0.85f, 0.0f, -1.55f),  // RL
        btVector3( 0.85f, 0.0f, -1.55f),  // RR
    };
    const bool isFront[4] = { true, true, false, false };

    for (int i = 0; i < 4; i++) {
        vehicle->addWheel(connPts[i], wheelDir, axleDir,
                          suspLen, wheelRadius, tuning, isFront[i]);
        btWheelInfo& wi        = vehicle->getWheelInfo(i);
        wi.m_rollInfluence     = P_ROLL_INFLUENCE;
        if (!isFront[i])
            wi.m_frictionSlip  = P_REAR_FRICTION;
    }

    // =========================================================================
    //  OBSTACLES — commented out for car physics testing
    //  Uncomment when ready to add collision testing back.
    // =========================================================================
    static const int NUM_OBS = 0;     // set to 3 and uncomment below to re-enable
    // Obstacle gObs[3];
    // gObs[0] = MakeObstacle("bump1.obj", celShader,
    //     btVector3(1.715f, 4.815f, 1.715f), {0.0f, 4.815f, 15.587f},
    //     1.0f, {220, 80, 40, 255});
    // gObs[1] = MakeObstacle("bump2.obj", celShader,
    //     btVector3(0.204f, 0.204f, 1.0f), {1.039f, -0.108f, -6.458f},
    //     1.0f, {255, 200, 50, 255});
    // gObs[2] = MakeObstacle("bump3.obj", celShader,
    //     btVector3(0.204f, 0.204f, 1.0f), {-7.225f, -0.074f, -9.358f},
    //     1.0f, {80, 180, 255, 255});

    // =========================================================================
    //  PHYSICS INPUT STATE
    // =========================================================================
    // Step 3 tuning constants
    float maxSteer      = P_MAX_STEER;
    float steerInRate   = P_STEER_IN_RATE;
    float steerOutRate  = P_STEER_OUT_RATE;
    float steerTarget   = 0.0f;
    float steerSmoothed = 0.0f;

    float engineTarget   = 0.0f;
    float engineSmoothed = 0.0f;
    float engineInRate   = P_ENGINE_IN_RATE;
    float engineOutRate  = P_ENGINE_OUT_RATE;

    const float MAX_ENGINE = P_ENGINE_FORCE;
    const float MAX_BRAKE  = P_BRAKE_FORCE;


    // =========================================================================
    //  STATE READ-BACK  (will be filled from Bullet each frame)
    // =========================================================================
    Vector3 carPos   = { 0, 0, 0 };
    float   heading  = 0.0f;    // yaw angle extracted from chassis matrix
    float   wheelSpin    = 0.0f;  // front wheels: from Bullet m_rotation
    float   rearWheelSpin = 0.0f; // rear wheels: engine-RPM-driven (shows spin even at standstill)

    // =========================================================================
    //  CAMERA  (unchanged from Step 0)
    // =========================================================================
    float camYaw    = 0.0f;
    float camPitch  = 25.0f;
    float camDist   = 12.0f;
    float camDistTgt= 12.0f;

    // =========================================================================
    //  GAME LOOP
    // =========================================================================
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;   // safety clamp
        bool  launching   = false;     // set in launch section, read in readback
        float launchT     = 1.0f;
        bool  throttleOn  = false;     // W held going forward — hoisted for drift section

        // =====================================================================
        //  STEP 3: INPUT SMOOTHING + SPEED-SENSITIVE STEERING
        // =====================================================================
        float curSpeedKmh = vehicle->getCurrentSpeedKmHour();
        float absSpeedKmh = fabsf(curSpeedKmh);

        // --- Speed-sensitive max steer ---
        // At 0 km/h  → full 35°
        // At 120 km/h → reduced to 12° (NFS-style)
        float speedFactor  = 1.0f - P_STEER_SPEED_REDUCE * (absSpeedKmh / 120.0f);
        if (speedFactor < 0.23f) speedFactor = 0.23f;
        float effectiveMax = maxSteer * speedFactor;

        // --- Raw steer target from key press ---
        if (IsKeyDown(KEY_A))       steerTarget =  effectiveMax;
        else if (IsKeyDown(KEY_D))  steerTarget = -effectiveMax;
        else                        steerTarget =  0.0f;

        // --- Lerp steer toward target ---
        // Turn in: slower  |  Return to 0: 2× faster (feels snappy on release)
        float steerRate = (fabsf(steerTarget) > 0.001f) ? steerInRate : steerOutRate;
        steerSmoothed += (steerTarget - steerSmoothed) * steerRate * dt;

        // Clamp to effective max
        if (steerSmoothed >  effectiveMax) steerSmoothed =  effectiveMax;
        if (steerSmoothed < -effectiveMax) steerSmoothed = -effectiveMax;

        // Apply to front wheels
        vehicle->setSteeringValue(steerSmoothed, 0);
        vehicle->setSteeringValue(steerSmoothed, 1);

        // =====================================================================
        //  ENGINE + BRAKE (with force smoothing)
        // =====================================================================
        float brakeForce = 0.0f;

        if (IsKeyDown(KEY_W)) {
            if (curSpeedKmh < -2.0f) { brakeForce = MAX_BRAKE; engineTarget = 0.0f; }
            else                       engineTarget = MAX_ENGINE;
        }
        else if (IsKeyDown(KEY_S)) {
            if (curSpeedKmh > 2.0f)  { brakeForce = MAX_BRAKE; engineTarget = 0.0f; }
            else                       engineTarget = -MAX_ENGINE * 0.5f;
        }
        else {
            engineTarget = 0.0f;  // coast

            // Idle braking: simulates rolling resistance + engine braking.
            // Heavier when nearly stopped (prevents infinite creep).
            // Light at speed (natural coast-down feel).
            if (absSpeedKmh < 3.0f)
                brakeForce = MAX_BRAKE * 0.20f;  // gentle hold when nearly stopped
            else
                brakeForce = MAX_BRAKE * 0.04f;  // light engine-brake at speed
        }

        // Lerp engine force; snap to zero to kill floating-point residual
        float engRate = (fabsf(engineTarget) > 0.1f) ? engineInRate : engineOutRate;
        engineSmoothed += (engineTarget - engineSmoothed) * engRate * dt;
        if (fabsf(engineSmoothed) < 1.0f) engineSmoothed = 0.0f;

        // LAUNCH: bypass the smooth ramp at low speed.
        // At <10 km/h with throttle held, hit the rear with full force INSTANTLY.
        // This causes wheel spin — the car bogs then grips, instead of just rolling.
        if (IsKeyDown(KEY_W) && curSpeedKmh >= -2.0f && absSpeedKmh < 10.0f) {
            engineSmoothed = P_ENGINE_FORCE;  // no lerp — full torque NOW
        }

        // RWD: engine to rear wheels only
        vehicle->applyEngineForce(engineSmoothed, 2);
        vehicle->applyEngineForce(engineSmoothed, 3);

        // Brakes on all 4 wheels
        vehicle->setBrake(brakeForce, 0);
        vehicle->setBrake(brakeForce, 1);
        vehicle->setBrake(brakeForce, 2);
        vehicle->setBrake(brakeForce, 3);

        // =====================================================================
        //  ARCADE DRIFT SYSTEM — research-based fake drift
        //  Methods: slip angle α, state machine, imaginary wall (momentum
        //  preservation), weight transfer, vector dot-product scoring.
        // =====================================================================

        // --- Velocity decomposition in car-local space ---
        btTransform driftTrans;
        chassis->getMotionState()->getWorldTransform(driftTrans);
        btVector3 vel        = chassis->getLinearVelocity();
        btVector3 fwdWorld   = driftTrans.getBasis().getColumn(2);  // car +Z
        btVector3 rightWorld = driftTrans.getBasis().getColumn(0);  // car +X
        float     fwdSpd     = vel.dot(fwdWorld);
        float     latSpd     = vel.dot(rightWorld);  // +ve = sliding right
        float     totalSpd   = btSqrt(vel.x()*vel.x() + vel.z()*vel.z());

        // 1. Slip angle α = atan2(lateral, forward) in degrees
        //    0° = straight tracking, 45° = drifting, 90° = fully broadside
        float slipAngle = (fabsf(fwdSpd) > 0.5f)
            ? fabsf(atan2f(fabsf(latSpd), fabsf(fwdSpd))) * RAD2DEG
            : 0.0f;

        // 2. Drift score via vector dot product (research formula)
        //    score = 1 - |velDir · carFwd|
        //    0 = perfect tracking, 1 = fully sideways
        btVector3 velDir   = (totalSpd > 0.5f) ? vel / totalSpd : fwdWorld;
        float     driftScore = 1.0f - fabsf(velDir.dot(fwdWorld));

        // Keep driftRatio for friction modulation (0-1 sideways fraction)
        float driftRatio = (totalSpd > 1.0f) ? fabsf(latSpd) / totalSpd : 0.0f;

        // 3. Drift state machine with hysteresis
        //    Entry: α > P_DRIFT_ANGLE_ENTRY (default 8°)
        //    Exit:  α < P_DRIFT_ANGLE_EXIT  (default 4°) after hold timer
        //    Hysteresis prevents rapid mode flicker at the boundary.
        static bool  driftActive    = false;
        static float driftHoldTimer = 0.0f;
        if (slipAngle > P_DRIFT_ANGLE_ENTRY && totalSpd > 5.0f) {
            driftActive    = true;
            driftHoldTimer = 0.3f;   // hold for 300ms
        } else {
            driftHoldTimer -= dt;
            if (driftHoldTimer <= 0.0f && slipAngle < P_DRIFT_ANGLE_EXIT)
                driftActive = false;
        }
        bool isDrifting = driftActive;

        // --- Handbrake ---
        bool handbrake = IsKeyDown(KEY_SPACE);

        // Speed-dependent handbrake mode:
        //   HIGH SPEED (> P_HB_SPEED_THRESH): REAL handbrake drift entry
        //     - Rear wheels LOCK (heavy brake) → rear slides freely
        //     - Engine fights the lock → rear wheel spin against ground
        //     - Front wheels FREE (no brake) → front steers the arc
        //     - Combined: W=rear spins, SPACE=rear locked/slipping, A=front steers = DRIFT
        //
        //   LOW SPEED (<= P_HB_SPEED_THRESH): launch feel
        //     - Light front brake only (weight transfer feel)
        bool highSpdHB = handbrake && absSpeedKmh > P_HB_SPEED_THRESH;
        bool lowSpdHB  = handbrake && absSpeedKmh <= P_HB_SPEED_THRESH;

        // ── Compute throttleOn / launching BEFORE spin-omega model ──────────
        // (These were hoisted as false at loop top; set their real values here
        //  so the spin-omega model below can use them correctly.)
        throttleOn = IsKeyDown(KEY_W) && curSpeedKmh > -2.0f;
        launching  = throttleOn && absSpeedKmh < P_LAUNCH_SPEED_KMH
                     && !highSpdHB && !isDrifting;
        launchT    = launching ? (absSpeedKmh / P_LAUNCH_SPEED_KMH) : 1.0f;

        // ─────────────────────────────────────────────────────────────────────
        //  SPIN-OMEGA FRICTION MODEL — replaces all separate friction lerps
        //  rearSpinOmega = actual rear wheel angular velocity (rad/s)
        //  carOmega      = what wheels SHOULD spin at for pure rolling contact
        //  slip          = rearSpinOmega - carOmega  (excess spin)
        //  slipRatio     = slip / refOmega  (0=grip, 1=full spin)
        //  friction      = linear interp P_REAR_FRICTION↔P_LAUNCH_SPIN_FRIC
        //
        //  KEY PROPERTY: omega decays slow (inertia) so releasing SPACE causes
        //  gradual 1-2s friction recovery, not an instant snap back to grip.
        // ─────────────────────────────────────────────────────────────────────
        static float rearSpinOmega = 0.0f;  // persists between frames
        static float rearFriction  = P_REAR_FRICTION;
        float carOmegas = (totalSpd > 0.1f) ? (totalSpd / P_WHEEL_RADIUS) : 0.0f;
        float refOmega  = (P_LAUNCH_SPEED_KMH / 3.6f) / P_WHEEL_RADIUS;  // reference max spin

        float targetRearOmega, omegaUp, omegaDown;
        if (highSpdHB) {
            // Real handbrake: rear braked/locked → wheels decelerate
            targetRearOmega = 0.0f;
            omegaUp = 1.0f; omegaDown = 10.0f;  // fast stop
        } else if (launching && handbrake) {
            // W + SPACE burnout: max spin, very slow organic decay
            targetRearOmega = refOmega * (1.5f + (1.0f - launchT) * 3.0f);
            omegaUp = 14.0f; omegaDown = 0.8f;  // quick spin-up, VERy slow spin-down
        } else if (launching && !handbrake) {
            // W only (no SPACE): normal rolling — wheels track car speed exactly
            // No visual over-spin without SPACE; spinOffset stays at 0; no elastic snap
            targetRearOmega = carOmegas;
            omegaUp = 8.0f; omegaDown = 8.0f;
        } else if (isDrifting && throttleOn) {
            // Sustained drift: spin proportional to sideways angle
            targetRearOmega = carOmegas * (1.0f + driftRatio * 1.5f);
            omegaUp = 5.0f; omegaDown = 2.0f;
        } else {
            // No throttle (S pressed, coasting, car stopped):
            // Decay quickly to car rolling speed — no reason to over-spin with no engine
            targetRearOmega = carOmegas;
            omegaUp = 3.0f; omegaDown = 8.0f;  // fast return to rolling; ~0.2s to stop
        }

        float omegaRate = (targetRearOmega > rearSpinOmega) ? omegaUp : omegaDown;
        rearSpinOmega  += (targetRearOmega - rearSpinOmega) * omegaRate * dt;
        if (rearSpinOmega < 0.0f) rearSpinOmega = 0.0f;

        // Friction derived from slip ratio (physical relationship)
        // Over-spin: wheel spinning faster than car → tire slip → low friction
        float slip      = fmaxf(0.0f, rearSpinOmega - carOmegas);  // excess spin
        float slipRatio = fminf(1.0f, slip / (refOmega + 0.01f));  // 0=grip, 1=full slip
        rearFriction    = P_REAR_FRICTION    * (1.0f - slipRatio)
                        + P_LAUNCH_SPIN_FRIC * slipRatio;

        // HIGH-SPEED HANDBRAKE OVERRIDE
        // Slip-ratio above only handles OVER-spin (wheel > car speed).
        // Locked rear wheel = UNDER-spin (wheel=0, car moving forward).
        // slip = max(0, 0 - carOmega) = 0  →  slipRatio=0  →  rearFriction=2.8 (WRONG!)
        // Fix: directly set the low slide friction needed for the rear to actually slide.
        if (highSpdHB) rearFriction = P_HB_HIGH_SPD_FRIC;

        vehicle->getWheelInfo(2).m_frictionSlip = rearFriction;
        vehicle->getWheelInfo(3).m_frictionSlip = rearFriction;

        if (highSpdHB) {
            // REAL HANDBRAKE: lock rear, free front
            // Rear slows/locks → rear slides when yaw begins
            // Engine (W) overpowers partially → rear wheel spin against locked brake
            // Front free → A/D steers the arc cleanly
            vehicle->setBrake(MAX_BRAKE * 2.8f, 2);  // lock rear-left
            vehicle->setBrake(MAX_BRAKE * 2.8f, 3);  // lock rear-right
            vehicle->setBrake(0.0f, 0);               // front-left: totally free
            vehicle->setBrake(0.0f, 1);               // front-right: totally free
        } else if (lowSpdHB) {
            // Low-speed: light front brake (weight transfer for launch)
            vehicle->setBrake(MAX_BRAKE * 0.6f, 0);
            vehicle->setBrake(MAX_BRAKE * 0.6f, 1);
        }

        // 4. Weight Transfer (research: W_r decreases during braking)
        //    Braking shifts load forward → rear axle unloads → grip drops
        //    Simplified: reduce rear friction proportional to brake input
        float brakeNorm   = brakeForce / P_BRAKE_FORCE;
        float weightShift = brakeNorm * P_WEIGHT_TRANSFER;
        float wtMult      = 1.0f - weightShift * 0.28f;  // max -28% rear grip on full brake
        vehicle->getWheelInfo(2).m_frictionSlip *= wtMult;
        vehicle->getWheelInfo(3).m_frictionSlip *= wtMult;

        // 5. Imaginary Wall — momentum preservation
        //    Resists rapid lateral velocity CHANGES so the car sweeps a large
        //    arc (stadium-style drift) instead of spinning in tight circles.
        //    Force = -P_DRIFT_MOMENTUM × m × latSpd² (opposes lateral accel)
        if (isDrifting && totalSpd > 5.0f) {
            float wallMag  = P_DRIFT_MOMENTUM * P_CHASSIS_MASS
                             * latSpd * fabsf(latSpd) * 0.001f;
            chassis->applyCentralForce(rightWorld * (-wallMag));
        }

        // 6. Yaw rate cap — relaxed during high-speed handbrake for fast drift initiation
        {
            btVector3 angVel      = chassis->getAngularVelocity();
            float     yawRate     = angVel.y();
            // Allow faster rotation when initiating with handbrake at speed
            float     yawCap      = highSpdHB ? P_MAX_YAW_RATE * 1.5f : P_MAX_YAW_RATE;
            if (fabsf(yawRate) > yawCap) {
                float clamped = yawCap * (yawRate > 0 ? 1.0f : -1.0f);
                chassis->setAngularVelocity(btVector3(angVel.x(), clamped, angVel.z()));
            }
        }

        // 7. Countersteer assist
        if (isDrifting && fabsf(steerSmoothed) > 0.05f) {
            float steerDotSlide = steerSmoothed * latSpd;
            if (steerDotSlide > 0.0f) {
                chassis->applyTorque(
                    btVector3(0, -latSpd * driftRatio * P_COUNTERSTEER, 0));
            }
        }

        // 8. Power Oversteer — rear wheels push car along heading during drift
        //
        //  Reality: when rear tires slip, they spin rapidly and push the car in
        //  the direction the rear axle is FACING (car heading), not where car
        //  travels. Steering angle makes front wheels form a pivot, so the combo
        //  of heading-thrust + steered fronts = proper large-radius drift arc.
        //
        //  Force = throttle% × driftDepth × P_DRIFT_POWER_FORCE
        //         applied along fwdWorld (car nose direction)
        if (isDrifting && throttleOn && fabsf(engineSmoothed) > 50.0f) {
            float throttleFraction = fabsf(engineSmoothed) / P_ENGINE_FORCE;   // 0-1
            float powerForce       = throttleFraction * driftRatio * P_DRIFT_POWER_FORCE;
            chassis->applyCentralForce(fwdWorld * powerForce);
        }

        // =====================================================================
        //  RWD LAUNCH / WHEEL SPIN
        //  Runs AFTER the drift friction system so it can override friction.
        //  Three-part system:
        //   A) Ultra-low friction (0.18) → rear tires actually SLIP in Bullet
        //   B) Bog force → backward impulse keeps car mostly in place during spin
        //   C) rearWheelSpin → visual spin driven by engine RPM (NOT car speed)
        // =====================================================================
        // throttleOn / launching / launchT computed above (before spin-omega model)
        static bool prevThrottle  = false;
        bool        freshThrottle = throttleOn && !prevThrottle && absSpeedKmh < 5.0f;
        prevThrottle = throttleOn;

        // A) Torque-steer kick on fresh throttle
        if (freshThrottle) {
            float kickDir = (GetRandomValue(0, 1) == 0) ? 1.0f : -1.0f;
            chassis->applyTorqueImpulse(btVector3(0, kickDir * P_LAUNCH_KICK, 0));
        }

        // Friction ramp handled by spin-omega model above (no separate override needed)

        // B) Bog force: only when handbrake held + tires spinning
        //    Without SPACE: car should move forward freely when steering
        //    With SPACE: bog keeps car in place during straight burnout
        if (launching && handbrake) {
            float steerFraction = fabsf(steerSmoothed) / (P_MAX_STEER * 0.6f);
            if (steerFraction > 1.0f) steerFraction = 1.0f;
            float bogScale = 1.0f - steerFraction;
            float bogMag   = P_LAUNCH_BOG_FORCE * (1.0f - launchT) * bogScale;
            chassis->applyCentralForce(fwdWorld * (-bogMag));
        }

        // C) UNIVERSAL ARCADE LATERAL FORCE — W + steer at ANY state/speed
        //    SPACE + steer → full donut force
        //    No SPACE, low speed → 45% force (car arcs gently, moves forward)
        //    No SPACE, high speed → tapers further with speed
        //
        //    FORCE DIRECTION: blended lateral + forward component
        //    Pure sideways (90°) = spinning in circles (bad)
        //    Diagonal (≈68° from forward) = rear pushed forward AND sideways = natural arc
        //    Blend: 100% lateral + 40% forward = car moves forward while arcing
        if (throttleOn && fabsf(steerSmoothed) > 0.05f) {
            float steerNorm    = steerSmoothed / P_MAX_STEER;           // -1 to +1
            float throttleFrac = fabsf(engineSmoothed) / P_ENGINE_FORCE; // 0-1

            float speedScale;
            if (handbrake) {
                speedScale = 1.0f;                              // SPACE: full force always
            } else if (absSpeedKmh < P_HB_SPEED_THRESH) {
                speedScale = 0.45f;                             // no SPACE, low speed: 45%
            } else {
                speedScale = 1.0f / (1.0f + absSpeedKmh * 0.04f); // no SPACE, high: taper
            }

            float lateral  = -steerNorm * throttleFrac * P_DONUT_FORCE * speedScale;
            float forward  = fabsf(lateral) * 0.40f;           // 40% forward = arc feel

            // Diagonal force at rear: lateral (sideways) + forward component
            // Result: rear steps out sideways AND car keeps moving forward → natural arc
            btVector3 slideForce = rightWorld * lateral + fwdWorld * forward;
            btVector3 rearRelPos = fwdWorld * (-1.3f);
            chassis->applyForce(slideForce, rearRelPos);
        }


        // =====================================================================
        //  R KEY: RESET CAR UPRIGHT
        //  Snaps the car back to upright orientation at its current XZ position.
        //  Essential escape hatch when the car is stuck flipped.
        // =====================================================================
        if (IsKeyPressed(KEY_R)) {
            btTransform cur;
            chassis->getMotionState()->getWorldTransform(cur);
            btVector3 pos = cur.getOrigin();

            btTransform upright;
            upright.setIdentity();
            upright.setOrigin(btVector3(pos.x(), 0.80f, pos.z()));

            chassis->setWorldTransform(upright);
            chassis->getMotionState()->setWorldTransform(upright);
            chassis->setLinearVelocity (btVector3(0, 0, 0));
            chassis->setAngularVelocity(btVector3(0, 0, 0));
            chassis->activate(true);
            engineSmoothed = 0.0f;
        }

        // =====================================================================
        //  ACTIVE ANTI-ROLL TORQUE
        //  Reads the car's current up vector and applies a corrective torque
        //  back toward world-up every frame. Proportional to tilt angle so it
        //  barely fires when the car is level and ramps up hard as it tips.
        // =====================================================================
        {
            btTransform cur;
            chassis->getMotionState()->getWorldTransform(cur);
            btVector3 carUp    = cur.getBasis().getColumn(1); // local Y in world space
            btVector3 worldUp  = btVector3(0, 1, 0);
            float     dotUp    = carUp.dot(worldUp);          // 1=flat, 0=sideways, -1=upside-down

            if (dotUp < 0.85f) {     // only fires when tilting > ~32° — ignores micro-tilts
                btVector3 corrAxis = carUp.cross(worldUp);
                float     strength = (1.0f - dotUp) * 4000.0f;
                chassis->applyTorque(corrAxis * strength);
            }
        }

        // =====================================================================
        //  BULLET SIMULATION STEP
        // =====================================================================
        gWorld->stepSimulation(dt, 10, 1.0f / 120.0f);

        // =====================================================================
        //  READ-BACK: extract carPos, heading, wheelSpin from Bullet
        // =====================================================================
        btTransform chassisTrans;
        chassis->getMotionState()->getWorldTransform(chassisTrans);

        btVector3 origin = chassisTrans.getOrigin();
        // We use the GROUND-LEVEL x/z; for y we subtract chassis half-extent
        // so the car renders at the same visual height as Step 0.
        carPos = {
            (float)origin.x(),
            0.0f,               // drive the render as if flat — suspension bob is subtle
            (float)origin.z()
        };

        // Extract yaw heading: forward axis in world space (+Z local → world)
        btVector3 fwdBt = chassisTrans.getBasis() * btVector3(0, 0, 1);
        heading = atan2f((float)fwdBt.x(), (float)fwdBt.z());

        // WheelSpin readback:
        //  Front wheels: use Bullet's actual m_rotation (rolls with car speed)
        //  Rear wheels:  custom accumulator driven by engine RPM so rear visually
        //                spins fast even when the car is stationary (wheel spin phase)
        vehicle->updateWheelTransform(2, true);
        wheelSpin = (float)vehicle->getWheelInfo(2).m_rotation;  // front wheels

        // Rear wheel visual: pure velocity-based accumulation — no elastic snap possible
        //
        //  rearWheelVel = smooth angular velocity, always forward during burnout
        //
        //  Burnout active:   targetVel = rearSpinOmega (large) → spin-up at 14/s  (fast)
        //  Everything else:  targetVel = signedCarOmega      → decel/match at 3.5/s (gentle)
        //
        //  rearWheelSpin is ONLY ever changed by += rearWheelVel * dt
        //  Never direct-assigned → backward motion is IMPOSSIBLE → no elastic snap
        //
        //  W+SPACE → release: rearWheelVel decelerates from 24→0 rad/s over ~1.5s,
        //                     always moving forward, just slower and slower. No snap.
        //  W only → release:  rearWheelVel tracks signedCarOmega, both reach 0. No snap.
        //  S → release:       rearWheelVel was negative, gently returns to 0. No snap.
        {
            static float rearWheelVel = 0.0f;

            float signedCarOmega = curSpeedKmh / 3.6f / P_WHEEL_RADIUS; // +fwd, -rev
            float extraOmega     = fmaxf(0.0f, rearSpinOmega - carOmegas);

            float targetVel;
            float velRate;

            if (extraOmega > 1.5f) {
                // Burnout/drift: fast forward spin-up
                targetVel = rearSpinOmega;
                velRate   = 14.0f;
            } else if (fabsf(curSpeedKmh) < 1.0f) {
                // Near-stop zone: smoothly zero out regardless of previous direction
                // Prevents S-release snap: rearWheelVel was negative, car nearly stopped,
                // target=0 reached at 10/s → wheels just stop, no forward flip
                targetVel = 0.0f;
                velRate   = 10.0f;
            } else {
                // Normal rolling: gently follow signed car speed
                targetVel = signedCarOmega;
                velRate   = 3.5f;
            }

            rearWheelVel += (targetVel - rearWheelVel) * velRate * dt;

            rearWheelSpin += rearWheelVel * dt;  // always accumulate — no direct assign, no snap
        }

        // =====================================================================
        //  CAMERA  (unchanged from Step 0)
        // =====================================================================
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 md = GetMouseDelta();
            camYaw   -= md.x * 0.3f;
            camPitch += md.y * 0.3f;
        }
        if (camPitch < 5)  camPitch = 5;
        if (camPitch > 80) camPitch = 80;
        camDistTgt -= GetMouseWheelMove() * 2.0f;
        if (camDistTgt < 3)  camDistTgt = 3;
        if (camDistTgt > 40) camDistTgt = 40;
        camDist += (camDistTgt - camDist) * 8.0f * dt;

        Vector3 camTgt = { carPos.x, 0.5f, carPos.z };
        float yr = camYaw * DEG2RAD, pr = camPitch * DEG2RAD;
        Camera3D camera = {0};
        camera.position = {
            camTgt.x + camDist * cosf(pr) * sinf(yr),
            camTgt.y + camDist * sinf(pr),
            camTgt.z + camDist * cosf(pr) * cosf(yr)
        };
        camera.target     = camTgt;
        camera.up         = { 0, 1, 0 };
        camera.fovy       = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        // =====================================================================
        //  DRAW  (unchanged from Step 0)
        // =====================================================================
        BeginDrawing();
            ClearBackground(BLACK);

            int hH = GetScreenHeight() / 2;
            DrawRectangleGradientV(0,  0, GetScreenWidth(), hH, {25,25,112,255}, {135,206,235,255});
            DrawRectangleGradientV(0, hH, GetScreenWidth(), hH, {135,206,235,255}, {255,200,150,255});

            BeginMode3D(camera);
                DrawPlane({0,0,0}, {200,200}, {50,50,50,255});
                DrawGrid(100, 1.0f);

                // =====================================================================
                //  BODY DYNAMICS: suspension bob + lateral roll + engine vibration
                //  Reads actual per-wheel suspension compression from Bullet each frame.
                // =====================================================================

                // 1. Average suspension compression across all 4 wheels
                float avgSuspComp = 0.0f;
                for (int wi = 0; wi < 4; wi++) {
                    vehicle->updateWheelTransform(wi, true);
                    const btWheelInfo& w = vehicle->getWheelInfo(wi);
                    // compression ratio: 0 = fully extended, 1 = fully compressed
                    float comp = 1.0f - (w.m_raycastInfo.m_suspensionLength / P_SUSP_LENGTH);
                    if (comp < 0) comp = 0;
                    avgSuspComp += comp;
                }
                avgSuspComp /= 4.0f;
                float bodyBob = avgSuspComp * P_SUSP_BOB_SCALE * P_SUSP_LENGTH;

                // 2. Engine vibration: scales with speed
                float vibration = P_VIB_AMP
                    * (fabsf(curSpeedKmh) / 80.0f)
                    * sinf((float)GetTime() * P_VIB_FREQ * 6.2832f);

                // 3. Body roll: lean into corners based on lateral g-force
                //    Clamped to ±10° so it never looks broken
                float bodyRoll = -latSpd * P_BODY_ROLL * 0.05f;
                if (bodyRoll >  0.17f) bodyRoll =  0.17f;
                if (bodyRoll < -0.17f) bodyRoll = -0.17f;

                // Build car matrix: roll → yaw → translate
                Matrix carMat = MatrixMultiply(
                    MatrixMultiply(
                        MatrixRotateZ(bodyRoll),
                        MatrixRotateY(heading)
                    ),
                    MatrixTranslate(carPos.x, bodyBob + vibration, carPos.z)
                );

                // --- Wheels: front use wheelSpin (Bullet), rear use rearWheelSpin ---
                for (int i = 0; i < 4; i++) {
                    bool front = (i < 2);
                    // Front: Bullet's actual rotation | Rear: custom RPM-driven spin
                    Matrix m = MatrixRotateX(front ? wheelSpin : rearWheelSpin);
                    if (front) m = MatrixMultiply(m, MatrixRotateY(steerSmoothed));
                    m = MatrixMultiply(m, MatrixTranslate(
                        wheelLocal[i].x, wheelLocal[i].y, wheelLocal[i].z));
                    m = MatrixMultiply(m, carMat);
                    wheelModels[i].transform = m;
                    DrawModel(wheelModels[i], {0,0,0}, 1.0f, WHITE);
                }

                // --- Chassis (same as Step 0) ---
                Matrix chassisMat = MatrixMultiply(MatrixTranslate(0, 0, -0.1f), carMat);
                carModel.transform = chassisMat;
                DrawModel(carModel, {0,0,0}, 1.0f, WHITE);

                // --- Obstacles: disabled for car physics testing ---
                // for (int i = 0; i < NUM_OBS; i++)
                //     DrawModel(gObs[i].model, {0,0,0}, 1.0f, gObs[i].tint);


            EndMode3D();

            // --- HUD ---
            DrawText("Arcade Drift Engine", 10, 10, 18, WHITE);
            DrawText(TextFormat("Speed     : %5.1f km/h",  fabsf(curSpeedKmh)), 10, 36, 16, GREEN);

            // Drift score bar (research: 1 - velDir.carFwd)
            const char* scoreLabel = TextFormat("Drift Score: %3.0f%%", driftScore * 100.0f);
            DrawText(scoreLabel, 10, 58, 16,
                isDrifting ? Color{255,80,30,255} : Color{160,160,160,255});
            // Draw score bar
            DrawRectangle(160, 60, 120, 14, {40,40,40,200});
            DrawRectangle(160, 60, (int)(driftScore * 120), 14,
                isDrifting ? Color{255,80,30,220} : Color{100,200,100,180});

            DrawText(TextFormat("Slip Angle: %5.1f deg  %s",
                slipAngle,
                isDrifting ? "<< DRIFT >>" : "grip"),
                10, 78, 15,
                isDrifting ? Color{255,160,30,255} : Color{140,140,140,255});

            DrawText(TextFormat("Rear Grip : %.2f  HB=%s",
                rearFriction, handbrake ? "ON" : "off"),
                10, 98, 15,
                handbrake ? Color{255,50,50,255} : Color{160,180,255,255});

            DrawText("SPACE: Handbrake  |  R: Reset", 10, 118, 14, {150,150,150,255});


            int y = GetScreenHeight() - 90;
            DrawText("W/S  : Throttle / Brake+Reverse", 10, y,    15, {180,180,180,255});
            DrawText("A/D  : Steer",                    10, y+18, 15, {180,180,180,255});
            DrawText("Mouse: Orbit camera",              10, y+36, 15, {180,180,180,255});
            DrawText("R    : Reset car upright",         10, y+54, 15, {255,100,100,255});
        EndDrawing();
    }

    // =========================================================================
    //  CLEANUP
    // =========================================================================
    // Bullet objects
    gWorld->removeVehicle(vehicle);
    delete vehicle;
    delete raycaster;

    // Cleanup obstacles (disabled — re-enable with the obstacle block above)
    // for (int i = 0; i < NUM_OBS; i++) {
    //     gWorld->removeRigidBody(gObs[i].body);
    //     if (gObs[i].body->getMotionState()) delete gObs[i].body->getMotionState();
    //     delete gObs[i].body;
    //     delete gObs[i].shape;
    //     UnloadModel(gObs[i].model);
    // }

    for (int i = gWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = gWorld->getCollisionObjectArray()[i];
        btRigidBody* rb = btRigidBody::upcast(obj);
        if (rb && rb->getMotionState()) delete rb->getMotionState();
        gWorld->removeCollisionObject(obj);
        delete obj;
    }
    delete chassisShape;
    delete groundShape;
    delete gWorld;
    delete gSolver;
    delete gBroadphase;
    delete gDispatcher;
    delete gCollCfg;

    // Raylib objects
    for (int i = 0; i < 4; i++) UnloadModel(wheelModels[i]);
    UnloadModel(carModel);
    UnloadShader(celShader);
    CloseWindow();
    return 0;
}
