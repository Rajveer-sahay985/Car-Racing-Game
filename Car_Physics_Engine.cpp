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
//  STATIC TRIMESH OBSTACLE
//  Uses btBvhTriangleMeshShape for exact slope collision.
//  Perfect for ramps — the car physically drives up the slope surface.
// =============================================================================
struct StaticTrimesh {
    Model                    model;
    btRigidBody*             body    = nullptr;
    btBvhTriangleMeshShape*  shape   = nullptr;
    btTriangleMesh*          triData = nullptr;  // owned here
    Vector3                  offset  = {0,0,0};  // applied to both visual + physics
    Color                    tint    = WHITE;
};

// Build a trimesh obstacle from an OBJ file.
// offset: world-space translation applied to the Bullet rigid body.
//         The visual model is drawn at the same offset via its transform matrix.
static StaticTrimesh MakeRampObstacle(const char* objFile, Shader sh,
                                       Vector3 offset, Color tint = WHITE)
{
    StaticTrimesh obs;
    obs.model  = LoadModel(objFile);
    obs.offset = offset;
    obs.tint   = tint;

    for (int m = 0; m < obs.model.materialCount; m++)
        obs.model.materials[m].shader = sh;

    // --- Build btTriangleMesh from Raylib vertex/index data ---
    obs.triData = new btTriangleMesh();

    for (int mi = 0; mi < obs.model.meshCount; mi++) {
        Mesh& mesh = obs.model.meshes[mi];
        float* vp  = mesh.vertices;  // packed XYZ per vertex

        if (mesh.indices) {
            // Indexed mesh (common after triangulation in Blender)
            for (int tri = 0; tri < mesh.triangleCount; tri++) {
                unsigned short i0 = mesh.indices[tri*3+0];
                unsigned short i1 = mesh.indices[tri*3+1];
                unsigned short i2 = mesh.indices[tri*3+2];
                btVector3 v0(vp[i0*3+0], vp[i0*3+1], vp[i0*3+2]);
                btVector3 v1(vp[i1*3+0], vp[i1*3+1], vp[i1*3+2]);
                btVector3 v2(vp[i2*3+0], vp[i2*3+1], vp[i2*3+2]);
                obs.triData->addTriangle(v0, v1, v2);        // front face
                obs.triData->addTriangle(v2, v1, v0);        // back face  (double-sided)
            }
        } else {
            // Non-indexed: every 3 vertices = 1 triangle
            for (int tri = 0; tri < mesh.triangleCount; tri++) {
                int b = tri * 9;
                btVector3 v0(vp[b+0], vp[b+1], vp[b+2]);
                btVector3 v1(vp[b+3], vp[b+4], vp[b+5]);
                btVector3 v2(vp[b+6], vp[b+7], vp[b+8]);
                obs.triData->addTriangle(v0, v1, v2);        // front face
                obs.triData->addTriangle(v2, v1, v0);        // back face  (double-sided)
            }
        }
    }

    // useQuantizedAabbCompression=true → faster queries, less memory
    obs.shape = new btBvhTriangleMeshShape(obs.triData, true);

    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(offset.x, offset.y, offset.z));

    obs.body = MakeRigidBody(0.0f, t, obs.shape);  // mass=0 → static
    obs.body->setFriction(0.85f);       // grippy surface
    obs.body->setRestitution(0.05f);    // low bounce on contact
    return obs;
}

// =============================================================================
//  MAIN
// =============================================================================
int main()
{
    InitWindow(1280, 720, "Vortex Racing — Bullet Physics");
    SetTargetFPS(60);
    InitAudioDevice();

    // =========================================================================
    // ██████████████████████████████████████████████████████████████████████
    //  TUNING PARAMETERS — change these freely to adjust feel!
    //  All physics, steering, drift and body-feel values live here.
    // ██████████████████████████████████████████████████████████████████████
    // =========================================================================

    // --- Engine / Brakes ---
    const float P_ENGINE_FORCE    = 3000.0f;  // N on rear wheels (higher = faster)
    const float P_BRAKE_FORCE     = 160.0f;   // N·m per wheel (higher = sharper max brake)
    const float P_ENGINE_IN_RATE  = 6.0f;     // throttle ramp speed
    const float P_ENGINE_OUT_RATE = 10.0f;    // coast ramp speed
    // Brake ramping rates — controls how fast brakes bite and release.
    // Fast bite (12/s): brakes feel responsive but not instantaneous (~1.5 frames to 95%).
    // Slow release (4/s): residual braking feel; car glides to rest rather than snapping.
    const float P_BRAKE_IN_RATE   = 12.0f;    // how fast brake force builds up (s⁻¹)
    const float P_BRAKE_OUT_RATE  = 4.0f;     // how fast brake force bleeds off (s⁻¹)

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
    // Issue #2: P_BODY_ROLL is now applied WITHOUT a hidden *0.05f factor.
    //   Old effective coefficient: 0.06 * 0.05 = 0.003 rad/(m/s) → ~1.7° at 8 m/s lateral.
    //   New effective coefficient: 0.012 rad/(m/s)              → ~5.5° at 8 m/s lateral.
    //   Clamped to ±0.18 rad (≈10°) so it never looks broken.
    const float P_BODY_ROLL       = 0.012f;  // lean amount in corners (rad per m/s lateral velocity)
    // Issue #2: P_BODY_PITCH controls nose-dive (braking) and rear-squat (acceleration).
    //   Suspension geometric pitch = (frontComp - rearComp) * suspLength / wheelBase * PITCH_SCALE
    //   At 0.15 compression diff: 0.15 * 0.45 / 2.5 * 2.5 = 0.068 rad ≈ 3.9°
    const float P_BODY_PITCH      = 2.5f;    // nose-dive / squat intensity (pitch scale factor)
    const float P_SUSP_BOB_SCALE  = 0.7f;   // suspension height bob multiplier (for chassis only)
    const float P_VIB_AMP         = 0.003f;  // engine vibration amplitude
    const float P_VIB_FREQ        = 14.0f;   // engine vibration frequency (Hz)
    // P_CHASSIS_Y_OFFSET: moves the whole visual car body up (+) or down (-).
    // Set negative to lower the chassis since bodyBob was raising it above the wheels.
    // Tune this until the car body sits flush over the wheel arches.
    const float P_CHASSIS_Y_OFFSET = -0.05f;

    // =========================================================================

    // =========================================================================
    //  ENGINE AUDIO — Pseudo-Granular Blender
    //  7 EXT samples: 1500, 2000, 3000, 4000, 5000, 6000, 7000 RPM.
    //  Each frame: find the two bracketing samples, set their volumes (crossfade)
    //  and pitch-shift them so they meet exactly at the current RPM.
    //  Press M to toggle audio on/off at any time.
    // =========================================================================
    constexpr int ENG_SAMPLE_COUNT = 7;
    const float   ENG_RPM_POINTS[ENG_SAMPLE_COUNT] = {
        1500.f, 2000.f, 3000.f, 4000.f, 5000.f, 6000.f, 7000.f
    };
    const char*   ENG_FILES[ENG_SAMPLE_COUNT] = {
        "Engine Sounds/Audi_1500_0_EXT.wav",
        "Engine Sounds/Audi_2000_1_EXT.wav",
        "Engine Sounds/Audi_3000_1_EXT.wav",
        "Engine Sounds/Audi_4000_1_EXT.wav",
        "Engine Sounds/Audi_5000_1_EXT.wav",
        "Engine Sounds/Audi_6000_1_EXT.wav",
        "Engine Sounds/Audi_7000_1_EXT.wav",
    };

    Sound engSounds[ENG_SAMPLE_COUNT];
    for (int i = 0; i < ENG_SAMPLE_COUNT; i++) {
        engSounds[i] = LoadSound(ENG_FILES[i]);
        SetSoundVolume(engSounds[i], 0.0f);  // start silent
        PlaySound(engSounds[i]);             // begin looping immediately (muted)
    }

    // M key: toggle engine audio on/off
    bool audioEnabled = true;

    // RPM model — derived from wheel angular velocity.
    // Idle at ~1500 RPM when stationary; peaks at 7000 RPM.
    float currentRPM        = 1500.0f;
    float rawRPM            = 1500.0f;
    const float RPM_IDLE    = 1500.0f;
    const float RPM_MAX     = 7000.0f;
    const float RPM_OMEGA_SCALE = 220.0f;

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
        LoadModel("Obj Files/wheel_fr.obj"),
        LoadModel("Obj Files/wheel_fl.obj"),
        LoadModel("Obj Files/wheel_rr.obj"),
        LoadModel("Obj Files/wheel_rl.obj"),
    };
    for (int i = 0; i < 4; i++)
        for (int m = 0; m < wheelModels[i].materialCount; m++)
            wheelModels[i].materials[m].shader = celShader;

    Model carModel = LoadModel("Obj Files/car.obj");
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
    //  OBSTACLES
    // =========================================================================
    //  ----- Old box obstacles (disabled for trimesh testing) -----
    static const int NUM_OBS = 0;
    // Obstacle gObs[3];
    // gObs[0] = MakeObstacle("bump1.obj", celShader,
    //     btVector3(1.715f, 4.815f, 1.715f), {0.0f, 4.815f, 15.587f}, 1.0f, {220, 80, 40, 255});
    // gObs[1] = MakeObstacle("bump2.obj", celShader,
    //     btVector3(0.204f, 0.204f, 1.0f), {1.039f, -0.108f, -6.458f}, 1.0f, {255, 200, 50, 255});
    // gObs[2] = MakeObstacle("bump3.obj", celShader,
    //     btVector3(0.204f, 0.204f, 1.0f), {-7.225f, -0.074f, -9.358f}, 1.0f, {80, 180, 255, 255});

    //  ----- RUNWAY RAMP (trimesh — exact slope collision) -----
    //
    //  Apply Transform was used on export — OBJ vertices ARE the final world positions.
    //  No offset applied here: the ramp sits exactly where you placed it in Blender.
    //  (You intentionally set the approach slightly underground for a smooth roll-on.)
    StaticTrimesh runway = MakeRampObstacle(
        "Obj Files/runway.obj",
        celShader,
        { 0.0f, 0.0f, 0.0f },          // NO offset — honour the Blender world position
        { 200, 200, 210, 255 }          // light grey-blue tint
    );

    // B key: toggle physics debug bounding-box visualizer
    bool showDebug = false;

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

    // brakeSmoothed mirrors engineSmoothed: instead of stamping full brake force
    // onto Bullet in a single frame, we lerp toward the target over time.
    // This gives the car momentum — it "fights" the stop instead of halting instantly.
    float brakeSmoothed  = 0.0f;

    const float MAX_ENGINE = P_ENGINE_FORCE;
    const float MAX_BRAKE  = P_BRAKE_FORCE;


    // =========================================================================
    //  STATE READ-BACK  (will be filled from Bullet each frame)
    // =========================================================================
    Vector3 carPos   = { 0, 0, 0 };
    float   heading  = 0.0f;    // yaw angle extracted from chassis matrix
    // Issue #3: separate spin variables for front and rear wheel display.
    //   wheelSpin        = FL (index 0) rotation  — used for front wheel visual
    //   rearWheelSpinRef = RL (index 2) rotation  — reference for rear accumulator delta
    //   rearWheelSpin    = custom accumulator (handles burnout over-spin)
    float   wheelSpin        = 0.0f;  // front-left Bullet m_rotation (display for FL/FR)
    float   rearWheelSpinRef = 0.0f;  // rear-left  Bullet m_rotation (delta ref for accumulator)
    float   rearWheelSpin    = 0.0f;  // rear wheels: velocity accumulator (shows burnout spin)

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
        //  ENGINE + BRAKE  —  Issue #1 fix: SMOOTHED BRAKE FORCE
        //
        //  Problem: brake force was stamped directly onto Bullet each frame.
        //  A 700 kg car going 80 km/h receiving 160 N·m wheel torque in one frame
        //  causes an instantaneous deceleration jump — the physics equivalent of
        //  hitting a wall. There was no inertia in the braking path at all.
        //
        //  Fix: brakeTarget is the desired brake force this frame (just like
        //  engineTarget). brakeSmoothed lerps toward it at P_BRAKE_IN_RATE when
        //  braking builds and P_BRAKE_OUT_RATE when it bleeds off. The car now
        //  "fights" the stop: momentum carries it forward while the brakes build,
        //  then the force fades after key release just like real hydraulic brakes.
        // =====================================================================
        float brakeTarget = 0.0f;

        if (IsKeyDown(KEY_W)) {
            if (curSpeedKmh < -2.0f) { brakeTarget = MAX_BRAKE; engineTarget = 0.0f; }
            else                       engineTarget = MAX_ENGINE;
        }
        else if (IsKeyDown(KEY_S)) {
            if (curSpeedKmh > 2.0f)  { brakeTarget = MAX_BRAKE; engineTarget = 0.0f; }
            else                       engineTarget = -MAX_ENGINE * 0.5f;
        }
        else {
            engineTarget = 0.0f;  // coast

            // Idle braking: rolling resistance + engine braking.
            // Stronger when nearly stopped (prevents creep), lighter at speed.
            if (absSpeedKmh < 3.0f)
                brakeTarget = MAX_BRAKE * 0.20f;
            else
                brakeTarget = MAX_BRAKE * 0.04f;
        }

        // Smooth brake force — same pattern as engineSmoothed.
        // Bite rate (12/s): brakes build quickly but NOT instantly.
        //   At 60 fps: ~0.2s to reach full pressure — matches hydraulic brake feel.
        // Release rate (4/s): brakes bleed off slowly so the car glides to rest,
        //   not snapping to zero deceleration the moment you release the pedal.
        float brakeRate  = (brakeTarget > brakeSmoothed) ? P_BRAKE_IN_RATE : P_BRAKE_OUT_RATE;
        brakeSmoothed   += (brakeTarget - brakeSmoothed) * brakeRate * dt;
        if (brakeSmoothed < 0.5f) brakeSmoothed = 0.0f;  // kill floating-point residual

        // Lerp engine force; snap to zero to kill floating-point residual
        float engRate = (fabsf(engineTarget) > 0.1f) ? engineInRate : engineOutRate;
        engineSmoothed += (engineTarget - engineSmoothed) * engRate * dt;
        if (fabsf(engineSmoothed) < 1.0f) engineSmoothed = 0.0f;

        // LAUNCH: bypass the smooth ramp at low speed.
        // At <10 km/h with throttle held, hit the rear with full force INSTANTLY.
        // Bug 2 fix (S→W jitter): also zero brakeSmoothed here.
        //   When reversing (S) the brake was building up to stop the car.
        //   At the moment curSpeedKmh crosses -2.0f, the launch fires full engine force
        //   BUT brakeSmoothed was still mid-ramp (release rate only 4/s) → engine + brake
        //   fighting each other → oscillation / jitter. Clearing it instantly removes that.
        if (IsKeyDown(KEY_W) && curSpeedKmh >= -2.0f && absSpeedKmh < 10.0f) {
            engineSmoothed = P_ENGINE_FORCE;  // no lerp — full torque NOW
            brakeSmoothed  = 0.0f;            // kill residual S-key brake so nothing fights the engine
        }

        // RWD: engine to rear wheels only
        vehicle->applyEngineForce(engineSmoothed, 2);
        vehicle->applyEngineForce(engineSmoothed, 3);

        // RWD brake bias with smoothed force.
        // Active braking (W or S): 70% front / 30% rear (weight transfers forward, front grips more).
        // Idle coast:              20% front / 80% rear (RWD engine braking is rear-axle only).
        bool activebraking = IsKeyDown(KEY_W) || IsKeyDown(KEY_S);
        float frontBrake = activebraking ? (brakeSmoothed * 0.70f) : (brakeSmoothed * 0.20f);
        float rearBrake  = activebraking ? (brakeSmoothed * 0.30f) : (brakeSmoothed * 0.80f);
        vehicle->setBrake(frontBrake, 0);
        vehicle->setBrake(frontBrake, 1);
        vehicle->setBrake(rearBrake,  2);
        vehicle->setBrake(rearBrake,  3);

        // --- Velocity-proportional linear damping (Issue #1 supplement) ---
        // Constant linear damping of 0.15 is too uniform: at high speed it barely
        // resists, so all deceleration came from the (formerly raw) brake force.
        // Now: damping ramps from 0.05 at highway speed to 0.35 at near-zero.
        // This creates a natural "coast curve" — the car bleeds speed gradually
        // even before the brakes fully engage, giving braking its proper inertia feel.
        {
            float speedNorm  = fminf(1.0f, absSpeedKmh / 80.0f);  // 0 at stop, 1 at 80+ km/h
            float dynLinDamp = 0.35f * (1.0f - speedNorm) + 0.05f * speedNorm;  // 0.35→0.05
            chassis->setDamping(dynLinDamp, P_ANGULAR_DAMP);
        }

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

        // signedCarOmega: SIGNED wheel speed derived from car's actual forward velocity.
        // Positive = forward, Negative = reversing.
        // fwdSpd is car velocity projected onto its own heading (Bullet local +Z).
        // This MUST be signed so that reverse (S) gives a negative target omega,
        // allowing rearSpinOmega to go negative and the visual accumulator to roll backward.
        float signedCarOmega = fwdSpd / P_WHEEL_RADIUS;
        float carOmegas      = fabsf(signedCarOmega);  // magnitude — used only for slip/friction math
        float refOmega       = (P_LAUNCH_SPEED_KMH / 3.6f) / P_WHEEL_RADIUS;  // reference max spin

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
            targetRearOmega = signedCarOmega;  // signed; always positive here (launching guards it)
            omegaUp = 8.0f; omegaDown = 8.0f;
        } else if (isDrifting && throttleOn) {
            // Sustained drift: spin proportional to sideways angle
            targetRearOmega = carOmegas * (1.0f + driftRatio * 1.5f);
            omegaUp = 5.0f; omegaDown = 2.0f;
        } else {
            // No throttle, S pressed, coasting, or reversing:
            // SIGNED target so reverse correctly drives rearSpinOmega negative.
            // On S-release: signedCarOmega rises from negative → 0 smoothly,
            // rearSpinOmega follows without ever jumping to a positive value.
            targetRearOmega = signedCarOmega;
            omegaUp = 3.0f; omegaDown = 8.0f;
        }

        float omegaRate = (targetRearOmega > rearSpinOmega) ? omegaUp : omegaDown;
        rearSpinOmega  += (targetRearOmega - rearSpinOmega) * omegaRate * dt;
        // DO NOT clamp rearSpinOmega to >= 0 here.
        // During reverse, rearSpinOmega must stay negative so the visual accumulator
        // receives a negative target and rolls the wheels backward.
        // The old clamp to 0 was the root cause of the forward snap on S-release.

        // Friction derived from slip ratio (physical relationship)
        // Over-spin: wheel spinning faster than car → tire slip → low friction
        float slip      = fmaxf(0.0f, rearSpinOmega - carOmegas);  // excess spin
        float slipRatio = fminf(1.0f, slip / (refOmega + 0.01f));  // 0=grip, 1=full slip
        rearFriction    = P_REAR_FRICTION    * (1.0f - slipRatio)
                        + P_LAUNCH_SPIN_FRIC * slipRatio;

        // HIGH-SPEED HANDBRAKE OVERRIDE
        // Use a smooth speed-blended factor instead of a binary threshold so there is
        // no hard snap in grip or braking when the car slows through P_HB_SPEED_THRESH.
        //   At high speed (>threshold): hbFactor ≈ 1.0 → full drift friction (0.35)
        //   At low speed  (<threshold): hbFactor ≈ 0.0 → normal rear friction (2.8)
        //   Transition is blended over ±4 km/h around the threshold → smooth.
        float hbFactor = 0.0f;
        if (handbrake) {
            // Smoothstep-style blend: 1 at high speed, 0 at low speed
            float t = (absSpeedKmh - P_HB_SPEED_THRESH + 4.0f) / 8.0f;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            hbFactor = t * t * (3.0f - 2.0f * t);  // smoothstep
        }

        if (highSpdHB) {
            // Blend from normal rear friction to low drift friction
            rearFriction = P_REAR_FRICTION * (1.0f - hbFactor)
                         + P_HB_HIGH_SPD_FRIC * hbFactor;
        }

        vehicle->getWheelInfo(2).m_frictionSlip = rearFriction;
        vehicle->getWheelInfo(3).m_frictionSlip = rearFriction;

        if (highSpdHB) {
            // REAL HANDBRAKE: lock rear, free front — rear slides, front steers the arc
            // Brake force also blended with speed so it doesn't hard-lock at the threshold
            float hbBrake = MAX_BRAKE * 2.8f * hbFactor;
            vehicle->setBrake(hbBrake, 2);
            vehicle->setBrake(hbBrake, 3);
            vehicle->setBrake(0.0f, 0);
            vehicle->setBrake(0.0f, 1);
        } else if (lowSpdHB) {
            // Low-speed: light front brake (weight transfer for launch)
            vehicle->setBrake(MAX_BRAKE * 0.6f, 0);
            vehicle->setBrake(MAX_BRAKE * 0.6f, 1);
        }

        // 4. Weight Transfer (research: W_r decreases during braking)
        //    Braking shifts load forward → rear axle unloads → grip drops
        //    Now uses brakeSmoothed so weight transfer also ramps in/out smoothly.
        float brakeNorm   = brakeSmoothed / P_BRAKE_FORCE;
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

        // B KEY: toggle physics AABB debug visualizer
        if (IsKeyPressed(KEY_B)) showDebug = !showDebug;

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
        //  ENGINE RPM CALCULATION + AUDIO UPDATE
        // =====================================================================
        {
            // --- M key: toggle audio on/off ---
            if (IsKeyPressed(KEY_M)) {
                audioEnabled = !audioEnabled;
                // Immediately silence everything when muting
                if (!audioEnabled) {
                    for (int i = 0; i < ENG_SAMPLE_COUNT; i++)
                        SetSoundVolume(engSounds[i], 0.0f);
                }
            }

            // --- Compute raw RPM from wheel spin omega ---
            float omegaForRPM  = fabsf(rearSpinOmega);
            float blipRPM  = RPM_IDLE + omegaForRPM * RPM_OMEGA_SCALE;

            if (throttleOn && absSpeedKmh < 5.0f) {
                float burnoutRPM = RPM_IDLE + fabsf(rearSpinOmega) * RPM_OMEGA_SCALE * 1.4f;
                if (burnoutRPM > blipRPM) blipRPM = burnoutRPM;
            }

            if (blipRPM < RPM_IDLE) blipRPM = RPM_IDLE;
            if (blipRPM > RPM_MAX ) blipRPM = RPM_MAX;
            rawRPM = blipRPM;

            float rpmRiseRate = 8.0f;
            float rpmFallRate = 3.5f;
            float rpmRate = (rawRPM > currentRPM) ? rpmRiseRate : rpmFallRate;
            currentRPM += (rawRPM - currentRPM) * rpmRate * dt;

            // --- Crossfade blender (only runs when audio is enabled) ---
            if (audioEnabled) {
                int idxA = 0;
                for (int i = 0; i < ENG_SAMPLE_COUNT - 1; i++) {
                    if (currentRPM >= ENG_RPM_POINTS[i]) idxA = i;
                    else break;
                }
                int idxB = idxA + 1;
                if (idxB >= ENG_SAMPLE_COUNT) idxB = ENG_SAMPLE_COUNT - 1;

                float rpmA = ENG_RPM_POINTS[idxA];
                float rpmB = ENG_RPM_POINTS[idxB];
                float span = rpmB - rpmA;
                float t    = (span > 0.01f) ? (currentRPM - rpmA) / span : 0.0f;
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;

                float volA   = (1.0f - t);
                float volB   = t;
                float pitchA = (rpmA > 0.0f) ? (currentRPM / rpmA) : 1.0f;
                float pitchB = (rpmB > 0.0f) ? (currentRPM / rpmB) : 1.0f;

                for (int i = 0; i < ENG_SAMPLE_COUNT; i++) {
                    if (!IsSoundPlaying(engSounds[i])) PlaySound(engSounds[i]);

                    if (i == idxA && idxA != idxB) {
                        SetSoundVolume(engSounds[i], volA * 0.85f);
                        SetSoundPitch (engSounds[i], pitchA);
                    } else if (i == idxB) {
                        float v = (idxA == idxB) ? 1.0f : volB;
                        SetSoundVolume(engSounds[i], v * 0.85f);
                        SetSoundPitch (engSounds[i], pitchB);
                    } else {
                        SetSoundVolume(engSounds[i], 0.0f);
                    }
                }
            }
        }

        // =====================================================================
        //  READ-BACK: extract carPos, heading, wheelSpin from Bullet
        // =====================================================================
        btTransform chassisTrans;
        chassis->getMotionState()->getWorldTransform(chassisTrans);

        btVector3 origin = chassisTrans.getOrigin();
        // carPos.y = chassis centre Y relative to flat-ground rest height (0.80m).
        // On flat ground: origin.y ≈ 0.80 → carPos.y = 0 → no change from before.
        // On a ramp:      origin.y rises  → carPos.y > 0 → car visual follows physics up.
        carPos = {
            (float)origin.x(),
            (float)origin.y() - 0.80f,   // relative height: 0 on flat ground, +ve on ramp
            (float)origin.z()
        };

        // Extract yaw heading: forward axis in world space (+Z local → world)
        btVector3 fwdBt = chassisTrans.getBasis() * btVector3(0, 0, 1);
        heading = atan2f((float)fwdBt.x(), (float)fwdBt.z());

        // Issue #3 fix 1: separate front/rear wheel readback.
        //   OLD: both front display and rear accumulator reference read from index 2 (RL wheel).
        //   BUG: front wheel models were showing rear-left wheel rotation — wrong index!
        //   FIX: front display → index 0 (FL).  Rear accumulator reference → index 2 (RL).
        vehicle->updateWheelTransform(0, true);
        wheelSpin        = (float)vehicle->getWheelInfo(0).m_rotation;  // FL — for front visual
        vehicle->updateWheelTransform(2, true);
        rearWheelSpinRef = (float)vehicle->getWheelInfo(2).m_rotation;  // RL — for rear delta

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
            // rearWheelVel accumulator.
            // Issue #3 fix 2: use rearWheelSpinRef (RL, index 2) for the delta
            // so the rear accumulator's omega derives from the correct driven wheel.
            static float rearWheelVel  = 0.0f;
            static float prevRearSpin  = 0.0f;  // tracks RL m_rotation across frames
            float deltaAngle       = rearWheelSpinRef - prevRearSpin;
            prevRearSpin          = rearWheelSpinRef;
            // wheelOmegaActual: RL wheel's actual angular velocity from Bullet.
            // Negative when reversing (S), positive forward, high during W-only launch slip.
            float wheelOmegaActual = (dt > 0.001f) ? (deltaAngle / dt) : 0.0f;

            // extraOmega: W+SPACE burnout — how much rear spin exceeds rolling speed.
            float extraOmega = fmaxf(0.0f, rearSpinOmega - carOmegas);

            float targetVel;
            float velRate;

            if (extraOmega > 1.5f) {
                // W+SPACE burnout / drift over-spin: fast forward spin-up
                targetVel = rearSpinOmega;
                velRate   = 14.0f;
            } else if (absSpeedKmh < 1.5f && !throttleOn && extraOmega < 0.5f) {
                // Near-stopped and no throttle: snap wheel spin to 0 quickly.
                // This prevents the "hover/glide" effect where rear wheels keep
                // spinning visually after the car has physically stopped.
                // Rate 25/s → spin decays from any value to <0.01 in ~0.25s.
                targetVel = 0.0f;
                velRate   = 25.0f;
            } else {
                // Normal rolling (W-only launch, forward, coasting, reverse).
                // Bug 3 fix (rear wheel lag): velRate raised from 5 → 25.
                //   Old rate 5/s: tau = 0.2s → rear visibly lags behind front for 0.2s
                //   New rate 25/s: tau = 0.04s → convergence in ~2 frames, imperceptible.
                //   wheelOmegaActual is derived from the same Bullet m_rotation that drives
                //   the front wheel display, so at high velRate the rear tracks front exactly.
                targetVel = wheelOmegaActual;
                velRate   = 25.0f;
            }

            rearWheelVel  += (targetVel - rearWheelVel) * velRate * dt;
            rearWheelSpin += rearWheelVel * dt;  // integrate — no direct assignment → no snap
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

                // Shadow disc: drawn flat on the ground directly under the car.
                // Makes ground contact visually obvious — very cheap (just a circle).
                // Radius scales slightly with speed to give a subtle "speed blob" feel.
                {
                    float shadowR = 1.15f + absSpeedKmh * 0.003f;
                    int   steps   = 24;
                    for (int s = 0; s < steps; s++) {
                        float a0 = (float)s       / steps * 6.2832f;
                        float a1 = (float)(s + 1) / steps * 6.2832f;
                        Vector3 p0 = { carPos.x + cosf(a0) * shadowR,
                                       0.005f,
                                       carPos.z + sinf(a0) * shadowR };
                        Vector3 p1 = { carPos.x + cosf(a1) * shadowR,
                                       0.005f,
                                       carPos.z + sinf(a1) * shadowR };
                        DrawTriangle3D({carPos.x, 0.005f, carPos.z}, p0, p1,
                                       {0, 0, 0, 80});
                    }
                }

                // =====================================================================
                //  BODY DYNAMICS — Issue #2: PER-WHEEL SUSPENSION + PITCH + ROLL
                //
                //  Two-layer system:
                //   Physical layer:    per-wheel compression differential (what Bullet measured)
                //   Perceptual layer:  G-force / force estimate (immediate visual response)
                //  Blended and smoothed with static lerp vars to avoid per-frame jitter.
                // =====================================================================

                // --- Per-wheel suspension compression (0=extended, 1=compressed) ---
                // Wheel layout:  FL=0 (left-front)  FR=1 (right-front)
                //                RL=2 (left-rear)   RR=3 (right-rear)
                float comp[4];
                for (int wi = 0; wi < 4; wi++) {
                    vehicle->updateWheelTransform(wi, true);
                    const btWheelInfo& w = vehicle->getWheelInfo(wi);
                    float c = 1.0f - (w.m_raycastInfo.m_suspensionLength / P_SUSP_LENGTH);
                    comp[wi] = fmaxf(0.0f, fminf(1.0f, c));
                }
                float avgSuspComp = (comp[0] + comp[1] + comp[2] + comp[3]) * 0.25f;
                float bodyBob = avgSuspComp * P_SUSP_BOB_SCALE * P_SUSP_LENGTH;

                // --- PITCH: Nose-dive (braking) and rear-squat (acceleration) ---
                //
                //  Physical: front suspension compresses more than rear under braking
                //  (weight transfers forward = nose dives).
                //  Rear compresses more under hard acceleration (rear settles = squat).
                //
                //  Geometric formula: pitch ≈ height_diff / wheelBase
                //  height_diff = (frontAvg - rearAvg) * P_SUSP_LENGTH
                //  Negative result = nose down (dive). Positive = nose up (squat).
                float frontAvg   = (comp[0] + comp[1]) * 0.5f;   // FL + FR average
                float rearAvg    = (comp[2] + comp[3]) * 0.5f;   // RL + RR average
                float suspPitch  = -(frontAvg - rearAvg) * P_SUSP_LENGTH / wheelBase * P_BODY_PITCH;

                //  Perceptual: brake/throttle force gives IMMEDIATE pitch feel before
                //  suspension has time to build up compression (lag-compensation).
                //  Bug 1 fix (false burnout pitch): throttle squat ONLY added when not holding
                //  handbrake. W+SPACE = stationary burnout → car isn't accelerating forward →
                //  no rear squat. Without this guard, the squat fired even when the car was
                //  standing still spinning its wheels, which looked obviously fake.
                float forcePitch = -(brakeSmoothed / P_BRAKE_FORCE) * 0.07f;
                if (throttleOn && !handbrake)   // real forward acceleration only, not burnout
                    forcePitch += (fabsf(engineSmoothed) / P_ENGINE_FORCE) * 0.045f;

                // Blend: 50% perceptual (snappy) + 50% physical (confirmed)
                // Negated: MatrixRotateX convention has opposite sign to what nose-dive implies.
                float rawPitch = -(forcePitch * 0.50f + suspPitch * 0.50f);

                // --- ROLL: Lean into corners ---
                //
                //  Physical: right wheels compressed more than left → body leans RIGHT.
                //  leftAvg > rightAvg → left-side lower → lean LEFT (negative Z rotation).
                //  Sign: (leftAvg - rightAvg) is negative when turning left (right compressed)
                //  which matches gRoll being negative when turning left — both correct ✓
                float leftAvg   = (comp[0] + comp[2]) * 0.5f;   // FL + RL average
                float rightAvg  = (comp[1] + comp[3]) * 0.5f;   // FR + RR average
                float suspRoll  = (leftAvg - rightAvg) * P_SUSP_LENGTH / trackWidth * 2.2f;

                //  Perceptual: lateral velocity gives immediate roll — matches existing formula
                //  but with correct coefficient (no hidden *0.05 factor).
                //  latSpd > 0 (sliding right / turning left) → negative Z → lean right ✓
                float gRoll = -latSpd * P_BODY_ROLL;
                if (gRoll >  0.18f) gRoll =  0.18f;
                if (gRoll < -0.18f) gRoll = -0.18f;

                // Blend: 65% G-force (snappy) + 35% suspension (physical confirmation)
                float rawRoll = gRoll * 0.65f + suspRoll * 0.35f;

                // --- Smooth pitch and roll across frames (static = persists per frame) ---
                //  Without smoothing, Bullet's per-substep suspension variance causes
                //  visible jitter in the body angle. Lerp rate:
                //    9/s for roll  → ~0.11s response  (snappy corners)
                //    7/s for pitch → ~0.14s response  (slightly slower = weight feels heavy)
                static float bodyRoll  = 0.0f;
                static float bodyPitch = 0.0f;
                bodyRoll  += (rawRoll  - bodyRoll)  * 9.0f * dt;
                bodyPitch += (rawPitch - bodyPitch) * 7.0f * dt;

                // --- Engine vibration: scales with speed ---
                float vibration = P_VIB_AMP
                    * (fabsf(curSpeedKmh) / 80.0f)
                    * sinf((float)GetTime() * P_VIB_FREQ * 6.2832f);

                // ── CHASSIS matrix: full 6DOF including pitch (nose-dive/squat) and roll ──
                // bodyBob raises the chassis body as suspension compresses (visual spring effect).
                // P_CHASSIS_Y_OFFSET is a static tune parameter — adjust to seat the body flush
                // in the wheel arches. Negative = lower the body.
                Matrix carMat = MatrixMultiply(
                    MatrixMultiply(
                        MatrixMultiply(
                            MatrixRotateX(bodyPitch),
                            MatrixRotateZ(bodyRoll)
                        ),
                        MatrixRotateY(heading)
                    ),
                    MatrixTranslate(carPos.x,
                                   carPos.y + P_CHASSIS_Y_OFFSET + vibration,
                                   carPos.z)
                );

                // ── WHEEL base matrix: yaw + gentle lean only, NO pitch ──
                //
                //  KEY FIX — wheelie elimination:
                //  The old code used carMat for both chassis AND wheels. When bodyPitch tilted
                //  the chassis (rear squats under acceleration), the front wheels rose with it
                //  — a full visual wheelie with no suspension illusion whatsoever.
                //
                //  Solution: give wheels their own matrix that has:
                //    • Yaw:   wheels rotate with car heading ✓
                //    • Lean:  20% of bodyRoll for subtle camber feel ✓
                //    • Y=0:  wheels anchored at ground level, translate y=0 ✓
                //    • NO pitch: wheels stay ground-parallel always ✓
                //
                //  Result: chassis pitches visibly over stationary wheels → looks like real
                //  suspension compressing/extending, with no fake wheelie.
                float wheelLean = bodyRoll * 0.2f;  // subtle camber (20% of chassis lean)
                Matrix wheelBaseMat = MatrixMultiply(
                    MatrixMultiply(
                        MatrixRotateZ(wheelLean),
                        MatrixRotateY(heading)
                    ),
                    MatrixTranslate(carPos.x, carPos.y, carPos.z)  // wheels follow chassis height
                );

                // --- Wheels: front use wheelSpin (FL, index 0), rear use rearWheelSpin ---
                // Each wheel placed at wheelLocal[i].y = wheelRadius in wheelBaseMat space.
                // wheelBaseMat.y = 0 → world wheel center = wheelRadius → bottom at y=0 ✓
                for (int i = 0; i < 4; i++) {
                    bool front = (i < 2);
                    Matrix m = MatrixRotateX(front ? wheelSpin : rearWheelSpin);
                    if (front) m = MatrixMultiply(m, MatrixRotateY(steerSmoothed));
                    m = MatrixMultiply(m, MatrixTranslate(
                        wheelLocal[i].x, wheelLocal[i].y, wheelLocal[i].z));
                    m = MatrixMultiply(m, wheelBaseMat);  // no pitch — stays on ground
                    wheelModels[i].transform = m;
                    DrawModel(wheelModels[i], {0,0,0}, 1.0f, WHITE);
                }

                // --- Chassis: uses full carMat (pitch + roll + yaw) ---
                Matrix chassisMat = MatrixMultiply(MatrixTranslate(0, 0, -0.1f), carMat);
                carModel.transform = chassisMat;
                DrawModel(carModel, {0,0,0}, 1.0f, WHITE);

                // --- Runway ramp ---
                runway.model.transform = MatrixTranslate(
                    runway.offset.x, runway.offset.y, runway.offset.z);
                DrawModel(runway.model, {0,0,0}, 1.0f, runway.tint);
                DrawModelWires(runway.model, {0,0,0}, 1.0f, {120, 120, 140, 80});

                // =====================================================================
                //  DEBUG: Physics AABB Visualizer  (toggle with B key)
                //  CYAN   = car chassis AABB  (where Bullet thinks the car box IS)
                //  YELLOW = runway trimesh AABB
                //  MAGENTA= wheel centre spheres  (actual suspension contact origin)
                //  RED    = active ground contact point per wheel
                //  ORANGE = ground plane reference line at Y=0
                // =====================================================================
                if (showDebug) {
                    // --- Car chassis AABB ---
                    btVector3 cMin, cMax;
                    chassis->getCollisionShape()->getAabb(
                        chassis->getWorldTransform(), cMin, cMax);
                    DrawBoundingBox({
                        {(float)cMin.x(),(float)cMin.y(),(float)cMin.z()},
                        {(float)cMax.x(),(float)cMax.y(),(float)cMax.z()}},
                        SKYBLUE);

                    // --- Runway trimesh AABB ---
                    btVector3 rMin, rMax;
                    runway.shape->getAabb(
                        runway.body->getWorldTransform(), rMin, rMax);
                    DrawBoundingBox({
                        {(float)rMin.x(),(float)rMin.y(),(float)rMin.z()},
                        {(float)rMax.x(),(float)rMax.y(),(float)rMax.z()}},
                        YELLOW);

                    // --- Wheel spheres + contact points ---
                    for (int wi = 0; wi < 4; wi++) {
                        vehicle->updateWheelTransform(wi, true);
                        const btWheelInfo& wInfo = vehicle->getWheelInfo(wi);

                        // Wheel centre (where the axle is)
                        btVector3 wc = wInfo.m_worldTransform.getOrigin();
                        DrawSphere({(float)wc.x(),(float)wc.y(),(float)wc.z()},
                                   0.08f, MAGENTA);

                        // Ground contact point (where the wheel touches)
                        if (wInfo.m_raycastInfo.m_isInContact) {
                            btVector3 cp = wInfo.m_raycastInfo.m_contactPointWS;
                            DrawSphere({(float)cp.x(),(float)cp.y(),(float)cp.z()},
                                       0.06f, RED);
                        }
                    }

                    // --- Y=0 ground plane reference box (shows where ground IS) ---
                    DrawBoundingBox({{-30.f,-0.02f,-30.f},{30.f,0.02f,30.f}},
                                    {255,140,0,180});

                    // --- Debug text overlay ---
                    DrawText("[DEBUG PHYSICS ON]", 10, GetScreenHeight()-115, 14,
                             {100,255,255,255});
                    DrawText("CYAN=Chassis  YELLOW=Ramp  MAGENTA=Wheels  RED=Contact",
                             10, GetScreenHeight()-98, 12, {200,200,200,200});
                }


            EndMode3D();

            // --- HUD ---
            DrawText("Arcade Drift Engine", 10, 10, 18, WHITE);

            // RPM gauge
            {
                float rpmFrac = (currentRPM - RPM_IDLE) / (RPM_MAX - RPM_IDLE);
                if (rpmFrac < 0.0f) rpmFrac = 0.0f;
                if (rpmFrac > 1.0f) rpmFrac = 1.0f;

                // Colour: green → yellow → red as RPM climbs
                Color rpmCol;
                if (rpmFrac < 0.6f)       rpmCol = {80, 220, 80, 255};
                else if (rpmFrac < 0.85f) rpmCol = {255, 200, 0, 255};
                else                       rpmCol = {255, 50, 50, 255};

                int barW = 200;
                DrawText(TextFormat("RPM       : %5.0f", currentRPM), 10, 138, 16, rpmCol);
                DrawRectangle(160, 140, barW, 14, {40,40,40,200});
                DrawRectangle(160, 140, (int)(rpmFrac * barW), 14, rpmCol);
            }
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

            DrawText("SPACE: Handbrake  |  R: Reset  |  M: Mute  |  B: Debug",
                     10, 118, 14, {150,150,150,255});

            int y = GetScreenHeight() - 90;
            DrawText("W/S  : Throttle / Brake+Reverse", 10, y,    15, {180,180,180,255});
            DrawText("A/D  : Steer",                    10, y+18, 15, {180,180,180,255});
            DrawText("Mouse: Orbit camera",              10, y+36, 15, {180,180,180,255});
            DrawText("R    : Reset car upright",         10, y+54, 15, {255,100,100,255});
            DrawText("M    : Toggle engine audio",       10, y+72, 15,
                audioEnabled ? Color{100,255,100,255} : Color{255,80,80,255});
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

    // Runway ramp trimesh (body already freed by collision objects loop above)
    delete runway.shape;
    delete runway.triData;
    UnloadModel(runway.model);

    // Engine audio
    for (int i = 0; i < ENG_SAMPLE_COUNT; i++) {
        StopSound(engSounds[i]);
        UnloadSound(engSounds[i]);
    }
    CloseAudioDevice();

    // Raylib objects
    for (int i = 0; i < 4; i++) UnloadModel(wheelModels[i]);
    UnloadModel(carModel);
    UnloadShader(celShader);
    CloseWindow();
    return 0;
}
