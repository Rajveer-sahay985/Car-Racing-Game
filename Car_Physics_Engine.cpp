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
    InitWindow(1280, 720, "Vortex Racing — Step 1: Bullet Physics");
    SetTargetFPS(60);

    // -------------------------------------------------------------------------
    //  CEL SHADER  (unchanged from Step 0)
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
    float wheelRadius = 0.333f;

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

    btRigidBody* chassis = MakeRigidBody(900.0f, startTrans, chassisShape);
    chassis->setActivationState(DISABLE_DEACTIVATION);
    chassis->setDamping(0.15f, 0.75f);


    // ---- btRaycastVehicle ----
    btVehicleRaycaster* raycaster = new btDefaultVehicleRaycaster(gWorld);

    btRaycastVehicle::btVehicleTuning tuning;
    tuning.m_suspensionStiffness    = 25.0f;
    tuning.m_suspensionDamping      = 3.0f;
    tuning.m_suspensionCompression  = 4.0f;
    tuning.m_maxSuspensionTravelCm  = 20.0f;
    tuning.m_frictionSlip           = 2.5f;
    tuning.m_maxSuspensionForce     = 7000.0f;

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
    const float     suspLen  = 0.45f;      // suspension rest length (m)

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
        wi.m_rollInfluence     = 0.15f;   // anti-roll resistance (higher = harder to flip)
        if (!isFront[i])
            wi.m_frictionSlip  = 2.8f;    // rear gets a bit more grip (RWD)
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
    float maxSteer      = 35.0f * DEG2RAD;  // maximum steer at low speed
    float steerInRate   = 3.5f;             // how fast steer ramps IN  (lerp speed)
    float steerOutRate  = 7.0f;             // how fast steer returns to 0 (2x faster)
    float steerTarget   = 0.0f;             // what the player is asking for
    float steerSmoothed = 0.0f;             // actual value sent to vehicle (smoothed)

    float engineTarget   = 0.0f;            // raw engine force target
    float engineSmoothed = 0.0f;            // smoothed engine force
    float engineInRate   = 6.0f;            // ramp-up rate
    float engineOutRate  = 10.0f;           // ramp-down / engine-off rate

    const float MAX_ENGINE = 3000.0f;
    const float MAX_BRAKE  = 160.0f;


    // =========================================================================
    //  STATE READ-BACK  (will be filled from Bullet each frame)
    // =========================================================================
    Vector3 carPos   = { 0, 0, 0 };
    float   heading  = 0.0f;    // yaw angle extracted from chassis matrix
    float   wheelSpin= 0.0f;    // rolling angle from btWheelInfo::m_rotation

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

        // =====================================================================
        //  STEP 3: INPUT SMOOTHING + SPEED-SENSITIVE STEERING
        // =====================================================================
        float curSpeedKmh = vehicle->getCurrentSpeedKmHour();
        float absSpeedKmh = fabsf(curSpeedKmh);

        // --- Speed-sensitive max steer ---
        // At 0 km/h  → full 35°
        // At 120 km/h → reduced to 12° (NFS-style)
        float speedFactor  = 1.0f - 0.77f * (absSpeedKmh / 120.0f);
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
        if (fabsf(engineSmoothed) < 1.0f) engineSmoothed = 0.0f; // kill ghost forces

        // RWD: engine to rear wheels only
        vehicle->applyEngineForce(engineSmoothed, 2);
        vehicle->applyEngineForce(engineSmoothed, 3);

        // Brakes on all 4 wheels
        vehicle->setBrake(brakeForce, 0);
        vehicle->setBrake(brakeForce, 1);
        vehicle->setBrake(brakeForce, 2);
        vehicle->setBrake(brakeForce, 3);


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

        // Wheel spin: Bullet accumulates rotation per-wheel; use rear-left (2)
        // m_rotation is in radians, accumulates over time — same as our old wheelSpin
        vehicle->updateWheelTransform(2, true);
        wheelSpin = (float)vehicle->getWheelInfo(2).m_rotation;

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

                Matrix carMat = MatrixMultiply(
                    MatrixRotateY(heading),
                    MatrixTranslate(carPos.x, 0, carPos.z)
                );

                // --- Wheels (same matrix construction as Step 0) ---
                for (int i = 0; i < 4; i++) {
                    bool front = (i < 2);
                    Matrix m = MatrixRotateX(wheelSpin);
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
            DrawText("Step 3: Input Smoothing + Speed-Sensitive Steer", 10, 10, 18, WHITE);
            DrawText(TextFormat("Speed  : %5.1f km/h",    fabsf(curSpeedKmh)),     10, 38, 16, GREEN);
            DrawText(TextFormat("Steer  : %5.1f deg",     steerSmoothed * RAD2DEG),10, 58, 16, YELLOW);
            DrawText(TextFormat("SpeedFactor: %.2f  (steer limit)", speedFactor),   10, 78, 15, {180,180,255,255});
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
