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

    // ---- Chassis: compound shape = body box + 4 wheel cylinder approximations ----
    //
    //  btRaycastVehicle suspension raycasts automatically EXCLUDE the chassis
    //  rigid body, so adding wheel cylinders to it does NOT interfere with
    //  suspension. The cylinders give the wheels real collision against walls.
    //
    //  At rest the wheel centres are at local y ≈ -(suspLen + wheelRadius)
    //               = -(0.45 + 0.333) = -0.783  below the chassis centre.
    //  We place cylinders at local y = -0.68 and radius 0.30 (slightly inside
    //  the tyre) so they float ~3cm off the ground at rest — they don't clash
    //  with the ground plane but DO hit obstacle sides.
    //
    const float HUB_Y      = -0.68f;   // local y for wheel hub cylinders
    const float CYL_RADIUS = 0.30f;    // slightly less than wheelRadius (0.333)
    const float CYL_HWIDTH = 0.18f;    // half tyre width on the axle axis

    btCompoundShape* chassisShape = new btCompoundShape();

    // Child 1: main body box
    btBoxShape* bodyBox = new btBoxShape(btVector3(0.85f, 0.35f, 2.1f));
    btTransform bodyLocal; bodyLocal.setIdentity();
    chassisShape->addChildShape(bodyLocal, bodyBox);

    // Children 2-5: wheel hub cylinders (btCylinderShapeX rotates around X)
    // half-extents: (half-axle-width, radius, radius)
    btCylinderShapeX* wheelCyl = new btCylinderShapeX(
        btVector3(CYL_HWIDTH, CYL_RADIUS, CYL_RADIUS));

    const float HUB_X[4] = { -0.85f,  0.85f, -0.85f,  0.85f };
    const float HUB_Z[4] = {  1.55f,  1.55f, -1.55f, -1.55f };
    for (int i = 0; i < 4; i++) {
        btTransform cylLocal; cylLocal.setIdentity();
        cylLocal.setOrigin(btVector3(HUB_X[i], HUB_Y, HUB_Z[i]));
        chassisShape->addChildShape(cylLocal, wheelCyl);
    }

    // Spawn the car chassis so its bottom face sits at y=0
    // Bottom of box = origin.y - halfExtent.y = 0.35  → origin.y = 0.35
    // Add suspension length so wheels can extend: origin.y = 0.35 + 0.45 = 0.80
    btTransform startTrans;
    startTrans.setIdentity();
    startTrans.setOrigin(btVector3(0, 0.80f, 0));

    btRigidBody* chassis = MakeRigidBody(900.0f, startTrans, chassisShape);
    chassis->setActivationState(DISABLE_DEACTIVATION);
    chassis->setDamping(0.05f, 0.7f);   // (linear, angular)


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
        wi.m_rollInfluence     = 0.08f;   // low = less body roll = planted feel
        if (!isFront[i])
            wi.m_frictionSlip  = 2.8f;    // rear gets a bit more grip (RWD)
    }

    // =========================================================================
    //  OBSTACLES — exactly 3, one per exported OBJ, at their Blender positions
    // =========================================================================
    //
    //  We compute the Bullet body centre and half-extents directly from the
    //  OBJ vertex min/max bounds so physics and visuals stay in sync.
    //  DrawModel is called with position (0,0,0) so the baked Blender vertex
    //  coordinates are used exactly as exported — no extra offset applied.
    //
    //  bump1 vertices:  x [-1.715 , 1.715]  y [0.000 ,  9.630]  z [13.872, 17.302]
    //    → centre ( 0.000,  4.815, 15.587 )   half (1.715, 4.815, 1.715)
    //
    //  bump2 vertices:  x [ 0.835 , 1.243]  y [-0.313,  0.096]  z [-7.458, -5.458]
    //    → centre ( 1.039, -0.108, -6.458 )   half (0.204, 0.204, 1.000)
    //
    //  bump3 vertices:  x [-7.429 ,-7.021]  y [-0.278,  0.130]  z [-10.358, -8.358]
    //    → centre (-7.225, -0.074, -9.358 )   half (0.204, 0.204, 1.000)
    //
    static const int NUM_OBS = 3;
    Obstacle gObs[NUM_OBS];

    // bump1 — tall block
    gObs[0] = MakeObstacle("bump1.obj", celShader,
        btVector3(1.715f, 4.815f, 1.715f),
        {0.0f, 4.815f, 15.587f},
        1.0f, {220, 80, 40, 255});

    // bump2 — small speed strip
    gObs[1] = MakeObstacle("bump2.obj", celShader,
        btVector3(0.204f, 0.204f, 1.0f),
        {1.039f, -0.108f, -6.458f},
        1.0f, {255, 200, 50, 255});

    // bump3 — small speed strip
    gObs[2] = MakeObstacle("bump3.obj", celShader,
        btVector3(0.204f, 0.204f, 1.0f),
        {-7.225f, -0.074f, -9.358f},
        1.0f, {80, 180, 255, 255});

    // =========================================================================
    //  PHYSICS INPUT STATE  (replaces the old physics state vars)
    // =========================================================================
    float steerAngle  = 0.0f;   // current steering (we keep applying to vehicle)
    float steerSpeed  = 2.5f;
    float steerReturn = 5.0f;
    float maxSteer    = 35.0f * DEG2RAD;

    const float MAX_ENGINE = 3000.0f;   // Newtons applied to rear wheels
    const float MAX_BRAKE  = 160.0f;    // N·m braking torque

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
        //  STEERING INPUT  (raw, same style as Step 0)
        // =====================================================================
        if (IsKeyDown(KEY_A))      steerAngle += steerSpeed * dt;
        else if (IsKeyDown(KEY_D)) steerAngle -= steerSpeed * dt;
        else {
            if (steerAngle > 0) { steerAngle -= steerReturn * dt; if (steerAngle < 0) steerAngle = 0; }
            if (steerAngle < 0) { steerAngle += steerReturn * dt; if (steerAngle > 0) steerAngle = 0; }
        }
        if (steerAngle >  maxSteer) steerAngle =  maxSteer;
        if (steerAngle < -maxSteer) steerAngle = -maxSteer;

        // Apply steering to front wheels
        vehicle->setSteeringValue(steerAngle, 0);
        vehicle->setSteeringValue(steerAngle, 1);

        // =====================================================================
        //  ENGINE / BRAKE INPUT
        // =====================================================================
        float curSpeedKmh = vehicle->getCurrentSpeedKmHour();

        float engineForce = 0.0f;
        float brakeForce  = 0.0f;

        if (IsKeyDown(KEY_W)) {
            if (curSpeedKmh < -2.0f) brakeForce = MAX_BRAKE;   // brake first
            else                      engineForce = MAX_ENGINE;
        }
        else if (IsKeyDown(KEY_S)) {
            if (curSpeedKmh > 2.0f)  brakeForce  = MAX_BRAKE;  // brake first
            else                      engineForce = -MAX_ENGINE * 0.5f; // reverse
        }

        // RWD: engine to rear wheels only
        vehicle->applyEngineForce(engineForce, 2);
        vehicle->applyEngineForce(engineForce, 3);

        // Brakes on all 4 wheels
        vehicle->setBrake(brakeForce, 0);
        vehicle->setBrake(brakeForce, 1);
        vehicle->setBrake(brakeForce, 2);
        vehicle->setBrake(brakeForce, 3);

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
                    if (front) m = MatrixMultiply(m, MatrixRotateY(steerAngle));
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

                // --- Obstacles: render at (0,0,0) — vertex positions already
                //     encode the Blender world transform, no extra offset needed ---
                for (int i = 0; i < NUM_OBS; i++)
                    DrawModel(gObs[i].model, {0,0,0}, 1.0f, gObs[i].tint);

            EndMode3D();

            // --- HUD ---
            DrawText("Step 1: Bullet btRaycastVehicle", 10, 10, 18, WHITE);
            DrawText(TextFormat("Speed : %5.1f km/h", fabsf(curSpeedKmh)), 10, 38, 16, GREEN);
            DrawText(TextFormat("Steer : %5.1f deg",  steerAngle * RAD2DEG),10, 58, 16, YELLOW);
            int y = GetScreenHeight() - 90;
            DrawText("W/S  : Throttle / Brake+Reverse", 10, y,    15, {180,180,180,255});
            DrawText("A/D  : Steer",                    10, y+18, 15, {180,180,180,255});
            DrawText("Mouse: Orbit camera",              10, y+36, 15, {180,180,180,255});
        EndDrawing();
    }

    // =========================================================================
    //  CLEANUP
    // =========================================================================
    // Bullet objects
    gWorld->removeVehicle(vehicle);
    delete vehicle;
    delete raycaster;

    // Cleanup obstacles
    for (int i = 0; i < NUM_OBS; i++) {
        gWorld->removeRigidBody(gObs[i].body);
        if (gObs[i].body->getMotionState()) delete gObs[i].body->getMotionState();
        delete gObs[i].body;
        delete gObs[i].shape;
        UnloadModel(gObs[i].model);
    }

    for (int i = gWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = gWorld->getCollisionObjectArray()[i];
        btRigidBody* rb = btRigidBody::upcast(obj);
        if (rb && rb->getMotionState()) delete rb->getMotionState();
        gWorld->removeCollisionObject(obj);
        delete obj;
    }
    delete chassisShape;   // compound
    delete bodyBox;        // child of compound (compound doesn't own children)
    delete wheelCyl;       // shared by all 4 wheel cylinders
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
