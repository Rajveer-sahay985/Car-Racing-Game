// =============================================================================
//  Physics Sandbox — objectPicking.cpp
//  Objects:
//    Concrete ramp   (runway.obj)  — 150 kg concrete,  standalone pickable
//    4 × Tire        (wheel_*.obj) — 15 kg rubber, ALL CONNECTED via an
//                                    invisible suspension chassis + spring
//                                    constraints.  Pick any tire → whole rig moves.
//
//  Invisible suspension system
//  ────────────────────────────────────────────────────────────────────────────
//  An invisible 80 kg steel chassis frame body sits at the centre of the wheel
//  formation.  Each tire is connected to it with a
//  btGeneric6DofSpring2Constraint:
//    • Linear X/Z locked  → tire cannot wander sideways / fore-aft off chassis
//    • Linear Y  spring   → ±20 cm suspension travel  (stiffness 50, damping 6)
//    • Angular   free     → tire can rotate/tilt naturally under gravity
//  Picking any tire pulls the chassis, which pulls all other tires through
//  their own constraints.
//
//  Controls:
//    Hover mesh → glowing vertex  |  LMB click → grab from that vertex
//    Mouse drag up/down           → raise / lower held object
//    W/A/S/D                      → slide held object (camera-relative)
//    LMB release                  → drop
//    R                            → reset everything to spawn
//    RMB drag                     → orbit camera  |  Scroll = zoom
//
//  Build:
//    clang++ objectPicking.cpp -std=c++17 \
//      -I/opt/homebrew/include -I/opt/homebrew/include/bullet \
//      -L/opt/homebrew/lib \
//      -lraylib -lBulletDynamics -lBulletCollision -lLinearMath \
//      -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo \
//      -O2 -o sandbox && ./sandbox
// =============================================================================

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <cmath>
#include <string>
#include <vector>

// =============================================================================
//  ★ ALIGNMENT TUNING — CHANGE ONLY THESE VALUES TO FIX VISUAL MISALIGNMENT ★
//
//  WHEEL FORMATION  (physics spawn positions, must match car.obj wheel arches)
//  ───────────────────────────────────────────────────────────────────
//  WHEEL_TRACK  = half-distance left/right from car centre to each wheel.
//                 Increase to spread wheels out. Decrease to bring them in.
static const float WHEEL_TRACK   = 0.85f;   // metres  (try 0.7 – 1.2)

//  WHEEL_BASE   = half-distance front/rear from car centre to each axle.
//                 Increase to push axles further out. Decrease to bring them in.
static const float WHEEL_BASE    = 1.43f;   // metres  (try 1.2 – 1.9)

//  WHEEL_HEIGHT = how high the wheel centre spawns above the ground.
//                 If wheels float above ground → decrease. If they clip in → increase.
static const float WHEEL_HEIGHT  = 0.40f;   // metres  (try 0.3 – 0.6)

//  CAR BODY VISUAL ROTATION  (pure render correction, zero physics effect)
//  ───────────────────────────────────────────────────────────────────
//  CAR_YAW_DEG  = rotate car body left/right around vertical axis.
//                 Car facing BACKWARDS?   → change to 180.0f
//                 Car facing FORWARDS?    → keep at   0.0f
static const float CAR_YAW_DEG   =   0.0f;  // degrees  (try 0 or 180)

//  CAR_PITCH_DEG = rotate car body nose-up / nose-down.
//                  Nose pointing UP?  → use a negative value.
//                  Nose pointing DOWN? → use a positive value.
static const float CAR_PITCH_DEG  =   0.0f;  // degrees  (try -5 to +5)

//  CAR_ROLL_DEG = tilt car body left or right (rarely needed).
//                 Leaning RIGHT? → use a negative value.
//                 Leaning LEFT?  → use a positive value.
static const float CAR_ROLL_DEG   =   0.0f;  // degrees  (try -5 to +5)

//  CAR BODY PHYSICS POSITION OFFSET  (moves the PHYSICS body relative to chassis
//  ─────────────────────────────────  AND the visual follows automatically)
//
//  These use the btFixedConstraint frame, so when you change them:
//   ► the physics bounding box moves
//   ► the visual model follows (SyncTransform reads the physics body)
//   ► they are ALWAYS in sync — no separate visual offset needed
//
//  CAR_PHYS_OFF_X  = shift car body LEFT (+) or RIGHT (-) of chassis centre.
static const float CAR_PHYS_OFF_X =   0.0f;   // metres

//  CAR_PHYS_OFF_Y  = shift car body UP (+) or DOWN (-).
//                    Car body too LOW / clipping floor? → increase this value.
//                    Car body too HIGH / floating?      → decrease this value.
//                    Start at 0.0 and tune in 0.1 m steps.
static const float CAR_PHYS_OFF_Y =   0.6f;   // metres  (try 0.3 – 0.8)

//  CAR_PHYS_OFF_Z  = push car body FORWARD (+) or BACKWARD (-) of chassis centre.
static const float CAR_PHYS_OFF_Z =  -0.1f;   // metres

// =============================================================================
//  ★ CENTER OF MASS OFFSET (Drift & Stability Tuning) ★
// =============================================================================
//  Lowering the COM (negative Y) makes it impossible to flip during drifts.
//  Moving it backward (negative Z) changes the pendulum effect when sliding.
static const float CAR_COM_OFFSET_Y  = -0.4f;  // metres (try -0.2 to -0.6)
static const float CAR_COM_OFFSET_Z  = -0.1f;  // metres (try 0.0 to -0.3)
// =============================================================================

// =============================================================================
//  ★ VEHICLE WEIGHTS — affects suspension sag, grip loading, inertia ★
// =============================================================================
//  WHEEL_MASS     = mass of each tire (kg).  Heavier → more rotational inertia.
static const float WHEEL_MASS      =  15.0f;   // kg  (try 8 – 25)

//  CHASSIS_MASS   = mass of the invisible suspension frame (kg).
static const float CHASSIS_MASS    =  80.0f;   // kg  (try 30 – 150)

//  CAR_BODY_MASS  = mass of the visible car body (kg).  This is the dominant weight
//                   that loads the suspension springs and increases tire grip.
static const float CAR_BODY_MASS   = 1500.0f;   // kg  (try 400 – 1000)
//                   Total sprung mass per corner = (CHASSIS_MASS + CAR_BODY_MASS) / 4
// =============================================================================

// =============================================================================
//  ★ ENGINE & DRIVETRAIN ★
// =============================================================================
//  DRIVE_TORQUE   = torque (Nm) applied to each rear wheel axle per physics step.
//                   Higher → faster acceleration and more wheelspin at launch.
//                   Lower  → sluggish acceleration, less wheelspin.
static const float DRIVE_TORQUE    = 1000.0f;   // Nm  (try 200 – 1500)

//  MAX_WHEEL_SPIN = angular velocity cap on driven wheels (rad/s).
//                   Prevents wheels from spinning infinitely fast.
//                   Lower  → lower top speed.  Higher → higher top speed.
static const float MAX_WHEEL_SPIN  =  60.0f;   // rad/s  (try 10 – 60)

//  MAX_STEER_ANGLE = maximum front-wheel steer angle (radians).
//                    0.35 ≈ 20°, 0.55 ≈ 31°, 0.79 ≈ 45°
static const float MAX_STEER_ANGLE =   0.55f;  // radians

//  STEER_BLEND    = how quickly steering smoothly reaches MAX_STEER_ANGLE.
//                   0.05 = very slow, lazy.   0.25 = snappy and responsive.
static const float STEER_BLEND     =   0.14f;  // 0.0 – 1.0

static const float P_STEER_IN_RATE   = 3.5f;    // how fast steer locks in
static const float P_STEER_OUT_RATE  = 7.0f;    // how fast steer returns to 0
static const float P_STEER_SPEED_REDUCE = 0.77f;// how much high speed reduces steer

//  STEER_SERVO_FORCE = max motor force of the front-wheel steering servo (N).
static const float STEER_SERVO_FORCE = 600.0f; // N  (try 300 – 2000)

//  STEER_SERVO_SPEED = convergence speed of the steering servo (rad/s).
static const float STEER_SERVO_SPEED =   8.0f; // rad/s  (try 4 – 20)

//  BRAKE_TORQUE   = counter-torque applied to ALL wheels when SPACE is held.
//                   Higher → shorter stopping distance, more wheel lock.
//                   Lower  → soft braking, ABS-like progressive slowdown.
//                   brakeForce ramps 0→1.0 over ~0.3 seconds when SPACE held.
static const float BRAKE_TORQUE    = 900.0f;   // Nm  (try 400 – 2000)
// =============================================================================

// =============================================================================
//  ★ TIRE PHYSICS — GRIP / TRACTION / DRIFT (Pacejka-inspired slip curve) ★
//
//  The friction coefficient of the driven rear wheels changes every frame
//  based on how much the tire is slipping relative to the car’s ground speed.
//  This produces realistic burnout, launch grip build-up, and controlled drift.
//
//  Slip ratio curve:
//
//   friction
//     1.1 |        .....PEAK.....
//         |      .·              ·.
//    0.65 |    .·                  ·.
//     0.4 |  .·                      ·.
//    0.35 |.·                          ·--------- SPIN
//         +----+-------+------------+------→ slip
//             0%      15%          30%     100%
// =============================================================================
//  TIRE_SLIP_ONSET  = slip% at which peak grip is FIRST reached.
//                     Lower → grip peaks immediately (grippy, hard to spin).
//                     Higher → needs more wheelspin to build full grip.
static const float TIRE_SLIP_ONSET =  0.15f;  // 0.0 – 0.30

//  TIRE_SLIP_PEAK   = slip% where grip starts DROPPING (end of peak plateau).
//                     Narrow window (ONSET → PEAK) = sharp peak, easy to overdrive.
//                     Wide window                   = more forgiving driving.
static const float TIRE_SLIP_PEAK  =  0.30f;  // 0.0 – 0.50

//  TIRE_GRIP_IDLE   = friction at zero-slip (steady rolling, no acceleration).
//                     This is the "grounded" baseline grip.
static const float TIRE_GRIP_IDLE  =  0.65f;  // try 0.5 – 0.9

//  TIRE_GRIP_PEAK   = maximum friction at optimal slip.
//                     Higher → more grip, harder to break traction.
//                     Lower  → less grip, easier to drift.
static const float TIRE_GRIP_PEAK  =  1.05f;  // try 0.8 – 1.4

//  TIRE_GRIP_SPIN   = friction when wheels are in full burnout spin or lockup.
//                     Lower  → tyres barely grip at all = easy long burnouts.
//                     Higher → tyres always partially grip = short burnouts.
static const float TIRE_GRIP_SPIN  =  0.35f;  // try 0.15 – 0.65

//  TIRE_FRONT_GRIP  = constant friction on the FRONT (non-driven) wheels.
//                     High value → strong steering response.
//                     Low value  → understeer / sliding nose.
static const float TIRE_FRONT_GRIP =  1.00f;  // try 0.7 – 1.2
// =============================================================================

// =============================================================================
//  BULLET WORLD
// =============================================================================
static btDefaultCollisionConfiguration*     gCollCfg    = nullptr;
static btCollisionDispatcher*               gDispatcher = nullptr;
static btBroadphaseInterface*               gBroadphase = nullptr;
static btSequentialImpulseConstraintSolver* gSolver     = nullptr;
static btDiscreteDynamicsWorld*             gWorld      = nullptr;

static void InitBullet()
{
    gCollCfg    = new btDefaultCollisionConfiguration();
    gDispatcher = new btCollisionDispatcher(gCollCfg);
    gBroadphase = new btDbvtBroadphase();
    gSolver     = new btSequentialImpulseConstraintSolver();
    gWorld      = new btDiscreteDynamicsWorld(gDispatcher, gBroadphase, gSolver, gCollCfg);
    gWorld->setGravity(btVector3(0, -9.81f, 0));
    // More solver iterations so spring constraints stay stable
    gWorld->getSolverInfo().m_numIterations = 20;
}

static btRigidBody* MakeRigidBody(float mass, const btTransform& t, btCollisionShape* shape)
{
    btVector3 inertia(0,0,0);
    if (mass > 0.0f) shape->calculateLocalInertia(mass, inertia);
    auto* ms = new btDefaultMotionState(t);
    auto  ci = btRigidBody::btRigidBodyConstructionInfo(mass, ms, shape, inertia);
    auto* b  = new btRigidBody(ci);
    gWorld->addRigidBody(b);
    return b;
}

// =============================================================================
//  GENERIC PHYSICS OBJECT
// =============================================================================
struct PhysicsObj {
    Model              model;
    btRigidBody*       body       = nullptr;
    btCollisionShape*   shape      = nullptr;
    btVector3          centroid   = {0,0,0};
    btVector3          spawnPos   = {0,3,0};
    float mass        = 100.0f;
    float friction    = 0.60f;
    float restitution = 0.10f;
    float linDamp     = 0.10f;
    float angDamp     = 0.60f;
    float linDampHeld = 0.25f;
    float angDampHeld = 0.97f;
    float wheelRadius = 0.35f;  // cylinder rolling radius (m) — set from shape on load
    Color baseColor   = WHITE;
    const char* label = "";
    // Pre-sampled pick anchors (LOCAL space) — max 16, computed once at load.
    // Only these are drawn / searched on hover, eliminating per-frame vertex loops.
    std::vector<Vector3> anchors;
    btVector3          comOffset  = {0,0,0}; // Used to keep visual in sync if COM is shifted
};

// useWheelCylinder=true: builds a btCylinderShapeX from the mesh AABB for smooth
// rolling contact.  The convex hull is used for non-wheel objects (ramp, slab, etc.)
static bool LoadPhysObj(PhysicsObj& obj, const char* path, bool useWheelCylinder = false, float comOffsetY = 0.0f, float comOffsetZ = 0.0f)
{
    obj.model = LoadModel(path);
    if (obj.model.meshCount == 0) return false;
    btVector3 centroid(0,0,0); int total=0;
    for (int mi=0; mi<obj.model.meshCount; mi++) {
        Mesh& m=obj.model.meshes[mi];
        for (int vi=0; vi<m.vertexCount; vi++) {
            centroid+=btVector3(m.vertices[vi*3],m.vertices[vi*3+1],m.vertices[vi*3+2]); total++;
        }
    }
    if (!total) return false;
    centroid/=(float)total; obj.centroid=centroid;

    if (useWheelCylinder) {
        // Compute AABB in centroid-centred local space
        float mnx=1e30f,mxx=-1e30f, mny=1e30f,mxy=-1e30f, mnz=1e30f,mxz=-1e30f;
        for (int mi=0; mi<obj.model.meshCount; mi++) {
            Mesh& m=obj.model.meshes[mi];
            for (int vi=0; vi<m.vertexCount; vi++) {
                float vx=m.vertices[vi*3]-(float)centroid.x();
                float vy=m.vertices[vi*3+1]-(float)centroid.y();
                float vz=m.vertices[vi*3+2]-(float)centroid.z();
                mnx=fminf(mnx,vx); mxx=fmaxf(mxx,vx);
                mny=fminf(mny,vy); mxy=fmaxf(mxy,vy);
                mnz=fminf(mnz,vz); mxz=fmaxf(mxz,vz);
            }
        }
        // halfW = half-width along X axle, radius = extent in YZ
        float halfW  = (mxx - mnx) * 0.5f + 0.005f;
        float radY   = (mxy - mny) * 0.5f;
        float radZ   = (mxz - mnz) * 0.5f;
        float radius = fmaxf(radY, radZ) + 0.005f;
        // Cylinder axis is X (the axle direction, which we locked Angular Y/Z on).
        obj.shape = new btCylinderShapeX(btVector3(halfW, radius, radius));
    } else {
        auto* hull = new btConvexHullShape();
        for (int mi=0; mi<obj.model.meshCount; mi++) {
            Mesh& m=obj.model.meshes[mi];
            for (int vi=0; vi<m.vertexCount; vi++) {
                btVector3 v(m.vertices[vi*3],m.vertices[vi*3+1],m.vertices[vi*3+2]);
                hull->addPoint(v-centroid,false);
            }
        }
        hull->recalcLocalAabb(); hull->optimizeConvexHull();
        obj.shape = hull;
    }

    if (comOffsetY != 0.0f || comOffsetZ != 0.0f) {
        // Shift collision shape in the opposite direction so the rigid body origin (COM) moves relatively.
        obj.comOffset = btVector3(0, -comOffsetY, -comOffsetZ);
        btCompoundShape* comp = new btCompoundShape();
        btTransform offsetT; offsetT.setIdentity();
        offsetT.setOrigin(obj.comOffset);
        comp->addChildShape(offsetT, obj.shape);
        obj.shape = comp;
    } else {
        obj.comOffset = btVector3(0,0,0);
    }

    obj.spawnPos=btVector3(0,centroid.y()+1.f,0);
    btTransform t; t.setIdentity(); t.setOrigin(obj.spawnPos);
    obj.body=MakeRigidBody(obj.mass,t,obj.shape);
    obj.body->setFriction(obj.friction); obj.body->setRestitution(obj.restitution);
    obj.body->setDamping(obj.linDamp,obj.angDamp);
    if (useWheelCylinder) {
        // Cylinder rolls freely — low rolling/spinning friction avoids artificial drag
        // on the axle rotation while still keeping high lateral grip from setFriction.
        obj.body->setRollingFriction(0.005f);
        obj.body->setSpinningFriction(0.005f);
    }
    return true;
}

// Pre-compute up to 16 spatially-distributed pick anchors in LOCAL space.
// Algorithm: divide the local AABB into a 4×2×2 grid, find the nearest actual
// mesh vertex to each cell centre.  Deduplicates near-identical results.
// Called once after LoadPhysObj — hover/draw cost is now O(16) not O(N vertices).
static void ComputeAnchors(PhysicsObj& obj)
{
    obj.anchors.clear();

    // ── Local AABB ─────────────────────────────────────────────────────
    float mnx=1e30f,mny=1e30f,mnz=1e30f;
    float mxx=-1e30f,mxy=-1e30f,mxz=-1e30f;
    for (int mi=0; mi<obj.model.meshCount; mi++) {
        Mesh& m=obj.model.meshes[mi];
        for (int vi=0; vi<m.vertexCount; vi++) {
            float vx=m.vertices[vi*3], vy=m.vertices[vi*3+1], vz=m.vertices[vi*3+2];
            if(vx<mnx)mnx=vx; if(vy<mny)mny=vy; if(vz<mnz)mnz=vz;
            if(vx>mxx)mxx=vx; if(vy>mxy)mxy=vy; if(vz>mxz)mxz=vz;
        }
    }

    // ── Grid-sample (NX×NY×NZ cells) ──────────────────────────────────────
    const int NX=4, NY=2, NZ=2;  // 4×2×2 = 16 cells max
    float dx=(mxx-mnx)/NX, dy=(mxy-mny)/NY, dz=(mxz-mnz)/NZ;
    if(dx<1e-4f)dx=1.f; if(dy<1e-4f)dy=1.f; if(dz<1e-4f)dz=1.f;

    for (int iz=0; iz<NZ; iz++)
    for (int iy=0; iy<NY; iy++)
    for (int ix=0; ix<NX; ix++)
    {
        float cx=mnx+(ix+0.5f)*dx;
        float cy=mny+(iy+0.5f)*dy;
        float cz=mnz+(iz+0.5f)*dz;

        float bestD=1e30f;
        Vector3 bestV={0,0,0};
        for (int mi=0; mi<obj.model.meshCount; mi++) {
            Mesh& m=obj.model.meshes[mi];
            for (int vi=0; vi<m.vertexCount; vi++) {
                float vx=m.vertices[vi*3], vy=m.vertices[vi*3+1], vz=m.vertices[vi*3+2];
                float d=(vx-cx)*(vx-cx)+(vy-cy)*(vy-cy)+(vz-cz)*(vz-cz);
                if(d<bestD){bestD=d; bestV={vx,vy,vz};}
            }
        }

        // Dedup: skip if too close to an existing anchor (< 1 cm)
        bool dup=false;
        for (auto& a : obj.anchors)
            if(fabsf(a.x-bestV.x)<0.01f&&fabsf(a.y-bestV.y)<0.01f&&fabsf(a.z-bestV.z)<0.01f)
                {dup=true;break;}
        if(!dup) obj.anchors.push_back(bestV);
    }
}

static void ResetPhysObj(PhysicsObj& obj)
{
    btTransform t; t.setIdentity(); t.setOrigin(obj.spawnPos);
    obj.body->setCollisionFlags(obj.body->getCollisionFlags()&~btCollisionObject::CF_KINEMATIC_OBJECT);
    obj.body->forceActivationState(ACTIVE_TAG);
    obj.body->setWorldTransform(t); obj.body->getMotionState()->setWorldTransform(t);
    obj.body->setLinearVelocity(btVector3(0,0,0)); obj.body->setAngularVelocity(btVector3(0,0,0));
    obj.body->setDamping(obj.linDamp,obj.angDamp); obj.body->activate(true);
}

static void SyncTransform(PhysicsObj& obj)
{
    btTransform bt; obj.body->getMotionState()->getWorldTransform(bt);
    // Shift the visual position to compensate for the COM offset
    btVector3 pos=bt.getOrigin() + bt.getBasis() * obj.comOffset;
    btMatrix3x3 rot=bt.getBasis();
    Matrix R;
    R.m0=(float)rot[0][0];R.m1=(float)rot[1][0];R.m2=(float)rot[2][0];R.m3=0;
    R.m4=(float)rot[0][1];R.m5=(float)rot[1][1];R.m6=(float)rot[2][1];R.m7=0;
    R.m8=(float)rot[0][2];R.m9=(float)rot[1][2];R.m10=(float)rot[2][2];R.m11=0;
    R.m12=0;R.m13=0;R.m14=0;R.m15=1;
    btVector3 c=obj.centroid;
    obj.model.transform=MatrixMultiply(MatrixMultiply(
        MatrixTranslate(-(float)c.x(),-(float)c.y(),-(float)c.z()),R),
        MatrixTranslate((float)pos.x(),(float)pos.y(),(float)pos.z()));
}

// =============================================================================
//  INPUT SYSTEM (Unified Keyboard + Gamepad)
// =============================================================================
struct InputState {
    float throttle = 0.0f;  // -1.0 to 1.0 (forward/reverse)
    float steering = 0.0f;  // -1.0 to 1.0 (left/right)
    float brake    = 0.0f;  //  0.0 to 1.0 (pressure)
    bool  reset    = false;
    bool  toggleBBox = false;
    bool  toggleCOM  = false;
    bool  toggleCamera = false;

    // Movement for object picking (when a piece is held)
    btVector3 pickMove = {0,0,0}; 
};

static int gRawSignalCount = 0;

static void PollInput(InputState& state, float dt)
{
    state.throttle = 0.0f;
    state.steering = 0.0f;
    state.brake    = 0.0f;
    state.reset    = IsKeyPressed(KEY_R);
    state.toggleBBox = IsKeyPressed(KEY_B);
    state.toggleCOM  = IsKeyPressed(KEY_V);
    state.toggleCamera = IsKeyPressed(KEY_C);
    state.pickMove = {0,0,0};

    // ── DRIVER FLAPPING DIAGNOSTIC ───────────────────────────────────────────
    static char lastDetectedName[128] = "";
    static int  lastDetectedSlot = -1;
    static float flapTimer = 0;
    
    float rawSteer=0, rawGas=0, rawBrak=0;
    bool anyInput = false;
    static float controllerPriorityTimer = 0.0f;

    // ── NATIVE RAYLIB HOOKS ──────────────────────────────────────────────────
    for (int s=0; s<16; s++) {
        if (!IsGamepadAvailable(s)) continue;
        
        // Read Axes regardless of reported AxisCount
        for (int a=0; a<8; a++) {
            float v = GetGamepadAxisMovement(s, a);
            if (fabsf(v) > 0.03f) { 
                if (gRawSignalCount == 0) TraceLog(LOG_INFO, "[GAMEPAD] SIGNAL ON AXIS %d", a);
                gRawSignalCount++;
                anyInput = true;
                
                // Left Stick X -> Steering
                if (a == GAMEPAD_AXIS_LEFT_X) rawSteer = v; 

                // Right Trigger -> Gas (Normalize from [-1, 1] to [0, 1])
                if (a == GAMEPAD_AXIS_RIGHT_TRIGGER) {
                    if (v > -0.98f) rawGas = (v + 1.0f) / 2.0f;
                }
                
                // Left Trigger -> Brake 
                if (a == GAMEPAD_AXIS_LEFT_TRIGGER) {
                    if (v > -0.98f) rawBrak = (v + 1.0f) / 2.0f;
                }
            }
        }
        
        // Read Buttons (Fallback if triggers act as buttons)
        for (int b=0; b<32; b++) {
            if (IsGamepadButtonDown(s, b)) {
                if (gRawSignalCount == 0) TraceLog(LOG_INFO, "[GAMEPAD] SIGNAL ON BUTTON %d", b);
                gRawSignalCount++;
                anyInput = true;
                
                // Fallbacks: If triggers are picked up as buttons
                if (b == GAMEPAD_BUTTON_RIGHT_TRIGGER_2) rawGas = 1.0f;
                if (b == GAMEPAD_BUTTON_LEFT_TRIGGER_2) rawBrak = 1.0f;
                
                // Face buttons as alternatives
                if (b == GAMEPAD_BUTTON_RIGHT_FACE_DOWN) rawGas = 1.0f;  // A button -> Gas
                if (b == GAMEPAD_BUTTON_RIGHT_FACE_LEFT) rawBrak = 1.0f; // X button -> Brake
            }
        }
    }
    
    // ── Input Spoofing (Hold SHIFT to simulate Gamepad signals) ────────────
    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
        anyInput = true;
        if (IsKeyDown(KEY_W)) rawGas = 1.0f;
        if (IsKeyDown(KEY_S)) rawBrak = 1.0f;
        if (IsKeyDown(KEY_A)) rawSteer = -1.0f;
        if (IsKeyDown(KEY_D)) rawSteer = 1.0f;
    }
    
    if (anyInput) controllerPriorityTimer = 2.0f;

    if (controllerPriorityTimer > 0) controllerPriorityTimer -= dt;

    // ── Keyboard ─────────────────────────────────────────────────────────────
    if (controllerPriorityTimer <= 0.0f) {
        if (IsKeyDown(KEY_W)) state.throttle += 1.0f;
        if (IsKeyDown(KEY_S)) state.throttle -= 1.0f;
        if (IsKeyDown(KEY_A)) state.steering -= 1.0f;
        if (IsKeyDown(KEY_D)) state.steering += 1.0f;
        if (IsKeyDown(KEY_SPACE)) state.brake = 1.0f;
    } else {
        // Controller taking over
        state.steering = rawSteer;
        state.brake = rawBrak;
        // If braking heavily and not gassing, apply reverse throttle
        if (rawBrak > 0.1f && rawGas < 0.1f) {
            state.throttle = -rawBrak;
        } else {
            state.throttle = rawGas;
        }
    }

    // Clamp values
    if (state.throttle > 1.0f) state.throttle = 1.0f;
    if (state.throttle < -1.0f) state.throttle = -1.0f;
    if (state.steering > 1.0f) state.steering = 1.0f;
    if (state.steering < -1.0f) state.steering = -1.0f;
}

// =============================================================================
//  VISUAL HELPERS
// =============================================================================
static void DrawSkyAndFloor()
{
    int W=GetScreenWidth(),H=GetScreenHeight(),hH=H/2;
    ClearBackground(BLACK);
    DrawRectangleGradientV(0,  0,W,hH,{25,25,112,255},{135,206,235,255});
    DrawRectangleGradientV(0,hH,W,hH,{135,206,235,255},{255,200,150,255});
}
static void DrawWorldFloor()
{
    DrawPlane({0,0,0},{160,160},{38,42,52,255});
    Color g={60,70,95,100};
    for(int i=-70;i<=70;i++){
        DrawLine3D({(float)i,.001f,-70},{(float)i,.001f,70},g);
        DrawLine3D({-70,.001f,(float)i},{70,.001f,(float)i},g);
    }
}
static void DrawAxes(){
    DrawLine3D({0,0,0},{0.8f,0,0},RED);
    DrawLine3D({0,0,0},{0,0.8f,0},GREEN);
    DrawLine3D({0,0,0},{0,0,0.8f},BLUE);
}

// =============================================================================
//  PICKING HELPERS
// =============================================================================
static bool RayHitsObj(const Camera3D& cam,const PhysicsObj& obj,Vector3* out=nullptr)
{
    Ray ray=GetMouseRay(GetMousePosition(),cam);
    for(int mi=0;mi<obj.model.meshCount;mi++){
        RayCollision rc=GetRayCollisionMesh(ray,obj.model.meshes[mi],obj.model.transform);
        if(rc.hit){if(out)*out=rc.point;return true;}
    }
    return false;
}
// Nearest anchor (pre-sampled, LOCAL space) transformed to world space.
// O(16) per call regardless of mesh complexity.
static void NearestVtx(const PhysicsObj& obj, Vector3 wp, int& outVI, Vector3& outVW)
{
    float best=1e30f; outVI=-1; outVW=wp;
    for (int i=0; i<(int)obj.anchors.size(); i++) {
        Vector3 vW=Vector3Transform(obj.anchors[i], obj.model.transform);
        float d=Vector3Distance(wp, vW);
        if(d<best){best=d; outVI=i; outVW=vW;}
    }
}
static btPoint2PointConstraint* GrabObj(PhysicsObj& obj,Vector3 vWorld)
{
    btVector3 vW(vWorld.x,vWorld.y,vWorld.z);
    btVector3 lp=obj.body->getWorldTransform().inverse()*vW;
    auto* c=new btPoint2PointConstraint(*obj.body,lp);
    c->m_setting.m_impulseClamp=obj.mass*5.0f;
    c->m_setting.m_tau=0.001f;
    gWorld->addConstraint(c,true);
    obj.body->setDamping(obj.linDampHeld,obj.angDampHeld);
    obj.body->setActivationState(DISABLE_DEACTIVATION);
    obj.body->activate(true);
    return c;
}
static void ReleaseObj(PhysicsObj& obj,btPoint2PointConstraint*& c)
{
    if(!c)return;
    gWorld->removeConstraint(c);delete c;c=nullptr;
    obj.body->setDamping(obj.linDamp,obj.angDamp);
    obj.body->forceActivationState(ACTIVE_TAG);obj.body->activate(true);
}
// Draw only the pre-sampled anchors (max 16 spheres) + glow/pivot overlays.
// All O(16) — no per-vertex loops in the render path.
static void DrawVtxViz(const PhysicsObj& obj, int hVI, Vector3 hVW,
                        bool held, Vector3 piv, Color dot, Color glow, Color pc)
{
    // Draw every anchor as a small dot
    for (int i=0; i<(int)obj.anchors.size(); i++) {
        Vector3 vW=Vector3Transform(obj.anchors[i], obj.model.transform);
        DrawSphere(vW, 0.055f, dot);
    }
    if(!held&&hVI>=0){
        float p=0.5f+0.5f*sinf((float)GetTime()*9.f);
        float r=0.07f+0.05f*p;
        DrawSphere(hVW,r,{glow.r,glow.g,glow.b,(unsigned char)(160+95*p)});
        DrawSphere(hVW,r*1.8f,{glow.r,glow.g,glow.b,(unsigned char)(50*p)});
        DrawCircle3D(hVW,r*5.f,{1,0,0},90.f,{glow.r,glow.g,glow.b,(unsigned char)(80+80*p)});
    }
    if(held){
        float p=0.5f+0.5f*sinf((float)GetTime()*6.f);
        DrawSphere(piv,0.12f+0.04f*p,{pc.r,pc.g,pc.b,(unsigned char)(180+75*p)});
        DrawLine3D(piv,hVW,{pc.r,pc.g,pc.b,180});
    }
}

// =============================================================================
//  SCENE GLOBALS
// =============================================================================
static PhysicsObj gConcrete;
static PhysicsObj gCarBody;                            // visible steel car chassis
static PhysicsObj gWheels[4];                          // FL FR RL RR
static btRigidBody*                  gChassisBody=nullptr;  // invisible chassis frame
static btCollisionShape*             gChassisShape=nullptr;
static btGeneric6DofSpring2Constraint* gSusp[4]={};    // per-wheel spring
static btFixedConstraint*            gCarWeldC=nullptr;  // welds car body to chassis

// =============================================================================
//  BUILD THE INVISIBLE SUSPENSION RIG
//
//  Architecture — mirrors a real car suspension:
//
//    INVISIBLE CHASSIS (80 kg box, no visible model)
//         │         │         │         │
//     Spring     Spring     Spring     Spring     ← btGeneric6DofSpring2Constraint
//         │         │         │         │
//        FL        FR        RL        RR         ← 15 kg tire bodies
//
//  Per-constraint DOF table (constraint frame: X=axle, Y=up, Z=forward)
//  ┌─────────────┬─────────────────────────────────────────────────────────┐
//  │ Linear  X   │ LOCKED  — wheel cannot slide sideways along axle        │
//  │ Linear  Y   │ SPRING  — suspension travel ±0.15 m, stiff=80, damp=8  │
//  │ Linear  Z   │ LOCKED  — wheel cannot move fore/aft off attachment     │
//  │ Angular X   │ FREE    — tire spins/rolls around axle (natural)         │
//  │ Angular Y   │ LOCKED  — no steering yaw  (both axles locked for now)  │
//  │ Angular Z   │ LOCKED  — no camber / tilt — wheel always stays upright  │
//  └─────────────┴─────────────────────────────────────────────────────────┘
// =============================================================================
static void BuildSuspension(const btVector3 wheelSpawns[4])
{
    // Chassis centre: centre of the four wheel spawn XZ, same Y
    btVector3 centre(0, wheelSpawns[0].y(), 0);

    // Invisible thin cross-frame spanning the wheel formation
    gChassisShape = new btBoxShape(btVector3(1.3f, 0.08f, 1.8f));
    btTransform t; t.setIdentity(); t.setOrigin(centre);
    gChassisBody = MakeRigidBody(CHASSIS_MASS, t, gChassisShape);
    gChassisBody->setFriction(0.0f);
    gChassisBody->setDamping(0.08f, 0.30f);

    for (int i = 0; i < 4; i++)
    {
        // ── Attachment pivot on chassis (in chassis local space) ──────────
        btVector3 chassisLocalPivot = wheelSpawns[i] - centre;

        // ── Attachment pivot on wheel (at wheel body centre = origin) ─────
        btVector3 wheelLocalPivot(0, 0, 0);

        // ── Constraint frames ─────────────────────────────────────────────
        // We align the frame so that:
        //   frame X axis = world X (left-right)  → the spin / axle axis
        //   frame Y axis = world Y (up)           → suspension travel direction
        //   frame Z axis = world Z (forward)      → must stay locked
        // Both frames use identity rotation (chassis and wheels spawn identity).
        btTransform frameA, frameB;
        frameA.setIdentity(); frameA.setOrigin(chassisLocalPivot);
        frameB.setIdentity(); frameB.setOrigin(wheelLocalPivot);

        auto* s = new btGeneric6DofSpring2Constraint(
            *gChassisBody, *gWheels[i].body, frameA, frameB);

        // ── Linear DOFs ───────────────────────────────────────────────────
        // X=locked, Z=locked → wheel cannot drift off its attachment point.
        // Y=±0.20 m suspension travel (increased from 0.15 to handle car body sag).
        s->setLinearLowerLimit(btVector3(0.f, -0.20f, 0.f));
        s->setLinearUpperLimit(btVector3(0.f,  0.20f, 0.f));

        // ── Angular DOFs ──────────────────────────────────────────────────
        // In btGeneric6Dof: lower > upper = FREE, lower = upper = LOCKED, lower < upper = LIMITED.
        //
        //   X (spin / roll):    FREE   — tire rolls naturally around the axle
        //   Z (camber / tilt):  LOCKED — wheel stays perfectly upright
        //   Y (steering yaw):   LOCKED for rear axle; LIMITED+MOTOR for front axle
        if (i < 2) {
            // FRONT wheels: ±34° steering range (≈0.60 rad) with servo motor
            s->setAngularLowerLimit(btVector3( 1.f, -0.60f, 0.f));  // X free, Y limited, Z locked
            s->setAngularUpperLimit(btVector3(-1.f,  0.60f, 0.f));
            // DOF index 4 = angular Y.  Servo drives it toward setServoTarget.
            s->enableMotor(4, true);
            s->setServo(4, true);
            s->setMaxMotorForce(4, STEER_SERVO_FORCE);
            s->setTargetVelocity(4, STEER_SERVO_SPEED);
            s->setServoTarget(4, 0.0f);      // start straight ahead
        } else {
            // REAR wheels: Y fully locked — they never steer
            s->setAngularLowerLimit(btVector3( 1.f, 0.f, 0.f));  // X free, Y/Z locked
            s->setAngularUpperLimit(btVector3(-1.f, 0.f, 0.f));
        }

        // ── Suspension spring on linear-Y (DOF index 1) ───────────────────
        // Sprung mass = 80 kg (frame) + 700 kg (car body) = 780 kg.
        // Per corner: 780/4 × 9.81 = 1913 N.  Stiffness 18000 N/m → ~10.6 cm sag.
        // Natural freq: (1/2π)√(18000/195) ≈ 1.53 Hz (real cars: 1–2 Hz). 
        // Critical damp: 2√(18000×195) = 3742 Ns/m.  We use ~0.32 of critical.
        s->enableSpring(1, true);
        s->setStiffness(1, 18000.0f);
        s->setDamping  (1,  1200.0f);
        s->setEquilibriumPoint();      // rest at zero offset (spawn height)

        gWorld->addConstraint(s, /*disableCollisionBetweenLinked=*/true);
        gSusp[i] = s;
    }
}

static void DestroySuspension()
{
    for (int i=0;i<4;i++) {
        if (gSusp[i]) { gWorld->removeConstraint(gSusp[i]); delete gSusp[i]; gSusp[i]=nullptr; }
    }
    if (gChassisBody) {
        if (gChassisBody->getMotionState()) delete gChassisBody->getMotionState();
        gWorld->removeRigidBody(gChassisBody); delete gChassisBody; gChassisBody=nullptr;
    }
    delete gChassisShape; gChassisShape=nullptr;
}

// Reset the full rig: chassis + all 4 wheels
static void ResetRig(const btVector3 wheelSpawns[4])
{
    btVector3 centre(0, wheelSpawns[0].y(), 0);
    btTransform tc; tc.setIdentity(); tc.setOrigin(centre);
    gChassisBody->forceActivationState(ACTIVE_TAG);
    gChassisBody->setWorldTransform(tc);
    gChassisBody->getMotionState()->setWorldTransform(tc);
    gChassisBody->setLinearVelocity(btVector3(0,0,0));
    gChassisBody->setAngularVelocity(btVector3(0,0,0));
    gChassisBody->activate(true);
    for (int i=0; i<4; i++) {
        btTransform tw; tw.setIdentity(); tw.setOrigin(wheelSpawns[i]);
        gWheels[i].body->forceActivationState(ACTIVE_TAG);
        gWheels[i].body->setWorldTransform(tw);
        gWheels[i].body->getMotionState()->setWorldTransform(tw);
        gWheels[i].body->setLinearVelocity(btVector3(0,0,0));
        gWheels[i].body->setAngularVelocity(btVector3(0,0,0));
        gWheels[i].body->activate(true);
    }
    // Re-equilibrate springs at current position
    for (int i=0;i<4;i++) if(gSusp[i]) gSusp[i]->setEquilibriumPoint();
}

// =============================================================================
//  MAIN
// =============================================================================
int main()
{
    ChangeDirectory("/Users/rajveersahay/C ++ test/Car-Racing-Game");
    InitWindow(1280, 720, "Physics Sandbox — Concrete + Suspended Tire Rig");
    
    SetTargetFPS(60);
    InitAudioDevice();
    InitBullet();

    // =========================================================================
    //  ENGINE AUDIO — Pseudo-Granular Blender
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
        SetSoundVolume(engSounds[i], 0.0f);
        PlaySound(engSounds[i]);
    }

    bool audioEnabled = true;
    float currentRPM        = 1500.0f;
    float rawRPM            = 1500.0f;
    const float RPM_IDLE    = 1500.0f;
    const float RPM_MAX     = 7000.0f;
    const float RPM_OMEGA_SCALE = 220.0f;

    // ── Ground ───────────────────────────────────────────────────────────────
    {
        auto* shape=new btStaticPlaneShape(btVector3(0,1,0),0);
        btTransform t;t.setIdentity();
        auto* b=MakeRigidBody(0.f,t,shape);
        // Ground friction = 1.0 so the tire's dynamic friction IS the contact friction.
        // (Bullet uses geometric mean: sqrt(1.0 * tire_friction) = tire_friction exactly)
        b->setFriction(2.2f); b->setRestitution(0.24f);
    }

    // ── Concrete ramp ────────────────────────────────────────────────────────
    gConcrete.mass=150.f; gConcrete.friction=0.70f; gConcrete.restitution=0.04f;
    gConcrete.linDamp=0.15f; gConcrete.angDamp=0.70f;
    gConcrete.linDampHeld=0.25f; gConcrete.angDampHeld=0.97f;
    gConcrete.baseColor={200,190,175,255};
    gConcrete.label="CONCRETE RAMP  150 kg";
    if (!LoadPhysObj(gConcrete,"Obj Files/runway.obj")) {
        TraceLog(LOG_ERROR,"runway.obj missing"); return 1;
    }
    ComputeAnchors(gConcrete);          // pre-build 16 pick anchors
    gConcrete.spawnPos=btVector3(-7.f, 5.f, 0.f);
    ResetPhysObj(gConcrete);

    // ── Four tires in car formation — positions come from the ALIGNMENT BLOCK at top ─
    //  Formation (front = +Z, rear = -Z, left = -X, right = +X):
    //     FL (-TRACK, H,  BASE)    FR (+TRACK, H,  BASE)
    //     RL (-TRACK, H, -BASE)    RR (+TRACK, H, -BASE)
    struct TireDef { const char* path; const char* label; btVector3 spawn; };
    TireDef td[4]={
        {"Obj Files/wheel_fl.obj","FL TIRE 15kg",{-WHEEL_TRACK, WHEEL_HEIGHT,  WHEEL_BASE}},
        {"Obj Files/wheel_fr.obj","FR TIRE 15kg",{ WHEEL_TRACK, WHEEL_HEIGHT,  WHEEL_BASE}},
        {"Obj Files/wheel_rl.obj","RL TIRE 15kg",{-WHEEL_TRACK, WHEEL_HEIGHT, -WHEEL_BASE}},
        {"Obj Files/wheel_rr.obj","RR TIRE 15kg",{ WHEEL_TRACK, WHEEL_HEIGHT, -WHEEL_BASE}},
    };

    btVector3 wheelSpawns[4];
    for (int i=0;i<4;i++) {
        gWheels[i].mass=WHEEL_MASS; gWheels[i].friction=TIRE_GRIP_IDLE; gWheels[i].restitution=0.30f;
        gWheels[i].linDamp=0.12f; gWheels[i].angDamp=0.35f;
        gWheels[i].linDampHeld=0.20f; gWheels[i].angDampHeld=0.90f;
        gWheels[i].baseColor={30,30,35,255};   // dark rubber
        gWheels[i].label=td[i].label;
        if (!LoadPhysObj(gWheels[i], td[i].path, /*useWheelCylinder=*/true)){
            TraceLog(LOG_ERROR,"Missing %s",td[i].path); return 1;
        }
        // Extract the true cylinder radius from the collision shape so our slip
        // calculation uses the physically correct wheel-to-ground contact speed.
        if (auto* cyl = dynamic_cast<btCylinderShapeX*>(gWheels[i].shape)) {
            // halfExtents.y() = radius of the cylinder in the YZ plane
            gWheels[i].wheelRadius = (float)cyl->getHalfExtentsWithMargin().getY();
        }
        ComputeAnchors(gWheels[i]);     // pre-build 16 pick anchors
        gWheels[i].spawnPos=td[i].spawn;
        ResetPhysObj(gWheels[i]);
        wheelSpawns[i]=td[i].spawn;
    }

    // ── Build invisible suspension rig ────────────────────────────────────────
    BuildSuspension(wheelSpawns);

    // ── Visible car body — 700 kg steel, rigidly bolted to chassis frame ─────
    // Architecture:  [car.obj body] ──btFixedConstraint──► [gChassisBody]
    //                                                           │
    //                                              suspension springs (×4)
    //                                                           │
    //                                                      [4 × tire]
    //                                                           │
    //                                                        [ground]
    // The 700 kg car body weight transfers through the fixed constraint
    // into the chassis, compresses the suspension springs, and loads the tires.
    // Net sprung mass per tire spring: (80+700)/4 = 195 kg.
    {
        gCarBody.mass=CAR_BODY_MASS; gCarBody.friction=0.40f; gCarBody.restitution=0.10f;
        gCarBody.linDamp=0.05f; gCarBody.angDamp=0.20f;
        gCarBody.linDampHeld=0.15f; gCarBody.angDampHeld=0.90f;
        gCarBody.baseColor={55,65,75,255};  // dark steel
        gCarBody.label="CAR BODY  700 kg  (steel)";
        if (!LoadPhysObj(gCarBody,"Obj Files/car.obj", false, CAR_COM_OFFSET_Y, CAR_COM_OFFSET_Z)) {
            TraceLog(LOG_ERROR,"car.obj missing"); return 1;
        }
        // Car body has FULL collision — convex hull contacts the floor/objects normally.
        // During normal driving it sits above the wheels so no floor contact occurs.
        // When flipped, it correctly rests on the ground instead of clipping through.
        // No flag needed — LoadPhysObj already creates a standard dynamic rigid body.

        // Place car body at chassis position + physics offset, then weld with btFixedConstraint.
        // CAR_PHYS_OFF_* is the pivot in CHASSIS local space where the car body anchors.
        // SyncTransform reads gCarBody.body's world transform — so visual follows automatically.
        btTransform ct; gChassisBody->getMotionState()->getWorldTransform(ct);
        btVector3 physOff(CAR_PHYS_OFF_X, CAR_PHYS_OFF_Y, CAR_PHYS_OFF_Z);
        
        // Compensate for the COM shift so the visual body remains at the exact same location
        physOff -= gCarBody.comOffset;
        
        btTransform carInitT = ct;
        carInitT.setOrigin(ct.getOrigin() + ct.getBasis() * physOff);
        gCarBody.body->setWorldTransform(carInitT);
        gCarBody.body->getMotionState()->setWorldTransform(carInitT);
        // Constraint: fA.origin = physOff (pivot in chassis local space)
        //             fB.origin = (0,0,0) (pivot at car body's own centre)
        btTransform fA, fB;
        fA.setIdentity(); fA.setOrigin(physOff);
        fB.setIdentity();
        gCarWeldC = new btFixedConstraint(*gChassisBody, *gCarBody.body, fA, fB);
        gWorld->addConstraint(gCarWeldC, /*disableCollision=*/true);

        // ───────────────────────────────────────────────────────────────────
        // CRITICAL: disable direct collision between car body and all tires.
        // The car.obj convex hull includes wheel arch geometry that OVERLAPS the
        // tire cylinder shapes, causing massive jitter/explosion if they collide.
        // The suspension constraints handle all car–tire positional relationships;
        // direct Bullet collision between them is harmful, not helpful.
        // ───────────────────────────────────────────────────────────────────
        for (int i = 0; i < 4; i++) {
            gCarBody.body->setIgnoreCollisionCheck(gWheels[i].body, true);
            gWheels[i].body->setIgnoreCollisionCheck(gCarBody.body, true);
        }
        // Also skip car body vs invisible chassis (btFixedConstraint disables it, but make explicit)
        gCarBody.body->setIgnoreCollisionCheck(gChassisBody, true);
        gChassisBody->setIgnoreCollisionCheck(gCarBody.body, true);
    }

    // ── Pickable object roster (concrete + 4 tires — car body/chassis invisible) ─
    PhysicsObj* allObjs[5]={&gConcrete,&gWheels[0],&gWheels[1],&gWheels[2],&gWheels[3]};
    const int nObjs=5;

    // ── Camera ────────────────────────────────────────────────────────────────
    float camYaw=30.f,camPitch=22.f,camDist=22.f,camDistTgt=22.f;
    Vector3 camFocus={0,0,0};

    // ── Picking state ─────────────────────────────────────────────────────────
    float moveSpeed=8.f;
    float steerAngle=0.f;               // current front-wheel steer angle (radians)
    float brakeForce=0.f;               // [SPACE] pressure 0=off 1=full (ramps smoothly)
    const float MAX_STEER = MAX_STEER_ANGLE;  // from tuning block at top
    btPoint2PointConstraint* pickC=nullptr;
    PhysicsObj* pickedObj=nullptr;
    btVector3   pickPivot;
    Vector3     pickVtx={0,0,0};
    bool        isPicking=false;
    PhysicsObj* hoveredObj=nullptr;
    int         hoverVI=-1;
    Vector3     hoverVW={0,0,0};
    bool        showBBox=false;   // [B] toggles physics AABB wireframe debug overlay
    bool        showCOM =false;   // [V] toggles Center of Mass sphere

    // ==========================================================================
    //  GAME LOOP
    // ==========================================================================
    InputState io;
    while (!WindowShouldClose())
    {
        float dt=GetFrameTime(); if(dt>0.05f)dt=0.05f;
        PollInput(io, dt);
        float scroll=GetMouseWheelMove();

        // =====================================================================
        //  ENGINE RPM CALCULATION + AUDIO UPDATE
        // =====================================================================
        if (IsKeyPressed(KEY_M)) {
            audioEnabled = !audioEnabled;
            if (!audioEnabled) {
                for (int i = 0; i < ENG_SAMPLE_COUNT; i++)
                    SetSoundVolume(engSounds[i], 0.0f);
            }
        }

        // Extract rear wheel spin to compute slip
        btRigidBody* rbRL = gWheels[2].body;
        btRigidBody* rbRR = gWheels[3].body;
        btVector3 axleRL = rbRL->getWorldTransform().getBasis() * btVector3(1,0,0);
        btVector3 axleRR = rbRR->getWorldTransform().getBasis() * btVector3(1,0,0);
        float spinRL = (float)(rbRL->getAngularVelocity().dot(axleRL));
        float spinRR = (float)(rbRR->getAngularVelocity().dot(axleRR));
        float rearSpinOmega = (fabsf(spinRL) + fabsf(spinRR)) * 0.5f;
        float wheelSpeedKmh = rearSpinOmega * gWheels[2].wheelRadius * 3.6f;

        bool throttleOn = (io.throttle > 0.0f);
        btTransform ctTgt; gChassisBody->getMotionState()->getWorldTransform(ctTgt);
        btVector3 fwdTgt = ctTgt.getBasis() * btVector3(0.f,0.f,1.f);
        float absSpeedKmh = fabsf((float)gChassisBody->getLinearVelocity().dot(fwdTgt)) * 3.6f;

        float blipRPM = RPM_IDLE;
        
        if (throttleOn) {
            // Gear simulation based on actual car speed instead of wheel spin
            const float speedPerGear = 35.0f; // km/h per gear
            int gear = (int)(absSpeedKmh / speedPerGear);
            if (gear > 5) gear = 5; // max 6 gears
            
            // Add wheel slip to the effective speed to make engine rev higher when tires are spinning/drifting
            float slipSpeedKmh = wheelSpeedKmh - absSpeedKmh;
            if (slipSpeedKmh < 0.0f) slipSpeedKmh = 0.0f;
            // 0.65x multiplier means spinning tires hard will bounce the RPM high into the rev range!
            float effectiveSpeedKmh = absSpeedKmh + slipSpeedKmh * 0.65f;
            
            float speedInGear = effectiveSpeedKmh - (gear * speedPerGear);
            float baseRPMForGear = (gear == 0) ? RPM_IDLE : 3800.0f;
            
            blipRPM = baseRPMForGear + (speedInGear / speedPerGear) * (RPM_MAX - baseRPMForGear);
            
            // Give it a minimum RPM purely based on throttle input so it sounds responsive immediately
            float minThrottleRPM = RPM_IDLE + io.throttle * (RPM_MAX - RPM_IDLE) * 0.45f;
            if (blipRPM < minThrottleRPM) blipRPM = minThrottleRPM;
            
        } else {
            // When off throttle, simulate disengaged clutch or idle coasting
            blipRPM = RPM_IDLE; 
            // Clamp coastal RPM to something low so it sounds like it's idling
            if (absSpeedKmh > 10.0f) blipRPM = 2200.0f; // fast idle when moving
        }

        if (blipRPM < RPM_IDLE) blipRPM = RPM_IDLE;
        if (blipRPM > RPM_MAX ) blipRPM = RPM_MAX;
        rawRPM = blipRPM;

        float rpmRiseRate = 12.0f;
        float rpmFallRate = 5.0f;
        float rpmRate = (rawRPM > currentRPM) ? rpmRiseRate : rpmFallRate;
        currentRPM += (rawRPM - currentRPM) * rpmRate * dt;

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

        // Camera orbit
        static bool useFollowCamera = false;
        if (io.toggleCamera) useFollowCamera = !useFollowCamera;

        if (!isPicking) {
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 d=GetMouseDelta();
                camYaw-=d.x*0.3f; camPitch+=d.y*0.3f;
            }
            if(camPitch<5.f)camPitch=5.f; if(camPitch>80.f)camPitch=80.f;
            camDistTgt-=scroll*2.f;
            if(camDistTgt<3.f)camDistTgt=3.f; if(camDistTgt>90.f)camDistTgt=90.f;
            camDist+=(camDistTgt-camDist)*8.f*dt;
        }
        
        Vector3 camTgt = camFocus;
        if (useFollowCamera && gChassisBody) {
            btTransform ct; gChassisBody->getMotionState()->getWorldTransform(ct);
            camTgt = { (float)ct.getOrigin().x(), 0.5f, (float)ct.getOrigin().z() };
        }

        float yr=camYaw*DEG2RAD, pr=camPitch*DEG2RAD;
        Camera3D cam={};
        cam.position={camTgt.x+camDist*cosf(pr)*sinf(yr),
                      camTgt.y+camDist*sinf(pr),
                      camTgt.z+camDist*cosf(pr)*cosf(yr)};
        cam.target=camTgt; cam.up={0,1,0}; cam.fovy=45.f;
        cam.projection=CAMERA_PERSPECTIVE;

        // Reset
        if (io.reset) {
            if(pickedObj) ReleaseObj(*pickedObj,pickC);
            isPicking=false; pickedObj=nullptr;
            ResetPhysObj(gConcrete);
            ResetRig(wheelSpawns);
            // Zero steering
            steerAngle=0.f;
            for (int fi=0;fi<2;fi++) if(gSusp[fi]) gSusp[fi]->setServoTarget(4,0.f);
        }

        // Hover test
        hoveredObj=nullptr; hoverVI=-1;
        if (!isPicking) {
            for (int i=0;i<nObjs;i++) {
                Vector3 hp;
                if (RayHitsObj(cam,*allObjs[i],&hp)) {
                    hoveredObj=allObjs[i];
                    NearestVtx(*hoveredObj,hp,hoverVI,hoverVW);
                    break;
                }
            }
        }

        // Grab
        if (!isPicking && hoveredObj && hoverVI>=0 &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            isPicking=true; pickedObj=hoveredObj;
            pickVtx=hoverVW;
            pickPivot=btVector3(hoverVW.x,hoverVW.y,hoverVW.z);
            pickC=GrabObj(*pickedObj,hoverVW);
        }

        // Drag
        if (isPicking && pickC && pickedObj) {
            Vector2 md=GetMouseDelta();
            pickPivot+=btVector3(0,-md.y*0.033f,0);
            if(pickPivot.y()<0.05f)pickPivot.setY(0.05f);
            // Arrow keys / D-Pad MOVE the held object in camera-relative XZ plane
            float spd=moveSpeed*dt;
            float cfX=sinf(yr),cfZ=cosf(yr),crX=cosf(yr),crZ=-sinf(yr);
            if(io.pickMove.z() < -0.1f) {pickPivot-=btVector3(cfX*spd,0,cfZ*spd);}
            if(io.pickMove.z() >  0.1f) {pickPivot+=btVector3(cfX*spd,0,cfZ*spd);}
            if(io.pickMove.x() < -0.1f) {pickPivot-=btVector3(crX*spd,0,crZ*spd);}
            if(io.pickMove.x() >  0.1f) {pickPivot+=btVector3(crX*spd,0,crZ*spd);}
            pickC->setPivotB(pickPivot);
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                ReleaseObj(*pickedObj,pickC);
                isPicking=false; pickedObj=nullptr;
            }
        }

        // ── Drive (WASD / Stick) + Steer (A/D / Stick) + Brake (SPACE / Trigger) ──
        // W/Stick Up = accelerate forward    S/Stick Down = reverse
        // A/Stick Left = steer left          D/Stick Right = steer right
        // SPACE/LT = brake (ramps up progressively, applies counter-torque to all 4 wheels)
        {
            // — Rear-wheel drive —
            float driveDir = io.throttle;
            if (driveDir != 0.f) {
                for (int wi = 2; wi <= 3; wi++) {
                    btRigidBody* wb   = gWheels[wi].body;
                    btVector3    axle = wb->getWorldTransform().getBasis() * btVector3(1,0,0);
                    float currSpin    = (float)(wb->getAngularVelocity().dot(axle));
                    if (fabsf(currSpin) < MAX_WHEEL_SPIN)
                        wb->applyTorqueImpulse(axle * (driveDir * DRIVE_TORQUE * dt));
                    wb->activate(true);
                }
                gChassisBody->activate(true);
            }

            // — Steering (Speed-sensitive + Smooth) —
            btTransform ctTgtSt; gChassisBody->getMotionState()->getWorldTransform(ctTgtSt);
            btVector3 fwdTgtSt = ctTgtSt.getBasis() * btVector3(0.f,0.f,1.f);
            float absSpeedKmhSteer = fabsf((float)gChassisBody->getLinearVelocity().dot(fwdTgtSt)) * 3.6f;

            float speedFactor  = 1.0f - P_STEER_SPEED_REDUCE * (absSpeedKmhSteer / 120.0f);
            if (speedFactor < 0.23f) speedFactor = 0.23f;
            float effectiveMax = MAX_STEER_ANGLE * speedFactor;

            float targetSteer = io.steering * effectiveMax;
            float steerRate = (fabsf(targetSteer) > 0.001f) ? P_STEER_IN_RATE : P_STEER_OUT_RATE;
            steerAngle += (targetSteer - steerAngle) * steerRate * dt;

            // Clamp
            if (steerAngle >  effectiveMax) steerAngle =  effectiveMax;
            if (steerAngle < -effectiveMax) steerAngle = -effectiveMax;

            if (gSusp[0]) gSusp[0]->setServoTarget(4, steerAngle);
            if (gSusp[1]) gSusp[1]->setServoTarget(4, steerAngle);

            // — Progressive braking —
            const float BRAKE_RAMP_UP   = 0.18f;  // per frame rate of build-up
            const float BRAKE_RAMP_DOWN = 0.12f;  // per frame rate of release
            if (io.brake > 0.1f) {
                brakeForce = fminf(brakeForce + BRAKE_RAMP_UP * io.brake, 1.0f);
            } else {
                brakeForce = fmaxf(brakeForce - BRAKE_RAMP_DOWN, 0.0f);
            }
            if (brakeForce > 0.01f) {
                for (int wi = 0; wi < 4; wi++) {
                    btRigidBody* wb   = gWheels[wi].body;
                    btVector3    axle = wb->getWorldTransform().getBasis() * btVector3(1,0,0);
                    float currSpin    = (float)(wb->getAngularVelocity().dot(axle));
                    if (fabsf(currSpin) > 0.01f)
                        wb->applyTorqueImpulse(axle * (-currSpin / fabsf(currSpin)) * (brakeForce * BRAKE_TORQUE * dt));
                    wb->activate(true);
                }
                gChassisBody->activate(true);
            }
        }

        // ── Pacejka-inspired slip-ratio tire friction ──────────────────────────────
        // Run EVERY frame before stepSimulation so Bullet sees the updated values.
        //
        //  slip = |wheelSpinSpeed - carGroundSpeed| / max(both, minSpeed)
        //
        //  slip  0.00 – 0.15 : friction ramps 0.65 → 1.05  (grip building up)
        //  slip  0.15 – 0.30 : peak friction 1.05           (optimal grip window)
        //  slip  0.30 – 1.00 : drops 1.05 → 0.35           (wheelspin / sliding)
        //  slip  >  1.00     : clamped minimum 0.35         (full burnout)
        //
        //  Front wheels are not torque-driven so they barely slip — held at 1.00.
        {
            // Chassis forward speed projected onto its own forward axis
            btTransform ct; gChassisBody->getMotionState()->getWorldTransform(ct);
            btVector3 chassisFwd = ct.getBasis() * btVector3(0.f, 0.f, 1.f);
            float vCar    = (float)gChassisBody->getLinearVelocity().dot(chassisFwd);
            float vCarAbs = fabsf(vCar);

            for (int wi = 0; wi < 4; wi++) {
                btRigidBody* wb   = gWheels[wi].body;
                btVector3    axle = wb->getWorldTransform().getBasis() * btVector3(1.f, 0.f, 0.f);
                float omega       = (float)wb->getAngularVelocity().dot(axle);
                float vSpin       = fabsf(omega) * gWheels[wi].wheelRadius;  // peripheral m/s

                float friction;
                if (wi >= 2) {
                    // DRIVEN rear wheels — dynamic slip-based friction
                    float denom = fmaxf(fmaxf(vCarAbs, vSpin), 0.30f);
                    float slip  = fminf(fabsf(vSpin - vCarAbs) / denom, 1.0f);

                    float gripRange = TIRE_GRIP_PEAK - TIRE_GRIP_IDLE;
                    if      (slip < TIRE_SLIP_ONSET)  friction = TIRE_GRIP_IDLE + (slip / TIRE_SLIP_ONSET) * gripRange;  // build
                    else if (slip < TIRE_SLIP_PEAK)   friction = TIRE_GRIP_PEAK;  // peak plateau
                    else                              { float t = (slip - TIRE_SLIP_PEAK) / (1.0f - TIRE_SLIP_PEAK);
                                                        friction = TIRE_GRIP_PEAK - t * (TIRE_GRIP_PEAK - TIRE_GRIP_SPIN); }  // drop
                    friction = fmaxf(friction, TIRE_GRIP_SPIN);
                } else {
                    // FRONT wheels — not driven, rolling contact, near peak always
                    friction = TIRE_FRONT_GRIP;
                }
                wb->setFriction(friction);
            }
        }

        // Physics
        gWorld->stepSimulation(dt,10,1.f/120.f);

        // Sync
        for(int i=0;i<nObjs;i++) SyncTransform(*allObjs[i]);
        SyncTransform(gCarBody);   // car body welded to chassis — sync its visual too

        // ── RENDER ────────────────────────────────────────────────────────────
        BeginDrawing();
            DrawSkyAndFloor();

            BeginMode3D(cam);
                DrawWorldFloor();
                DrawAxes();

                // Draw all pickable objects
                for (int i=0;i<nObjs;i++) {
                    PhysicsObj& obj=*allObjs[i];
                    bool held=(isPicking&&pickedObj==&obj);
                    bool hov=(hoveredObj==&obj&&!isPicking);
                    bool isConcrete=(&obj==&gConcrete);

                    Color c;
                    if (isConcrete){
                        c=held?Color{255,160,80,255}:hov?Color{255,240,160,255}:obj.baseColor;
                    } else {
                        c=held?Color{255,180,60,255}:hov?Color{80,80,95,255}:obj.baseColor;
                    }
                    DrawModel(obj.model,{0,0,0},1.f,c);
                    DrawModelWires(obj.model,{0,0,0},1.f,
                        isConcrete?Color{60,55,50,90}:Color{55,55,60,90});

                    if (hov||held) {
                        Vector3 piv=held?Vector3{(float)pickPivot.x(),(float)pickPivot.y(),(float)pickPivot.z()}:Vector3{0,0,0};
                        Color dot=isConcrete?Color{130,120,110,150}:Color{80,80,85,150};
                        Color glo=isConcrete?Color{255,100,0,255}:Color{0,200,255,255};
                        Color pc =isConcrete?Color{255,160,80,255}:Color{60,220,255,255};
                        DrawVtxViz(obj,hoverVI,hoverVW,held,piv,dot,glo,pc);
                    }
                }

                // Draw car body: visual rotation is applied in LOCAL space before world transform.
                // Position comes from SyncTransform (physics body) — no extra translate needed.
                {
                    Matrix carLocal =
                        MatrixMultiply(
                            MatrixRotateXYZ({CAR_PITCH_DEG*DEG2RAD, CAR_YAW_DEG*DEG2RAD, CAR_ROLL_DEG*DEG2RAD}),
                            gCarBody.model.transform);
                    Matrix savedT = gCarBody.model.transform;
                    gCarBody.model.transform = carLocal;
                    DrawModel(gCarBody.model,{0,0,0},1.f,WHITE);   // show texture
                    DrawModelWires(gCarBody.model,{0,0,0},1.f,{40,50,60,100});
                    gCarBody.model.transform = savedT;
                }

                // ── Draw invisible suspension as coloured spring-lines ───────
                // Shows the suspension connections so users understand what's
                // happening even though the chassis is invisible.
                {
                    btTransform bt; gChassisBody->getMotionState()->getWorldTransform(bt);
                    btVector3 cc=bt.getOrigin();
                    Vector3 chassisCentre={(float)cc.x(),(float)cc.y(),(float)cc.z()};

                    for (int i=0;i<4;i++) {
                        btTransform wt; gWheels[i].body->getMotionState()->getWorldTransform(wt);
                        btVector3 wp=wt.getOrigin();
                        Vector3 wheelCtr={(float)wp.x(),(float)wp.y(),(float)wp.z()};

                        // Compress/stretch colour: green=at rest, red=compressed, blue=extended
                        float offset=wp.y()-cc.y()-(wheelSpawns[i].y()-wheelSpawns[0].y()+0.f);
                        float t01=(offset+0.20f)/0.40f; // 0=compressed, 0.5=rest, 1=extended
                        unsigned char rr=(unsigned char)(255*(1.f-t01));
                        unsigned char gg=(unsigned char)(200);
                        unsigned char bb=(unsigned char)(255*t01);
                        Color springCol={rr,gg,bb,220};

                        DrawLine3D(chassisCentre, wheelCtr, springCol);
                        // Small dot at chassis attachment point
                        DrawSphere(chassisCentre, 0.06f, {200,200,200,180});
                    }

                    // Chassis outline (thin rectangle showing the frame footprint)
                    Color frameCol={80,100,80,100};
                    DrawLine3D({-1.1f,(float)cc.y(), 1.6f},{ 1.1f,(float)cc.y(), 1.6f},frameCol);
                    DrawLine3D({-1.1f,(float)cc.y(),-1.6f},{ 1.1f,(float)cc.y(),-1.6f},frameCol);
                    DrawLine3D({-1.1f,(float)cc.y(), 1.6f},{-1.1f,(float)cc.y(),-1.6f},frameCol);
                    DrawLine3D({ 1.1f,(float)cc.y(), 1.6f},{ 1.1f,(float)cc.y(),-1.6f},frameCol);
                }

                // ── Wheel direction arrows ─────────────────────────────────────
                // Arrows use CHASSIS orientation + steerAngle, NOT the rolling wheel
                // body transform.  The wheel body spins around X as it rolls, so its
                // local-Z sweeps constantly — reading it causes the flicker.
                // Instead: rear arrow = chassis forward; front arrow = chassis forward
                // rotated by steerAngle around chassis up (Rodrigues' formula).
                {
                    btMatrix3x3 cb       = gChassisBody->getWorldTransform().getBasis();
                    btVector3   cFwd     = cb * btVector3(0.f, 0.f, 1.f);  // chassis +Z
                    btVector3   cRight   = cb * btVector3(1.f, 0.f, 0.f);  // chassis +X
                    // Front-wheel direction = cFwd rotated by steerAngle around chassis Y
                    // Rodrigues (cRight ⊥ cFwd): fwd' = fwd*cos + right*sin
                    btVector3 frontBt = cFwd * btCos(btScalar(steerAngle))
                                      - cRight * btSin(btScalar(steerAngle));

                    bool driving = fabsf(io.throttle) > 0.1f;

                    for (int i = 0; i < 4; i++) {
                        btTransform wt; gWheels[i].body->getMotionState()->getWorldTransform(wt);
                        btVector3 wp = wt.getOrigin();
                        Vector3 wc = {(float)wp.x(), (float)wp.y(), (float)wp.z()};

                        bool isRear  = (i >= 2);
                        bool isFront = !isRear;
                        bool isSteering = isFront && (fabsf(io.steering) > 0.1f);

                        // Pick which direction vector to use
                        btVector3 dir = isRear ? cFwd : frontBt;
                        Vector3 d = {(float)dir.x(), 0.f, (float)dir.z()};
                        float dlen = sqrtf(d.x*d.x + d.z*d.z);
                        if (dlen > 0.001f){ d.x/=dlen; d.z/=dlen; }

                        Color arrowCol = (isRear && driving) ? Color{80,255,80,255}   // driven rear: bright green
                                       : isSteering          ? Color{255,200,60,255}  // steering front: yellow
                                       : isRear              ? Color{120,200,120,160} // idle rear: dim green
                                                             : Color{120,180,255,160};// idle front: blue

                        Vector3 tip = {wc.x + d.x*0.9f, wc.y, wc.z + d.z*0.9f};
                        DrawLine3D(wc, tip, arrowCol);
                        DrawSphere(tip, 0.08f, arrowCol);
                    }
                }

                // ── Physics AABB debug overlay (press B to toggle) ──────────────────
                // Shows the Bullet AABB for every dynamic body in the world.
                // Useful for diagnosing overlap / tunnelling issues.
                if (io.toggleBBox) showBBox = !showBBox;
                if (showBBox) {
                    int n = gWorld->getNumCollisionObjects();
                    for (int oi = 0; oi < n; oi++) {
                        btCollisionObject* co = gWorld->getCollisionObjectArray()[oi];
                        btVector3 mn, mx;
                        gWorld->getBroadphase()->getAabb(
                            co->getBroadphaseHandle(), mn, mx);
                        BoundingBox bb = {
                            {(float)mn.x(), (float)mn.y(), (float)mn.z()},
                            {(float)mx.x(), (float)mx.y(), (float)mx.z()}
                        };
                        // Colour: red = car body, green = tires, white = chassis, cyan = concrete
                        Color bc = WHITE;
                        if      (co == gCarBody.body)   bc = {255, 60, 60, 200};
                        else if (co == gChassisBody)    bc = {255,255, 80, 200};
                        else if (co == gConcrete.body)  bc = {80, 200,255, 200};
                        else {
                            for (int wi=0;wi<4;wi++) if (co==gWheels[wi].body) bc={80,255,80,200};
                        }
                        DrawBoundingBox(bb, bc);
                    }
                }

                if (io.toggleCOM) showCOM = !showCOM;
                if (showCOM) {
                    // Calculate true combined Center of Mass of the entire car assembly
                    btVector3 combinedCOM(0,0,0);
                    float combinedMass = 0.0f;
                    
                    auto addBodyCOM = [&](btRigidBody* rb) {
                        float m = (rb->getInvMass() > 0.0f) ? (1.0f / rb->getInvMass()) : 0.0f;
                        if (m > 0.0f) {
                            combinedCOM += rb->getCenterOfMassPosition() * m;
                            combinedMass += m;
                        }
                    };
                    
                    addBodyCOM(gChassisBody);
                    addBodyCOM(gCarBody.body);
                    for (int wi = 0; wi < 4; wi++) addBodyCOM(gWheels[wi].body);
                    
                    if (combinedMass > 0.0f) {
                        combinedCOM /= combinedMass;
                        Vector3 vCOM = {(float)combinedCOM.x(), (float)combinedCOM.y(), (float)combinedCOM.z()};
                        
                        // Disable depth testing so the COM sphere renders "through" the car
                        rlDisableDepthTest();
                        
                        // Draw a large magenta sphere at the true COM
                        DrawSphere(vCOM, 0.25f, {255, 0, 255, 200});
                        DrawSphereWires(vCOM, 0.25f, 8, 8, {255, 255, 255, 255});
                        
                        // Draw a line straight down to the ground to show where the weight rests
                        DrawLine3D(vCOM, {vCOM.x, 0.0f, vCOM.z}, {255, 0, 255, 150});
                        
                        rlEnableDepthTest();
                        
                        // NOTE: The spheres will be drawn in 2D space after EndMode3D to guarantee X-Ray visibility
                    }
                }

            EndMode3D();

            // HUD
            DrawText("Physics Sandbox — Spring Suspension Rig", 10, 10, 18, WHITE);
            DrawText(TextFormat("FPS: %d",GetFPS()), 10, 32, 14, {180,255,180,255});

            auto PrintP=[](const PhysicsObj& o,int y,Color col){
                btTransform bt;o.body->getMotionState()->getWorldTransform(bt);btVector3 p=bt.getOrigin();
                DrawText(TextFormat("%s  (%.1f, %.1f, %.1f)",o.label,(float)p.x(),(float)p.y(),(float)p.z()),10,y,12,col);
            };
            PrintP(gConcrete, 54,{255,200,140,255});
            PrintP(gWheels[0],70,{180,220,255,255});
            PrintP(gWheels[1],84,{180,220,255,255});
            PrintP(gWheels[2],98,{180,220,255,255});
            PrintP(gWheels[3],112,{180,220,255,255});

                // --- GAMEPAD STATE (Top Right, no overlap) ---
                int gpId = -1;
                for (int i=0; i<15; i++) if (IsGamepadAvailable(i)) { gpId=i; break; }
                if (gpId != -1) {
                    int baseX = GetScreenWidth() - 250;
                    int baseY = 50;
                    DrawRectangle(baseX - 5, baseY - 5, 240, 150, {0,0,0,180}); 
                    DrawText(TextFormat("HW: %s", GetGamepadName(gpId)), baseX, baseY, 10, {150, 255, 150, 255});
                    DrawText(TextFormat("RAW SIGNAL TICK: %d", gRawSignalCount), baseX, baseY + 15, 12, ORANGE);
                    
                    for (int a=0; a<8; a++) {
                        float v = GetGamepadAxisMovement(gpId, a);
                        Color c = (fabsf(v) > 0.05f) ? GREEN : GRAY;
                        DrawText(TextFormat("AX %d: [%.2f]", a, v), baseX + (a/4)*110, baseY + 35 + (a%4)*15, 11, c);
                    }
                    
                    if (GetGamepadAxisCount(gpId) == 0) {
                        DrawText("NO AXIS DATA - MAC OS PERMISSION DROP", baseX, baseY+70, 10, RED);
                    }
                }
                else {
                    int baseX = GetScreenWidth() - 250;
                    int baseY = 50;
                    DrawText("GAMEPAD: NONE DETECTED", baseX, baseY, 11, {255,100,100,200});
                }
                btTransform ct; gChassisBody->getMotionState()->getWorldTransform(ct);
                btVector3 fwd = ct.getBasis() * btVector3(0.f,0.f,1.f);
                float vCar = fabsf((float)gChassisBody->getLinearVelocity().dot(fwd));
                int yRow = 156;
                DrawText("TIRE TELEMETRY", 10, yRow, 11, {160,255,160,200}); yRow+=14;
                const char* wlabel[4]={"FL","FR","RL","RR"};
                Color wcol[4]={{120,180,255,220},{120,180,255,220},{80,255,80,220},{80,255,80,220}};
                for (int wi=0;wi<4;wi++) {
                    btRigidBody* wb   = gWheels[wi].body;
                    btVector3    axle = wb->getWorldTransform().getBasis()*btVector3(1,0,0);
                    float omega = (float)wb->getAngularVelocity().dot(axle);
                    float vSpin = fabsf(omega)*gWheels[wi].wheelRadius;
                    float denom = fmaxf(fmaxf(vCar,vSpin),0.30f);
                    float slip  = fminf(fabsf(vSpin-vCar)/denom,1.0f)*100.f;
                    float fric  = (float)wb->getFriction();
                    DrawText(TextFormat("%s  slip:%3.0f%%  grip:%.2f  spin:%.1f m/s",
                        wlabel[wi],slip,fric,vSpin), 10,yRow,11,wcol[wi]);
                    yRow+=13;
                }
                DrawText(TextFormat("car speed: %.1f m/s (%.0f km/h)",vCar,vCar*3.6f),
                    10,yRow,11,{200,200,255,200});


            if (isPicking&&pickedObj){
                bool isC=(pickedObj==&gConcrete);
                Color bc=isC?Color{255,160,80,255}:Color{60,220,255,255};
                DrawText(TextFormat("HOLDING: %s — springs pull other tires along",pickedObj->label),10,146,14,bc);
            } else if(hoveredObj){
                bool isC=(hoveredObj==&gConcrete);
                DrawText(isC?"[LMB] grab concrete":"[LMB] grab tire — suspension connects all 4",
                    10,146,13,isC?Color{255,200,100,255}:Color{80,200,255,255});
            }

            int ly=GetScreenHeight()-142;
            DrawText("WASD / Left Stick  — drive & steer (RWD friction drives car)", 10, ly,    13, {120,220,120,255});
            DrawText("SPACE / L-Trigger  — progressive braking",                    10, ly+16, 13, {255,160,160,255});
            DrawText("LMB / D-Pad        — grab vertex & move held object",         10, ly+32, 13, {160,160,160,255});
            DrawText("R / Gamepad Y      — reset rig",                              10, ly+48, 13, {160,160,160,255});
            DrawText("Spring lines: GREEN=rest  RED=compressed  BLUE=extended",       10, ly+64, 13, {100,180,100,200});
            DrawText("RMB = orbit  |  Scroll = zoom",                                 10, ly+80, 13, {160,160,160,255});
            DrawText("M = Toggle engine audio",                                       10, ly+96, 13, audioEnabled ? Color{100,255,100,255} : Color{255,80,80,255});
            
            if (showBBox) DrawText("B / Gamepad X — BBOX DEBUG ON",                   10, ly+112, 13, {255,200,100,255});
            else          DrawText("B / Gamepad X — show bounding boxes",             10, ly+112, 13, {120,120,140,180});
            
            if (showCOM)  DrawText("V — COM VISUALIZER ON (Magenta = True Center)",   10, ly+128, 13, {255,100,255,255});
            else          DrawText("V — show combined center of mass",                10, ly+128, 13, {120,120,140,180});
            
            DrawText("C — Toggle Camera Mode (Free Orbit / Car Follow)",              10, ly+144, 13, {180,220,180,255});

            // Draw the 2D Center of Mass Overlay
            if (showCOM) {
                btVector3 combinedCOM(0,0,0);
                float combinedMass = 0.0f;
                auto addBodyCOM = [&](btRigidBody* rb) {
                    float m = (rb->getInvMass() > 0.0f) ? (1.0f / rb->getInvMass()) : 0.0f;
                    if (m > 0.0f) { combinedCOM += rb->getCenterOfMassPosition() * m; combinedMass += m; }
                };
                addBodyCOM(gChassisBody); addBodyCOM(gCarBody.body);
                for (int wi = 0; wi < 4; wi++) addBodyCOM(gWheels[wi].body);
                
                if (combinedMass > 0.0f) {
                    combinedCOM /= combinedMass;
                    Vector3 vCOM = {(float)combinedCOM.x(), (float)combinedCOM.y(), (float)combinedCOM.z()};
                    Vector2 screenPos = GetWorldToScreen(vCOM, cam);
                    
                    DrawCircleV(screenPos, 8.0f, {255, 0, 255, 220});
                    DrawRing(screenPos, 10.0f, 13.0f, 0, 360.0f, 16, {255, 255, 255, 220});
                    DrawLine(screenPos.x - 20, screenPos.y, screenPos.x + 20, screenPos.y, {255, 255, 255, 150});
                    DrawLine(screenPos.x, screenPos.y - 20, screenPos.x, screenPos.y + 20, {255, 255, 255, 150});
                }
            }

            // RPM Gauge
            float rpmFrac = (currentRPM - RPM_IDLE) / (RPM_MAX - RPM_IDLE);
            if (rpmFrac < 0.0f) rpmFrac = 0.0f;
            if (rpmFrac > 1.0f) rpmFrac = 1.0f;
            Color rpmCol;
            if (rpmFrac < 0.6f)       rpmCol = {80, 220, 80, 255};
            else if (rpmFrac < 0.85f) rpmCol = {255, 200, 0, 255};
            else                       rpmCol = {255, 50, 50, 255};

            DrawText(TextFormat("RPM       : %5.0f", currentRPM), 10, 15, 16, rpmCol);
            DrawRectangle(160, 17, 200, 14, {40,40,40,200});
            DrawRectangle(160, 17, (int)(rpmFrac * 200), 14, rpmCol);

        EndDrawing();
    }

    // Cleanup
    if(pickC){gWorld->removeConstraint(pickC);delete pickC;}
    if(gCarWeldC){gWorld->removeConstraint(gCarWeldC);delete gCarWeldC; gCarWeldC=nullptr;}
    DestroySuspension();
    for(int i=0;i<nObjs;i++){UnloadModel(allObjs[i]->model);delete allObjs[i]->shape;}
    UnloadModel(gCarBody.model); delete gCarBody.shape;
    for(int i=gWorld->getNumCollisionObjects()-1;i>=0;i--){
        btCollisionObject* o=gWorld->getCollisionObjectArray()[i];
        btRigidBody* b=btRigidBody::upcast(o);
        if(b&&b->getMotionState())delete b->getMotionState();
        gWorld->removeCollisionObject(o);delete o;
    }
    delete gWorld;delete gSolver;delete gBroadphase;delete gDispatcher;delete gCollCfg;
    for (int i = 0; i < ENG_SAMPLE_COUNT; i++) {
        StopSound(engSounds[i]);
        UnloadSound(engSounds[i]);
    }
    CloseAudioDevice();
    CloseWindow();
    return 0;
}
