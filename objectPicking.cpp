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
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <cmath>
#include <string>
#include <vector>

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
    btConvexHullShape* shape      = nullptr;
    btVector3          centroid   = {0,0,0};
    btVector3          spawnPos   = {0,3,0};
    float mass        = 100.0f;
    float friction    = 0.60f;
    float restitution = 0.10f;
    float linDamp     = 0.10f;
    float angDamp     = 0.60f;
    float linDampHeld = 0.25f;
    float angDampHeld = 0.97f;
    Color baseColor   = WHITE;
    const char* label = "";
    // Pre-sampled pick anchors (LOCAL space) — max 16, computed once at load.
    // Only these are drawn / searched on hover, eliminating per-frame vertex loops.
    std::vector<Vector3> anchors;
};

static bool LoadPhysObj(PhysicsObj& obj, const char* path)
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
    obj.shape=new btConvexHullShape();
    for (int mi=0; mi<obj.model.meshCount; mi++) {
        Mesh& m=obj.model.meshes[mi];
        for (int vi=0; vi<m.vertexCount; vi++) {
            btVector3 v(m.vertices[vi*3],m.vertices[vi*3+1],m.vertices[vi*3+2]);
            obj.shape->addPoint(v-centroid,false);
        }
    }
    obj.shape->recalcLocalAabb(); obj.shape->optimizeConvexHull();
    obj.spawnPos=btVector3(0,centroid.y()+1.f,0);
    btTransform t; t.setIdentity(); t.setOrigin(obj.spawnPos);
    obj.body=MakeRigidBody(obj.mass,t,obj.shape);
    obj.body->setFriction(obj.friction); obj.body->setRestitution(obj.restitution);
    obj.body->setDamping(obj.linDamp,obj.angDamp);
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
    btVector3 pos=bt.getOrigin(); btMatrix3x3 rot=bt.getBasis();
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
static PhysicsObj gWheels[4];                          // FL FR RL RR
static btRigidBody*                  gChassisBody=nullptr;  // invisible chassis
static btCollisionShape*             gChassisShape=nullptr;
static btGeneric6DofSpring2Constraint* gSusp[4]={};    // per-wheel spring

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
    gChassisBody = MakeRigidBody(80.0f, t, gChassisShape);  // 80 kg frame
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
        // Y=±0.15 m  → suspension travel window.
        s->setLinearLowerLimit(btVector3(0.f, -0.15f, 0.f));
        s->setLinearUpperLimit(btVector3(0.f,  0.15f, 0.f));

        // ── Angular DOFs ──────────────────────────────────────────────────
        // In btGeneric6Dof: lower > upper → FREE,  lower = upper → LOCKED.
        //
        //   X (spin / roll around axle):  FREE  — tire can rotate naturally
        //   Y (yaw / steering):           LOCKED — wheel always faces forward
        //   Z (camber / tilt):            LOCKED — wheel stays perfectly upright
        s->setAngularLowerLimit(btVector3( 1.f, 0.f, 0.f));  // X: lower(1)>upper(-1)=FREE
        s->setAngularUpperLimit(btVector3(-1.f, 0.f, 0.f));  // Y,Z: lower(0)=upper(0)=LOCKED

        // ── Suspension spring on linear-Y (DOF index 1) ───────────────────
        // Stiffness: 80 N/m.  A 15 kg tire at rest sags ~15*9.81/80 ≈ 1.8 cm.
        // Damping: 8 Ns/m — damps oscillation without feeling rubber-band.
        s->enableSpring(1, true);
        s->setStiffness(1, 80.0f);
        s->setDamping  (1,  8.0f);
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
    InitWindow(1280, 720, "Physics Sandbox — Concrete + Suspended Tire Rig");
    SetTargetFPS(60);
    InitBullet();

    // ── Ground ───────────────────────────────────────────────────────────────
    {
        auto* shape=new btStaticPlaneShape(btVector3(0,1,0),0);
        btTransform t;t.setIdentity();
        auto* b=MakeRigidBody(0.f,t,shape);
        b->setFriction(0.70f); b->setRestitution(0.04f);
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

    // ── Four tires — car formation ────────────────────────────────────────────
    //  Formation coords (settled on ground ~ y=0.40):
    //     FL (-1.1,  0.90,  1.6)    FR (1.1,  0.90,  1.6)
    //     RL (-1.1,  0.90, -1.6)    RR (1.1,  0.90, -1.6)
    struct TireDef { const char* path; const char* label; btVector3 spawn; };
    TireDef td[4]={
        {"Obj Files/wheel_fl.obj","FL TIRE 15kg",{-1.1f,0.90f, 1.6f}},
        {"Obj Files/wheel_fr.obj","FR TIRE 15kg",{ 1.1f,0.90f, 1.6f}},
        {"Obj Files/wheel_rl.obj","RL TIRE 15kg",{-1.1f,0.90f,-1.6f}},
        {"Obj Files/wheel_rr.obj","RR TIRE 15kg",{ 1.1f,0.90f,-1.6f}},
    };

    btVector3 wheelSpawns[4];
    for (int i=0;i<4;i++) {
        gWheels[i].mass=15.f; gWheels[i].friction=0.90f; gWheels[i].restitution=0.30f;
        gWheels[i].linDamp=0.12f; gWheels[i].angDamp=0.35f;
        gWheels[i].linDampHeld=0.20f; gWheels[i].angDampHeld=0.90f;
        gWheels[i].baseColor={30,30,35,255};   // dark rubber
        gWheels[i].label=td[i].label;
        if (!LoadPhysObj(gWheels[i],td[i].path)){
            TraceLog(LOG_ERROR,"Missing %s",td[i].path); return 1;
        }
        ComputeAnchors(gWheels[i]);     // pre-build 16 pick anchors
        gWheels[i].spawnPos=td[i].spawn;
        ResetPhysObj(gWheels[i]);
        wheelSpawns[i]=td[i].spawn;
    }

    // ── Build invisible suspension rig ────────────────────────────────────────
    BuildSuspension(wheelSpawns);

    // ── Pickable object roster (concrete + 4 tires — chassis is invisible) ───
    PhysicsObj* allObjs[5]={&gConcrete,&gWheels[0],&gWheels[1],&gWheels[2],&gWheels[3]};
    const int nObjs=5;

    // ── Camera ────────────────────────────────────────────────────────────────
    float camYaw=30.f,camPitch=22.f,camDist=22.f,camDistTgt=22.f;
    Vector3 camFocus={0,0,0};

    // ── Picking state ─────────────────────────────────────────────────────────
    float moveSpeed=8.f;
    btPoint2PointConstraint* pickC=nullptr;
    PhysicsObj* pickedObj=nullptr;
    btVector3   pickPivot;
    Vector3     pickVtx={0,0,0};
    bool        isPicking=false;
    PhysicsObj* hoveredObj=nullptr;
    int         hoverVI=-1;
    Vector3     hoverVW={0,0,0};

    // ==========================================================================
    //  GAME LOOP
    // ==========================================================================
    while (!WindowShouldClose())
    {
        float dt=GetFrameTime(); if(dt>0.05f)dt=0.05f;
        float scroll=GetMouseWheelMove();

        // Camera orbit
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
        float yr=camYaw*DEG2RAD, pr=camPitch*DEG2RAD;
        Camera3D cam={};
        cam.position={camFocus.x+camDist*cosf(pr)*sinf(yr),
                      camFocus.y+camDist*sinf(pr),
                      camFocus.z+camDist*cosf(pr)*cosf(yr)};
        cam.target=camFocus; cam.up={0,1,0}; cam.fovy=45.f;
        cam.projection=CAMERA_PERSPECTIVE;

        // Reset
        if (IsKeyPressed(KEY_R)) {
            if(pickedObj) ReleaseObj(*pickedObj,pickC);
            isPicking=false; pickedObj=nullptr;
            ResetPhysObj(gConcrete);
            ResetRig(wheelSpawns);
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
            float spd=moveSpeed*dt;
            float cfX=sinf(yr),cfZ=cosf(yr),crX=cosf(yr),crZ=-sinf(yr);
            if(IsKeyDown(KEY_W)){pickPivot-=btVector3(cfX*spd,0,cfZ*spd);}
            if(IsKeyDown(KEY_S)){pickPivot+=btVector3(cfX*spd,0,cfZ*spd);}
            if(IsKeyDown(KEY_A)){pickPivot-=btVector3(crX*spd,0,crZ*spd);}
            if(IsKeyDown(KEY_D)){pickPivot+=btVector3(crX*spd,0,crZ*spd);}
            pickC->setPivotB(pickPivot);
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                ReleaseObj(*pickedObj,pickC);
                isPicking=false; pickedObj=nullptr;
            }
        }

        // ── Rear-wheel drive + steering (arrow keys) ─────────────────────────
        // Drive:  torque applied to RL+RR around each wheel's LOCAL X axis (axle).
        //         Rubber friction (0.90) against ground creates actual contact force.
        //         Chassis is pulled forward by the suspension springs.
        // Steer:  torque applied to chassis LOCAL Y axis — whole rig yaws gently.
        // Picking the rig does NOT override driving; both can coexist.
        {
            const float DRIVE_TORQUE = 100.0f;    // Nm per second applied to rear axles
            const float MAX_SPIN     = 20.0f;    // rad/s cap (≈40 km/h for r≈0.35 m)
            const float STEER_TORQUE = 18.0f;    // Nm per second applied to chassis Y
            const float MAX_YAW_RATE =  2.5f;    // rad/s cap on chassis spin

            float driveDir = 0.f;
            if (IsKeyDown(KEY_UP))   driveDir = -1.f;  // −X spin = forward roll
            if (IsKeyDown(KEY_DOWN)) driveDir =  1.f;  // +X spin = reverse roll

            if (driveDir != 0.f) {
                for (int wi = 2; wi <= 3; wi++) {   // RL=2, RR=3
                    btRigidBody* wb = gWheels[wi].body;
                    // Get the wheel's own axle direction (local X in world)
                    btVector3 axle = wb->getWorldTransform().getBasis() * btVector3(1,0,0);
                    // Cap spin speed so the torque doesn’t run away
                    float currSpin = (float)(wb->getAngularVelocity().dot(axle));
                    if (fabsf(currSpin) < MAX_SPIN) {
                        wb->applyTorqueImpulse(axle * (driveDir * DRIVE_TORQUE * dt));
                    }
                    wb->activate(true);
                }
                gChassisBody->activate(true);
            }

            float steerDir = 0.f;
            if (IsKeyDown(KEY_LEFT))  steerDir =  1.f;
            if (IsKeyDown(KEY_RIGHT)) steerDir = -1.f;

            if (steerDir != 0.f) {
                // Get chassis up axis in world space
                btVector3 up = gChassisBody->getWorldTransform().getBasis() * btVector3(0,1,0);
                float currYaw = (float)(gChassisBody->getAngularVelocity().dot(up));
                if (fabsf(currYaw) < MAX_YAW_RATE) {
                    gChassisBody->applyTorqueImpulse(up * (steerDir * STEER_TORQUE * dt));
                }
                gChassisBody->activate(true);
            }
        }

        // Physics
        gWorld->stepSimulation(dt,10,1.f/120.f);

        // Sync
        for(int i=0;i<nObjs;i++) SyncTransform(*allObjs[i]);

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
                // Arrow points in the FORWARD direction of each wheel.
                // Rear driven wheels glow bright green when a drive key is held.
                {
                    // Chassis forward = its local +Z
                    btMatrix3x3 cb = gChassisBody->getWorldTransform().getBasis();
                    btVector3 fwdBt = cb * btVector3(0, 0, 1);
                    Vector3 fwd = {(float)fwdBt.x(), 0.f, (float)fwdBt.z()};
                    float fl = sqrtf(fwd.x*fwd.x + fwd.z*fwd.z);
                    if(fl > 0.001f){ fwd.x/=fl; fwd.z/=fl; }

                    bool driving = IsKeyDown(KEY_UP) || IsKeyDown(KEY_DOWN);

                    for (int i = 0; i < 4; i++) {
                        btTransform wt; gWheels[i].body->getMotionState()->getWorldTransform(wt);
                        btVector3 wp = wt.getOrigin();
                        Vector3 wc = {(float)wp.x(), (float)wp.y(), (float)wp.z()};

                        bool isRear = (i >= 2);
                        // Rear wheels glow green when driven, white when idle
                        Color arrowCol = (isRear && driving)
                            ? Color{80,255,80,255}
                            : (isRear ? Color{120,200,120,180} : Color{120,180,255,160});
                        float arrowLen = isRear ? 1.0f : 0.75f;

                        Vector3 tip = {wc.x + fwd.x*arrowLen,
                                       wc.y,
                                       wc.z + fwd.z*arrowLen};
                        DrawLine3D(wc, tip, arrowCol);
                        DrawSphere(tip, 0.07f, arrowCol);
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
            DrawText("Suspension: stiffness=50 N/m  damping=6 Ns/m  travel=±20cm",10,128,12,{120,180,120,220});

            if (isPicking&&pickedObj){
                bool isC=(pickedObj==&gConcrete);
                Color bc=isC?Color{255,160,80,255}:Color{60,220,255,255};
                DrawText(TextFormat("HOLDING: %s — springs pull other tires along",pickedObj->label),10,146,14,bc);
            } else if(hoveredObj){
                bool isC=(hoveredObj==&gConcrete);
                DrawText(isC?"[LMB] grab concrete":"[LMB] grab tire — suspension connects all 4",
                    10,146,13,isC?Color{255,200,100,255}:Color{80,200,255,255});
            }

            int ly=GetScreenHeight()-84;
            DrawText("Arrow UP/DN  — rear wheels spin (RWD friction drives car fwd/back)",10,ly,   13,{120,220,120,255});
            DrawText("Arrow L/R   — steer chassis yaw (whole rig turns)",               10,ly+16,13,{120,200,200,255});
            DrawText("LMB click = grab vertex  |  drag up/dn = height  |  WASD = slide", 10,ly+32,13,{160,160,160,255});
            DrawText("Spring lines: GREEN=rest  RED=compressed  BLUE=extended",           10,ly+48,13,{100,180,100,200});
            DrawText("Arrows on tires = forward direction  |  BRIGHT GREEN = driven ring",10,ly+64,13,{100,180,100,180});
            DrawText("RMB = orbit  |  Scroll = zoom  |  R = reset all",                  10,ly+80,13,{160,160,160,255});

        EndDrawing();
    }

    // Cleanup
    if(pickC){gWorld->removeConstraint(pickC);delete pickC;}
    DestroySuspension();
    for(int i=0;i<nObjs;i++){UnloadModel(allObjs[i]->model);delete allObjs[i]->shape;}
    for(int i=gWorld->getNumCollisionObjects()-1;i>=0;i--){
        btCollisionObject* o=gWorld->getCollisionObjectArray()[i];
        btRigidBody* b=btRigidBody::upcast(o);
        if(b&&b->getMotionState())delete b->getMotionState();
        gWorld->removeCollisionObject(o);delete o;
    }
    delete gWorld;delete gSolver;delete gBroadphase;delete gDispatcher;delete gCollCfg;
    CloseWindow();
    return 0;
}
