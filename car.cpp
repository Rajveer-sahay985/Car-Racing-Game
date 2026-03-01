#include <iostream>
#include "raylib.h"
#include "raymath.h"
#include "cmath"
using namespace std;

int main() {

    InitWindow(1280, 720, "Vortex Engine - step 1");
    SetTargetFPS(60);
    DisableCursor();


    Camera3D camera = {};
    camera.position = {0.f, 8.f, 16.f};
    camera.target = {0.f, 0.f, 0.f};
    camera.up = {0.f, 1.f, 0.f};
    camera.fovy = 35.f;
    camera.projection = CAMERA_PERSPECTIVE;
    float camYaw = 0.f;
    float camPitch = 0.45f;
    float camDist = 16.f;


    Vector3 boxPos = {0.f, 6.f, 0.f};
    Vector3 prevPos = {0.f, 6.f, 0.f};


    float boxAngle = 0.f;
    float moveSpeed = 8.f;
    float turnSpeed = 2.0f;
    
    //Physics constants
    float gravity = -20.0f;
    float bounciness = -0.4f;
    float groundFriction = 0.97f;
    float groundY = 1.f;


    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if(dt>0.05f) dt = 0.05f;

        // Input + Movement

        if(IsKeyDown(KEY_A)) boxAngle -=turnSpeed * dt;
        if(IsKeyDown(KEY_D)) boxAngle +=turnSpeed * dt;

        Vector3 forward = {sinf(boxAngle),0.f, -cosf(boxAngle)};


        //VERLET PHYSICS
        float velX = boxPos.x - prevPos.x;
        float velY = boxPos.y - prevPos.y;
        float velZ = boxPos.z - prevPos.z;


        if(IsKeyDown(KEY_W)){
            boxPos.x += forward.x * moveSpeed * dt;
            boxPos.z += forward.z * moveSpeed * dt;
        }

        if(IsKeyDown(KEY_S)){
            boxPos.x -= forward.x * moveSpeed * dt;
            boxPos.z -= forward.z * moveSpeed * dt; 
        }

        //SPACE - Jump!
        bool onGround = (boxPos.y <= groundY + 0.05f);
        if(IsKeyDown(KEY_SPACE) && onGround){
            velY = 8.f * dt;
        }

        prevPos = boxPos;

        float accY = gravity;

        //Calculate new position (Verlet Formula)
        boxPos.x +=velX;
        boxPos.y += velY + accY * dt * dt;
        boxPos.z += velZ;

        //Ground Collision
        if(boxPos.y < groundY){

            boxPos.y = groundY;

            float currentVelY = boxPos.y - prevPos.y;
            prevPos.y = boxPos.y + currentVelY * bounciness;

            //Ground Friction XZ slow movement
            float fX = boxPos.x - prevPos.x;
            float fZ = boxPos.z - prevPos.z;

            prevPos.x = boxPos.x + fX * groundFriction;
            prevPos.z = boxPos.z + fZ * groundFriction;
        }
        


        // Camera Rotate (Mouse se)
        Vector2 mouseDelta = GetMouseDelta();
        camYaw -= mouseDelta.x * 0.04f;
        camPitch += mouseDelta.y * 0.04f;

        if(camPitch < 0.1f) camPitch = 0.1f;
        if(camPitch > 1.4f) camPitch = 1.4f;

        camDist -= GetMouseWheelMove() * 1.5f;
        if(camDist < 3.f) camDist = 3.f;
        if(camDist > 30.f) camDist = 30.f;

        camera.position = {
            boxPos.x + camYaw * cosf(camPitch) * sinf(camDist),
            boxPos.y + camPitch * sinf(camDist),
            boxPos.z + camYaw * sinf(camPitch) * sinf(camDist)
        };
        camera.target = boxPos;


        float dispVelY = (boxPos.y - prevPos.y)/dt;



        BeginDrawing();
            ClearBackground((Color){100, 149, 237, 255});

            BeginMode3D(camera);

                DrawGrid(40,1.f);

                DrawCircle3D(
                    {boxPos.x, 0.05f, boxPos.z},
                    1.2f, {1,0,0}, 90.f,
                    {0,0,0,80}
                );

                DrawCube(boxPos, 2.f, 2.f, 4.f, RED);

                DrawCubeWires(boxPos, 2.f, 2.f, 4.f, BLACK);

                Vector3 arrowEnd = {
                    boxPos.x + forward.x * 3.f,
                    boxPos.y,
                    boxPos.z + forward.z * 3.f
                };

                DrawLine3D(boxPos, arrowEnd, YELLOW);
                DrawSphere(arrowEnd, 0.2f, RED);

            EndMode3D();

            DrawText("Vortex Engine - Step 4", 20, 20, 24, WHITE);
            DrawText("W/S: Move  |  A/D: Turn  |  SPACE: Jump", 20, 52, 16, YELLOW);

            DrawText(TextFormat("FPS: %i", GetFPS()), 20, 90, 20, WHITE);
            DrawRectangle(10, 100, 310, 90, (Color){0,0,0,140});
            DrawText("── Physics Debug ──", 18, 106, 15, RAYWHITE);
            DrawText(TextFormat("Position Y : %.2f", boxPos.y),        18, 124, 15, WHITE);
            DrawText(TextFormat("Velocity Y : %.2f m/s", dispVelY), 18, 142, 15,
                dispVelY < 0 ? RED : GREEN);  // red = girna, green = uchalna
            DrawText(TextFormat("On Ground  : %s", onGround ? "YES" : "NO"), 18, 160, 15,
                onGround ? GREEN : YELLOW);


            DrawRectangle(10, 640, 500, 42, (Color){0,0,0,140});
            DrawText("Verlet: newPos = pos + (pos-prevPos) + accel*dt*dt", 16, 648, 14, GRAY);


        EndDrawing();
    }

    CloseWindow();
    return 0;
}

