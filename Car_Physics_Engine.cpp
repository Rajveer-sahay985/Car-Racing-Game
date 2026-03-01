// =============================================================================
//  Vortex Racing Engine — STEP 0: Working base (bicycle physics + textures)
//  Build:
//    clang++ Car_Physics_Engine.cpp -std=c++17 \
//      -I/opt/homebrew/include -L/opt/homebrew/lib \
//      -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo \
//      -O2 -o car_main && ./car_main
// =============================================================================
#include "raylib.h"
#include "raymath.h"
#include <cmath>

int main()
{
    InitWindow(1280, 720, "Vortex Racing — Step 0: Base");
    SetTargetFPS(60);

    // -------------------------------------------------------------------------
    //  CEL SHADER — textured toon shading
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
    //  MODELS
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
    //  CAR DIMENSIONS
    // -------------------------------------------------------------------------
    float wheelBase  = 2.5f;
    float trackWidth = 1.6f;
    float frontZOffset = 0.35f;
    float rearZOffset  = 0.0f;

    // Wheel positions in car-local space: FL(0) FR(1) RL(2) RR(3)
    Vector3 wheelLocal[4] = {
        { -trackWidth/2, wheelRadius,  wheelBase/2 + frontZOffset },  // FL
        {  trackWidth/2, wheelRadius,  wheelBase/2 + frontZOffset },  // FR
        { -trackWidth/2, wheelRadius, -wheelBase/2 - rearZOffset  },  // RL
        {  trackWidth/2, wheelRadius, -wheelBase/2 - rearZOffset  },  // RR
    };

    // -------------------------------------------------------------------------
    //  PHYSICS STATE  (bicycle model)
    // -------------------------------------------------------------------------
    Vector3 carPos    = { 0, 0, 0 };
    float   heading   = 0.0f;     // yaw in radians
    float   speed     = 0.0f;     // m/s, positive = forward
    float   steerAngle= 0.0f;     // radians
    float   wheelSpin = 0.0f;     // accumulated roll angle

    // --- Tuning ---
    float engineForce = 10.0f;
    float brakeForce  = 15.0f;
    float maxSpeed    = 25.0f;
    float maxReverse  = 10.0f;
    float friction    = 2.0f;
    float maxSteer    = 35.0f * DEG2RAD;
    float steerSpeed  = 2.5f;
    float steerReturn = 5.0f;

    // -------------------------------------------------------------------------
    //  CAMERA  (basic orbital)
    // -------------------------------------------------------------------------
    float camYaw   = 0.0f;
    float camPitch = 25.0f;
    float camDist  = 12.0f;
    float camDistTgt = 12.0f;

    // =========================================================================
    //  GAME LOOP
    // =========================================================================
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // =====================================================================
        //  INPUT + BICYCLE PHYSICS
        // =====================================================================
        if (IsKeyDown(KEY_W))      speed += engineForce * dt;
        else if (IsKeyDown(KEY_S)) speed -= (speed > 0.5f ? brakeForce : engineForce * 0.5f) * dt;
        else {
            if (speed > 0) { speed -= friction * dt; if (speed < 0) speed = 0; }
            if (speed < 0) { speed += friction * dt; if (speed > 0) speed = 0; }
        }
        if (speed >  maxSpeed)   speed =  maxSpeed;
        if (speed < -maxReverse) speed = -maxReverse;

        if (IsKeyDown(KEY_A))      steerAngle += steerSpeed * dt;
        else if (IsKeyDown(KEY_D)) steerAngle -= steerSpeed * dt;
        else {
            if (steerAngle > 0) { steerAngle -= steerReturn * dt; if (steerAngle < 0) steerAngle = 0; }
            if (steerAngle < 0) { steerAngle += steerReturn * dt; if (steerAngle > 0) steerAngle = 0; }
        }
        if (steerAngle >  maxSteer) steerAngle =  maxSteer;
        if (steerAngle < -maxSteer) steerAngle = -maxSteer;

        // Speed-dependent steering
        float sFactor  = 1.0f - 0.5f * (fabsf(speed) / maxSpeed);
        float effSteer = steerAngle * sFactor;

        // Bicycle model
        if (fabsf(speed) > 0.01f)
            heading += (speed * tanf(effSteer) / wheelBase) * dt;

        carPos.x += speed * sinf(heading) * dt;
        carPos.z += speed * cosf(heading) * dt;
        wheelSpin += (speed / wheelRadius) * dt;

        // =====================================================================
        //  ORBITAL CAMERA
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
        //  DRAW
        // =====================================================================
        BeginDrawing();
            ClearBackground(BLACK);

            // Sky gradient
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

                // --- Wheels ---
                for (int i = 0; i < 4; i++) {
                    bool front = (i < 2);
                    Matrix m = MatrixRotateX(wheelSpin);
                    if (front) m = MatrixMultiply(m, MatrixRotateY(steerAngle));
                    m = MatrixMultiply(m, MatrixTranslate(wheelLocal[i].x, wheelLocal[i].y, wheelLocal[i].z));
                    m = MatrixMultiply(m, carMat);
                    wheelModels[i].transform = m;
                    DrawModel(wheelModels[i], {0,0,0}, 1.0f, WHITE);
                }

                // --- Chassis ---
                Matrix chassisMat = MatrixMultiply(MatrixTranslate(0, 0, -0.1f), carMat);
                carModel.transform = chassisMat;
                DrawModel(carModel, {0,0,0}, 1.0f, WHITE);

            EndMode3D();

            // --- HUD ---
            DrawText("Step 0: Base (Bicycle Physics + Textures)", 10, 10, 18, WHITE);
            DrawText(TextFormat("Speed: %.1f m/s", speed),           10, 38, 16, GREEN);
            DrawText(TextFormat("Steer: %.1f deg", steerAngle*RAD2DEG),10,58,16, YELLOW);
            int y = GetScreenHeight() - 90;
            DrawText("W/S  : Throttle / Brake",  10, y,    15, {180,180,180,255});
            DrawText("A/D  : Steer",              10, y+18, 15, {180,180,180,255});
            DrawText("Mouse: Orbit camera",        10, y+36, 15, {180,180,180,255});
        EndDrawing();
    }

    // Cleanup
    for (int i = 0; i < 4; i++) UnloadModel(wheelModels[i]);
    UnloadModel(carModel);
    UnloadShader(celShader);
    CloseWindow();
    return 0;
}
