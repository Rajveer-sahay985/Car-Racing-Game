#include "raylib.h"
#include "raymath.h"
#include <cmath>

int main(){
    InitWindow(1280, 720, "Car Physics Engine");
    SetTargetFPS(60);
    // --- Cel-shading (toon) shader ---
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
        "void main() {\n"
        "    fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));\n"
        "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
        "}\n";

    const char *celFS =
        "#version 410 core\n"
        "in vec3 fragNormal;\n"
        "uniform vec4 colDiffuse;\n"
        "out vec4 finalColor;\n"
        "void main() {\n"
        "    vec3 lightDir = normalize(vec3(0.4, 1.0, 0.3));\n"
        "    float NdotL = dot(normalize(fragNormal), lightDir);\n"
        "    float intensity;\n"
        "    if      (NdotL > 0.6) intensity = 5.0;\n"
        "    else if (NdotL > 0.3) intensity = 3.0;\n"
        "    else if (NdotL > 0.0) intensity = 2.0;\n"
        "    else                  intensity = 1.0;\n"
        "    finalColor = vec4(colDiffuse.rgb * intensity, colDiffuse.a);\n"
        "}\n";

    Shader celShader = LoadShaderFromMemory(celVS, celFS);

    // --- Load actual wheel .obj models ---
    float wheelRadius = 0.333f;   // from OBJ: Y size is 0.666, so radius = 0.333
    Model wheelModels[4] = {
        LoadModel("wheel_fr.obj"),   // 0 = Front-Left
        LoadModel("wheel_fl.obj"),   // 1 = Front-Right
        LoadModel("wheel_rr.obj"),   // 2 = Rear-Left
        LoadModel("wheel_rl.obj"),   // 3 = Rear-Right
    };

    // Apply cel shader to every material of every wheel
    for (int i = 0; i < 4; i++)
        for (int m = 0; m < wheelModels[i].materialCount; m++)
            wheelModels[i].materials[m].shader = celShader;

    // --- Load car chassis .obj model ---
    Model carModel = LoadModel("car.obj");
    for (int m = 0; m < carModel.materialCount; m++)
        carModel.materials[m].shader = celShader;

    // --- Car dimensions ---
    float wheelBase  = 2.5f;
    float trackWidth = 1.6f;

    // Fine-tune wheel arch alignment (positive = further forward for front, further back for rear)
    float frontZOffset = 0.35f;   // push front wheels forward into their arch
    float rearZOffset  = 0.0f;   // push rear wheels back into their arch

    // Wheel local positions: FL, FR, RL, RR
    Vector3 wheelLocal[4] = {
        {-trackWidth/2, wheelRadius,  wheelBase/2 + frontZOffset},   // FL
        { trackWidth/2, wheelRadius,  wheelBase/2 + frontZOffset},   // FR
        {-trackWidth/2, wheelRadius, -wheelBase/2 - rearZOffset},    // RL
        { trackWidth/2, wheelRadius, -wheelBase/2 - rearZOffset},    // RR
    };

    // --- Physics state ---
    Vector3 carPos = {0, 0, 0};
    float heading    = 0.0f;
    float speed      = 0.0f;
    float steerAngle = 0.0f;
    float wheelSpin  = 0.0f;

    // --- Tuning ---
    float engineForce    = 10.0f;
    float brakeForce     = 15.0f;
    float maxSpeed       = 25.0f;
    float maxReverse     = 10.0f;
    float friction       = 2.0f;
    float maxSteer       = 35.0f * DEG2RAD;
    float steerSpeed     = 2.5f;
    float steerReturn    = 5.0f;

    // --- Orbital camera ---
    float camYaw = 0.0f, camPitch = 25.0f;
    float camDist = 12.0f, camDistTgt = 12.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // ===== INPUT =====
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

        // Speed-dependent steering (harder to turn at high speed)
        float sFactor = 1.0f - 0.5f * (fabsf(speed) / maxSpeed);
        float effSteer = steerAngle * sFactor;

        // ===== BICYCLE MODEL PHYSICS =====
        if (fabsf(speed) > 0.01f)
            heading += (speed * tanf(effSteer) / wheelBase) * dt;

        carPos.x += speed * sinf(heading) * dt;
        carPos.z += speed * cosf(heading) * dt;
        // 👇 WHEEL ROLL CALCULATION — this accumulates the rolling angle.
        // angular velocity = speed / radius  (like a real tire: faster you go, faster it spins)
        // Positive speed (W) → wheelSpin increases → wheels roll forward (clockwise from right side)
        // Negative speed (S/reverse) → wheelSpin decreases → wheels roll backward (counter-clockwise)
        wheelSpin += (speed / wheelRadius) * dt;

        // ===== CAMERA =====
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

        Vector3 camTgt = {carPos.x, 0.5f, carPos.z};
        float yr = camYaw * DEG2RAD, pr = camPitch * DEG2RAD;
        Camera3D camera = {0};
        camera.position = {
            camTgt.x + camDist * cosf(pr) * sinf(yr),
            camTgt.y + camDist * sinf(pr),
            camTgt.z + camDist * cosf(pr) * cosf(yr)
        };
        camera.target = camTgt;
        camera.up = {0, 1, 0};
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        // ===== DRAW =====
        BeginDrawing();
            ClearBackground(BLACK);

            // Sky gradient
            int hH = GetScreenHeight() / 2;
            DrawRectangleGradientV(0, 0,  GetScreenWidth(), hH, (Color){25,25,112,255}, (Color){135,206,235,255});
            DrawRectangleGradientV(0, hH, GetScreenWidth(), hH, (Color){135,206,235,255}, (Color){255,200,150,255});

            BeginMode3D(camera);
                DrawPlane({0,0,0}, {100,100}, (Color){50,50,50,255});
                DrawGrid(100, 1.0f);

                // Compute world wheel positions for frame lines
                Matrix carMat = MatrixMultiply(MatrixRotateY(heading), MatrixTranslate(carPos.x, 0, carPos.z));
                Vector3 wWorld[4];

                // Draw 4 wheels (using actual .obj models)
                for (int i = 0; i < 4; i++) {
                    bool front = (i < 2);

                    // --- VISUAL ORIENTATION AND PLACEMENT ---
                    
                    // 1. ROTATE AROUND AXLE (Spinning)
                    // This rotates the wheel around its own X-axis based on the distance traveled.
                    Matrix m = MatrixRotateX(wheelSpin);

                    // 2. STEER (Front Wheels Only)
                    // For the front two wheels (i=0 and i=1), we rotate them around the Y-axis
                    // so they point in the direction of the steering.
                    if (front) m = MatrixMultiply(m, MatrixRotateY(steerAngle));

                    // 3. ATTACH TO CHASSIS (Local Offset)
                    // We move the wheel from the origin to its specific corner on the vehicle.
                    m = MatrixMultiply(m, MatrixTranslate(wheelLocal[i].x, wheelLocal[i].y, wheelLocal[i].z));

                    // 4. ORIENT WITH VEHICLE (Heading and World Position)
                    // Finally, we rotate the whole wheel setup based on the car's current heading
                    // and move it to the car's current position in the world.
                    m = MatrixMultiply(m, carMat);

                    wheelModels[i].transform = m;
                    DrawModel(wheelModels[i], {0,0,0}, 1.0f, WHITE);
                    DrawModelWires(wheelModels[i], {0,0,0}, 1.0f, BLACK);

                    wWorld[i] = Vector3Transform(wheelLocal[i], carMat);
                }

                // Draw car chassis — shifted backward in local space to align rear wheel arches
                // Negative Z = move chassis toward the rear relative to the car's heading
                float chassisZOffset = -0.1f;
                Matrix chassisMat = MatrixMultiply(MatrixTranslate(0.0f, 0.0f, chassisZOffset), carMat);
                carModel.transform = chassisMat;
                DrawModel(carModel, {0,0,0}, 1.0f, WHITE);
                DrawModelWires(carModel, {0,0,0}, 1.0f, BLACK);
            EndMode3D();

            // HUD
            DrawText("Car Physics Engine", 10, 10, 22, WHITE);
            DrawText(TextFormat("Speed: %.1f m/s", speed), 10, 40, 18, GREEN);
            DrawText(TextFormat("Steer: %.1f deg", steerAngle * RAD2DEG), 10, 62, 18, YELLOW);
            int y = GetScreenHeight() - 90;
            DrawText("W / S  :  Throttle / Brake-Reverse", 10, y,    16, WHITE);
            DrawText("A / D  :  Steer Left / Right",       10, y+20, 16, WHITE);
            DrawText("Mouse drag : Orbit camera",          10, y+40, 16, WHITE);
            DrawText("Scroll     : Zoom",                  10, y+60, 16, WHITE);
        EndDrawing();
    }

    for (int i = 0; i < 4; i++) UnloadModel(wheelModels[i]);
    UnloadModel(carModel);
    UnloadShader(celShader);
    CloseWindow();
    return 0;
}
