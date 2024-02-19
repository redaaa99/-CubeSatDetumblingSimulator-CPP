#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

#define MASS_SPCRFT 2.6 // [kg] *Range: MASS_SPCRFT > 0.0*
#define DIMENSIONS_X_SPCRFT 0.5f // [m] *Range: DIMENSIONS_X_SPCRFT > 0.0*
#define DIMENSIONS_Y_SPCRFT 0.5f // [m] *Range: DIMENSIONS_Y_SPCRFT > 0.0*
#define DIMENSIONS_Z_SPCRFT 1.0f // [m] *Range: DIMENSIONS_Z_SPCRFT > 0.0*
#define INERTIA_TENSOR_SPCRFT {0.0108, 0, 0, \
                                0, 0.0108, 0, \
                                0, 0, 0.0043} // [kg-m^2] *Range: INERTIA_TENSOR_SPCRFT > [0.0, 0.0, 0.0; 0.0, 0.0, 0.0; 0.0, 0.0, 0.0]*
#define INIT_YAW_ANGLE_SPCRFT 0 // [deg] *Range: 0.0 ≤ INIT_YAW_ANGLE_SPCRFT < 360.0*
#define INIT_PITCH_ANGLE_SPCRFT 0 // [deg] *Range: 0.0 ≤ INIT_PITCH_ANGLE_SPCRFT < 360.0*
#define INIT_ROLL_ANGLE_SPCRFT 0 // [deg] *Range: 0.0 ≤ INIT_ROLL_ANGLE_SPCRFT < 360.0*
#define INIT_BODY_ANGULAR_RATE_X_SPCRFT 10 // [deg/s] *Range: INIT_BODY_ANGULAR_RATE_X_SPCRFT ≥ 0.0*
#define INIT_BODY_ANGULAR_RATE_Y_SPCRFT 10 // [deg/s] *Range: INIT_BODY_ANGULAR_RATE_Y_SPCRFT ≥ 0.0*
#define INIT_BODY_ANGULAR_RATE_Z_SPCRFT 10 // [deg/s] *Range: INIT_BODY_ANGULAR_RATE_Z_SPCRFT ≥ 0.0*
#define DESIRED_BODY_ANGULAR_RATE_X_SPCRFT 0 // [deg/s] *Range: DESIRED_BODY_ANGULAR_RATE_X_SPCRFT ≥ 0.0*
#define DESIRED_BODY_ANGULAR_RATE_Y_SPCRFT 0 // [deg/s] *Range: DESIRED_BODY_ANGULAR_RATE_Y_SPCRFT ≥ 0.0*
#define DESIRED_BODY_ANGULAR_RATE_Z_SPCRFT 0 // [deg/s] *Range: DESIRED_BODY_ANGULAR_RATE_Z_SPCRFT ≥ 0.0*

#define deg2rad(deg) ((deg) * M_PI / 180.0)

Vector3 center = { 0.0f, 0.0f, 0.0f };
Vector3 endOfXAxis = { 1.0f, 0.0f, 0.0f };
Vector3 endOfYAxis = { 0.0f, 0.0f, 1.0f };
Vector3 endOfZAxis = { 0.0f, 1.0f, 0.0f };
Vector3 endOfXSpcrft = { 0.8f, 0.0f, 0.0f };
Vector3 endOfYSpcrft = { 0.0f, 0.0f, 0.8f };
Vector3 endOfZSpcrft = { 0.0f, 0.8f, 0.0f };


#include "raylib.h"

Vector3 RotatePoint(Vector3 point, Vector3 axis, float angle)
{
    // Normalize the axis vector
    axis = Vector3Normalize(axis);

    // Convert the angle from degrees to radians
    angle = deg2rad(angle);

    // Calculate trigonometric values
    float cosTheta = cosf(angle);
    float sinTheta = sinf(angle);

    // Calculate the rotation matrix components
    float ux = axis.x;
    float uy = axis.y;
    float uz = axis.z;

    // Calculate the new coordinates after rotation
    Vector3 rotatedPoint;
    rotatedPoint.x = (cosTheta + (1 - cosTheta) * ux * ux) * point.x +
                     ((1 - cosTheta) * ux * uy - sinTheta * uz) * point.y +
                     ((1 - cosTheta) * ux * uz + sinTheta * uy) * point.z;
    rotatedPoint.y = ((1 - cosTheta) * ux * uy + sinTheta * uz) * point.x +
                     (cosTheta + (1 - cosTheta) * uy * uy) * point.y +
                     ((1 - cosTheta) * uy * uz - sinTheta * ux) * point.z;
    rotatedPoint.z = ((1 - cosTheta) * ux * uz - sinTheta * uy) * point.x +
                     ((1 - cosTheta) * uy * uz + sinTheta * ux) * point.y +
                     (cosTheta + (1 - cosTheta) * uz * uz) * point.z;

    return rotatedPoint;
}


//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 800;
    
    InitWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera free");

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 3.0f, 3.0f, 3.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    double bodyAngularRate[3] = { deg2rad(INIT_BODY_ANGULAR_RATE_X_SPCRFT), deg2rad(INIT_BODY_ANGULAR_RATE_Y_SPCRFT), deg2rad(INIT_BODY_ANGULAR_RATE_Z_SPCRFT) };

    Quaternion q = QuaternionFromEuler(INIT_PITCH_ANGLE_SPCRFT, INIT_YAW_ANGLE_SPCRFT, INIT_ROLL_ANGLE_SPCRFT);
    float dqdtmat[4][4] = { {0                  , -bodyAngularRate[0], -bodyAngularRate[1], -bodyAngularRate[2] }, 
                            {bodyAngularRate[0] , 0                  , bodyAngularRate[2] , -bodyAngularRate[1] },
                            {bodyAngularRate[1] , -bodyAngularRate[2], 0                  , bodyAngularRate[0]  },
                            {bodyAngularRate[2] , bodyAngularRate[1] , -bodyAngularRate[0], 0                   }};
                      
    
    double dqtrnion_dt[4]; // Assuming dqtrnion_dt is a 1D array representing the quaternion derivative
    double qtrnion_spcrft[4] = {q.w, q.x, q.y, q.z}; // Assuming dqtrnion_dt is a 1D array representing the quaternion derivative
    // Matrix multiplication: matrix * qtrnion_spcrft * 0.5
    
    for (int i = 0; i < 4; i++) {
        dqtrnion_dt[i] = 0;
        for (int j = 0; j < 4; j++) {
            dqtrnion_dt[i] += dqdtmat[i][j] * qtrnion_spcrft[j];
        }
        dqtrnion_dt[i] *= 0.5;
    }
    

    Quaternion dqdt = (Quaternion){ dqtrnion_dt[0], dqtrnion_dt[1], dqtrnion_dt[2], dqtrnion_dt[3]};
        
    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float c = 0.0f;
    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------

        q = QuaternionNormalize(QuaternionAdd(q, dqdt));
        for (int i = 0; i < 4; i++) {
            dqtrnion_dt[i] = 0;
            for (int j = 0; j < 4; j++) {
                dqtrnion_dt[i] += dqdtmat[i][j] * qtrnion_spcrft[j];
            }
            dqtrnion_dt[i] *= 0.5;
        }

        dqdt = (Quaternion){ dqtrnion_dt[0], dqtrnion_dt[1], dqtrnion_dt[2], dqtrnion_dt[3]};
        c += 2*acos(q.w);
        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            BeginMode3D(camera);
                DrawLine3D(center, endOfXAxis, WHITE);
                DrawLine3D(center, endOfYAxis, WHITE);
                DrawLine3D(center, endOfZAxis, WHITE);

                //DrawSpaceCraft();

                Vector3 spaceCrftSize = { DIMENSIONS_X_SPCRFT, DIMENSIONS_Z_SPCRFT, DIMENSIONS_Y_SPCRFT};
                                    
                rlPushMatrix();
                    rlRotatef(c, q.x, q.y, q.z);
                    DrawCubeV(center, spaceCrftSize, Fade(DARKBLUE, 0.9f));
                    DrawCubeWiresV(center, spaceCrftSize, GREEN); 
                    DrawLine3D(center, endOfXSpcrft, RED);
                    DrawLine3D(center, endOfYSpcrft, RED);
                    DrawLine3D(center, endOfZSpcrft, GREEN);
                    
                    rlBegin(RL_TRIANGLES);
                        rlColor4ub(0, 228, 48, 255);
                        // Top face
                        rlVertex3f(center.x - spaceCrftSize.x/2, center.y + spaceCrftSize.y/2, center.z - spaceCrftSize.z/2);  // Top Left
                        rlVertex3f(center.x - spaceCrftSize.x/2, center.y + spaceCrftSize.y/2, center.z + spaceCrftSize.z/2);  // Bottom Left
                        rlVertex3f(center.x + spaceCrftSize.x/2, center.y + spaceCrftSize.y/2, center.z + spaceCrftSize.z/2);  // Bottom Right

                        rlVertex3f(center.x + spaceCrftSize.x/2, center.y + spaceCrftSize.y/2, center.z - spaceCrftSize.z/2);  // Top Right
                        rlVertex3f(center.x - spaceCrftSize.x/2, center.y + spaceCrftSize.y/2, center.z - spaceCrftSize.z/2);  // Top Left
                        rlVertex3f(center.x + spaceCrftSize.x/2, center.y + spaceCrftSize.y/2, center.z + spaceCrftSize.z/2);  // Bottom Right
                    rlEnd();
                    
                rlPopMatrix();

            EndMode3D();


            
            Vector2 xisp = GetWorldToScreen(RotatePoint(endOfXSpcrft, (Vector3){q.x, q.y, q.z}, c), camera);
            DrawText("Xb", xisp.x + 10, xisp.y, 30, RED);
            
            Vector2 yisp = GetWorldToScreen(RotatePoint(endOfYSpcrft, (Vector3){q.x, q.y, q.z}, c), camera);
            DrawText("Yb", yisp.x - 25, yisp.y, 30, RED);
            
            Vector2 zisp = GetWorldToScreen(RotatePoint(endOfZSpcrft, (Vector3){q.x, q.y, q.z}, c), camera);
            DrawText("Zb", zisp.x + 20, zisp.y, 30, GREEN);

            
            Vector2 xi = GetWorldToScreen(endOfXAxis, camera);
            DrawText("Xi", xi.x + 10, xi.y, 30, WHITE);
            
            Vector2 yi = GetWorldToScreen(endOfYAxis, camera);
            DrawText("Yi", yi.x - 25, yi.y, 30, WHITE);
            
            Vector2 zi = GetWorldToScreen(endOfZAxis, camera);
            DrawText("Zi", zi.x + 20, zi.y, 30, WHITE);

            EndDrawing();
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
