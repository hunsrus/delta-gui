#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso
//#define PLATFORM_DESKTOP
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include "rlgl.h"
#include "DeltaKinematics.h"

#include <iostream>
#include <stdio.h>

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif

#define TARGET_FPS 60
#define MARGIN 20
#define STARTING_DELAY 2.0f //[s]

#define CAMERA_FOV 90

#define ARM_LENGTH 90.0f
#define ROD_LENGTH 166.0f
#define BASS_TRI 35.0f
#define PLATFORM_TRI 42.0f
#define PLATFORM_POS (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}

static bool SHOW_FPS = true;
static bool STARTING_ANIMATION = true;

static Color COLOR_BG = {34,34,34,255};
static Color COLOR_FG = {238,238,238,255};

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1024;
    const int screenHeight = 768;

    int i;
    char c[100];
    float auxValue = 0;

    int animationStep = 0;
    int animationState = 0;

    DeltaKinematics dk = DeltaKinematics(ARM_LENGTH, ROD_LENGTH, BASS_TRI, PLATFORM_TRI);
    double x = 0, y = 0, z = -ROD_LENGTH/2.0f;
    double lastX = -1, lastY = -1, lastZ = -1;
    double thetaA, thetaB, thetaC;
    double rod1Phi, rod1Theta;
    double rod2Phi, rod2Theta;
    double rod3Phi, rod3Theta;

    Vector3 auxVector;

    Vector3 arm1Pos = Vector3Add(PLATFORM_POS,Vector3Scale(Vector3Normalize((Vector3){0.0f,0.0f,-1.0f}), PLATFORM_TRI));
    Vector3 arm2Pos = Vector3Add(PLATFORM_POS,Vector3Scale(Vector3Normalize((Vector3){1.0f*cos(30*DEG2RAD),0.0f,1.0f*sin(30*DEG2RAD)}), PLATFORM_TRI));
    Vector3 arm3Pos = Vector3Add(PLATFORM_POS,Vector3Scale(Vector3Normalize((Vector3){-1.0f*cos(30*DEG2RAD),0.0f,1.0f*sin(30*DEG2RAD)}), PLATFORM_TRI));
    Vector3 arm1Axis = {-1.0f,0.0f,0.0f};
    Vector3 arm2Axis = {1.0f*cos(60*DEG2RAD),0.0f,-1.0f*sin(60*DEG2RAD)};
    Vector3 arm3Axis = {1.0f*cos(60*DEG2RAD),0.0f,1.0f*sin(60*DEG2RAD)};
    Vector3 rod1Pos = (Vector3){arm2Pos.x,arm2Pos.y+ARM_LENGTH*sin(dk.b*DEG2RAD),arm2Pos.z};
    Vector3 rod2Pos = (Vector3){arm1Pos.x,arm1Pos.y+ARM_LENGTH*sin(dk.a*DEG2RAD),arm1Pos.z};
    Vector3 rod3Pos = (Vector3){arm1Pos.x,arm1Pos.y+ARM_LENGTH*sin(dk.c*DEG2RAD),arm1Pos.z};
    Vector3 baseJoint1, baseJoint2, baseJoint3;
    Vector3 basePos = (Vector3){x,z,y};
    Vector3 arm1Projection, arm2Projection, arm3Projection;

	//SetConfigFlags(FLAG_FULLSCREEN_MODE);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "delta gui test");

    // CARGAR LOS MODELOS DESPUÉS DE INICIAR LA VENTANA
	Model* deltaModel = new Model(LoadModel(std::string("../models/delta/delta.obj").c_str()));
	//bodyModel->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/fordfocus128.png").c_str()));
    //Model* platformModel = new Model(LoadModelFromMesh(GenMeshPoly(10,PLATFORM_TRI)));
    Model* platformModel = new Model(LoadModel(std::string("../models/platform/platform.obj").c_str()));
    platformModel->transform = MatrixScale(1000,1000,1000);
    platformModel->transform = MatrixMultiply(platformModel->transform, MatrixRotate((Vector3){0,1,0},45*DEG2RAD));
    platformModel->transform = MatrixMultiply(platformModel->transform, MatrixTranslate(0,-24,0));
    Model* baseModel = new Model(LoadModelFromMesh(GenMeshPoly(10,BASS_TRI)));
    Model* armModel = new Model(LoadModel(std::string("../models/arm/arm.obj").c_str()));
    //Model* armModel = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ARM_LENGTH)));
    Model* armModel1 = new Model(*armModel);
    Model* armModel2 = new Model(*armModel);
    Model* armModel3 = new Model(*armModel);
    Model* rodModel = new Model(LoadModel(std::string("../models/rod/rod.obj").c_str()));
    //Model* rodModel = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ROD_LENGTH)));
    rodModel->transform = MatrixScale(1000,1000,1000);
    rodModel->transform = MatrixMultiply(rodModel->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    Model* rodModel1 = new Model(*rodModel);
    Model* rodModel2 = new Model(*rodModel);
    Model* rodModel3 = new Model(*rodModel);
    //Model* armModel2 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ARM_LENGTH)));
    //Model* armModel1 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ARM_LENGTH)));
    //Model* armModel3 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ARM_LENGTH)));    
    //Model* rodModel1 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ROD_LENGTH)));
    //Model* rodModel2 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ROD_LENGTH)));
    //Model* rodModel3 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,4.0f,ROD_LENGTH)));
 
    // Define the camera to look into our 3d world
    //Camera camera = { {-20.0f, 12.0f, 0.0f}, { 0.0f, 4.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    Camera camera = { {-PLATFORM_TRI*10, (ARM_LENGTH+ROD_LENGTH)*1.5f, 0.0f}, {PLATFORM_POS.x, PLATFORM_POS.y/1.5f, PLATFORM_POS.z}, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 180.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);

    //SetCameraMode(camera, CAMERA_THIRD_PERSON);
	//SetCameraMode(camera, CAMERA_ORBITAL);
    SetCameraMode(camera, CAMERA_CUSTOM);
    HideCursor();
    SetTargetFPS(TARGET_FPS);
    //--------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------
    while (!WindowShouldClose())
    {
        //update image----------------------------------------
        UpdateCamera(&camera);      // Actualizar camara interna y mi camara

        if(STARTING_ANIMATION)
        {
            animationStep++;
            if(animationState == 1)
            {
                //camera.position.z = camera.position.z+0.0001f*animationStep/GetFrameTime();
                camera.fovy = camera.fovy-0.0001f*animationStep/GetFrameTime();
                if(camera.fovy <= CAMERA_FOV) STARTING_ANIMATION = false;
                //if(animationStep > 100) STARTING_ANIMATION = false;
            }
        }

        if(IsKeyDown(KEY_A))
        {
            y -= 1.0f;
        }
        if(IsKeyDown(KEY_D))
        {
            y += 1.0f;
        }
        if(IsKeyDown(KEY_S))
        {
            x -= 1.0f;
        }
        if(IsKeyDown(KEY_W))
        {
            x += 1.0f;
        }
        if(IsKeyDown(KEY_LEFT_SHIFT))
        {
            z -= 1.0f;
        }
        if(IsKeyDown(KEY_LEFT_CONTROL))
        {
            z += 1.0f;
        }

        if(IsKeyDown(KEY_SPACE))
        {
            auxValue += 1.0f;
            if(auxValue >= 360) auxValue = 0;
        }

        if(lastX != x || lastY != y || lastZ != z)
        {
            // Cálculos de cinemática
            dk.inverse(x,y,z);
            
            rod1Pos = (Vector3){arm1Pos.x,arm1Pos.y-ARM_LENGTH*sin(dk.a*DEG2RAD),arm1Pos.z-ARM_LENGTH*cos(dk.a*DEG2RAD)};
            rod2Pos = (Vector3){arm2Pos.x+ARM_LENGTH*cos(dk.b*DEG2RAD)*cos(30*DEG2RAD),arm2Pos.y-ARM_LENGTH*sin(dk.b*DEG2RAD),arm2Pos.z+ARM_LENGTH*cos(dk.b*DEG2RAD)*sin(30*DEG2RAD)};
            rod3Pos = (Vector3){arm3Pos.x-ARM_LENGTH*cos(dk.c*DEG2RAD)*cos(30*DEG2RAD),arm3Pos.y-ARM_LENGTH*sin(dk.c*DEG2RAD),arm3Pos.z+ARM_LENGTH*cos(dk.c*DEG2RAD)*sin(30*DEG2RAD)};
        
            float rod1Dist = sqrt(pow(rod1Pos.x,2.0f)+pow(rod1Pos.z,2.0f))-BASS_TRI;
            float rod2Dist = sqrt(pow(rod2Pos.x,2.0f)+pow(rod2Pos.z,2.0f))-sqrt(pow(basePos.x,2.0f)+pow(basePos.z,2.0f))-BASS_TRI;
            float rod3Dist = sqrt(pow(rod3Pos.x,2.0f)+pow(rod3Pos.z,2.0f))-BASS_TRI;

            basePos = (Vector3){x,PLATFORM_POS.y+z,y};

            // rod 1 angles
            arm1Projection = rod1Pos;
            arm1Projection.y = 0;
            arm1Projection = Vector3Normalize(arm1Projection);
            baseJoint1 = Vector3Scale(arm1Projection,BASS_TRI);
            baseJoint1 = Vector3Add(basePos,baseJoint1);

            Vector3 rod1Direction = Vector3Subtract(rod1Pos,baseJoint1);

            rod1Theta = atan2(rod1Direction.z,rod1Direction.x)+90*DEG2RAD;
            rod1Phi = Vector3Angle(rod1Direction,(Vector3){0,1.0f,0})+90*DEG2RAD;

            // rod 2 angles
            arm2Projection = rod2Pos;
            arm2Projection.y = 0;
            arm2Projection = Vector3Normalize(arm2Projection);
            baseJoint2 = Vector3Scale(arm2Projection,BASS_TRI);
            baseJoint2 = Vector3Add(basePos,baseJoint2);

            Vector3 rod2Direction = Vector3Subtract(rod2Pos,baseJoint2);

            rod2Theta = atan2(rod2Direction.z,rod2Direction.x)-30*DEG2RAD;
            rod2Phi = Vector3Angle(rod2Direction,(Vector3){0,1.0f,0})+90*DEG2RAD;

            // rod 3 angles
            arm3Projection = rod3Pos;
            arm3Projection.y = 0;
            arm3Projection = Vector3Normalize(arm3Projection);
            baseJoint3 = Vector3Scale(arm3Projection,BASS_TRI);
            baseJoint3 = Vector3Add(basePos,baseJoint3);

            Vector3 rod3Direction = Vector3Subtract(rod3Pos,baseJoint3);
            auxVector = Vector3Scale(arm3Projection,BASS_TRI);
            auxVector.y = baseJoint3.y;

            rod3Theta = atan2(rod3Direction.z,rod3Direction.x)-150*DEG2RAD;
            rod3Phi = Vector3Angle(rod3Direction,(Vector3){0,1.0f,0})+90*DEG2RAD;

            // transformations

            //// arms

            armModel1->transform = MatrixScale(1000,1000,1000);
            //armModel1->transform = MatrixMultiply(armModel1->transform,MatrixRotate((Vector3){1,0,0}, M_PI_2));
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
            //armModel1->transform = MatrixMultiply(armModel1->transform,MatrixTranslate(0, 0, -ARM_LENGTH/2.0f));
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixRotate(arm1Axis,dk.a*DEG2RAD));
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixTranslate(arm1Pos.x,arm1Pos.y,arm1Pos.z));

            armModel2->transform = MatrixScale(1000,1000,1000);
            //armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate((Vector3){1,0,0}, M_PI_2));
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate((Vector3){0.0f,1.0f,0.0f},-120.0f*DEG2RAD));
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate(arm2Axis,dk.b*DEG2RAD));
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixTranslate(arm2Pos.x,arm2Pos.y,arm2Pos.z));

            armModel3->transform = MatrixScale(1000,1000,1000);
            //armModel3->transform = MatrixMultiply(armModel3->transform,MatrixRotate((Vector3){1,0,0}, M_PI_2));
            armModel3->transform = MatrixMultiply(armModel3->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
            armModel3->transform = MatrixMultiply(armModel3->transform, MatrixRotate((Vector3){0.0f,1.0f,0.0f},120.0f*DEG2RAD));
            armModel3->transform = MatrixMultiply(armModel3->transform,MatrixRotate(arm3Axis,dk.c*DEG2RAD));
            armModel3->transform = MatrixMultiply(armModel3->transform,MatrixTranslate(arm3Pos.x,arm3Pos.y,arm3Pos.z));

            //// rods

            rodModel1->transform = MatrixScale(1000,1000,1000);
            //rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixRotate((Vector3){1,0,0}, M_PI_2));
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
            //rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixTranslate(0, 0, -ROD_LENGTH/2.0f));
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixRotate(arm1Axis,rod1Phi));
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixRotate((Vector3){0,-1,0},rod1Theta));
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixTranslate(rod1Pos.x, rod1Pos.y, rod1Pos.z));
            
            //rodModel2->transform = MatrixTranslate(0, 0, -ROD_LENGTH/2.0f);
            rodModel2->transform = MatrixScale(1000,1000,1000);
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixRotate((Vector3){0.0f,1.0f,0.0f},-120.0f*DEG2RAD));
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixRotate(arm2Axis,rod2Phi));
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixRotate((Vector3){0,-1,0},rod2Theta));
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixTranslate(rod2Pos.x, rod2Pos.y, rod2Pos.z));

            //rodModel3->transform = MatrixTranslate(0, 0, -ROD_LENGTH/2.0f);
            rodModel3->transform = MatrixScale(1000,1000,1000);
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixRotate((Vector3){0,1,0}, M_PI_2));
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixRotate((Vector3){0.0f,1.0f,0.0f},120.0f*DEG2RAD));
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixRotate(arm3Axis,rod3Phi));
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixRotate((Vector3){0,-1,0},rod3Theta));
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixTranslate(rod3Pos.x, rod3Pos.y, rod3Pos.z));
        }

        lastX = x;
        lastY = y;
        lastZ = z;

        //armModel->transform = MatrixRotateXYZ((Vector3){ 0, 0, x });
        //----------------------------------------------------------------------------------
        // Dibuja
        //----------------------------------------------------------------------------------

        ClearBackground(COLOR_BG);  // Clear texture background

        BeginTextureMode(target);       // Enable drawing to texture
            ClearBackground(COLOR_BG);  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                //DrawGrid(20,1.0f);
                DrawGrid((int)PLATFORM_TRI/1.5f,PLATFORM_TRI);
                //DrawModel(*platformModel,PLATFORM_POS,1.0f,RED);
                DrawModel(*platformModel,PLATFORM_POS,1.0f,WHITE);
                DrawModel(*baseModel,basePos,1.0f,RED);
                //DrawModelEx(*armModel1, arm1Pos, arm1Axis, dk.a, Vector3One(), YELLOW);
                DrawModel(*armModel1, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*armModel2, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*armModel3, Vector3Zero(), 1.0f, WHITE);
                //DrawModel(*armModel, Vector3Zero(), 1000.0f, WHITE);
                //DrawModelEx(*rodModel1, rod1Pos, arm1Axis, rod1Theta, Vector3One(), YELLOW);
                //DrawModel(*rodModel, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*rodModel1, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*rodModel2, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*rodModel3, Vector3Zero(), 1.0f, WHITE);
                //DrawModelEx(*rodModel2, rod2Pos, arm2Axis, rod2Theta, Vector3One(), BLUE);
                //DrawModelEx(*rodModel3, rod3Pos, arm3Axis, rod3Theta, Vector3One(), GREEN);
                DrawCircle3D(PLATFORM_POS,PLATFORM_TRI,(Vector3){1.0f,0.0f,0.0f},90.0f,WHITE);
                //DrawModel(*deltaModel,PLATFORM_POS,1000.0f,WHITE);
                //DrawPlane((Vector3){0.0f,0.0f,0.0f},(Vector2){100.0f,100.0f},WHITE);
                DrawLine3D(auxVector,rod3Pos, ORANGE);
                DrawLine3D(baseJoint3,rod3Pos, GREEN);
            EndMode3D();                // End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode();               // End drawing to texture (now we have a texture available for next passes)

        BeginDrawing();
            ClearBackground(COLOR_BG);
            if(GetTime() > STARTING_DELAY)
            {
                // BeginShaderMode(shader_pixel);
                    // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                    Vector2 viewSize = {(float)target.texture.width/4, (float)target.texture.height/2};
                    Rectangle viewRectangle = {(float)target.texture.width/2-viewSize.x/2, (float)target.texture.height/2-viewSize.y/2, viewSize.x, -viewSize.y};
                    Vector2 viewPos = { screenWidth-viewSize.x-MARGIN, MARGIN};
                    DrawTextureRec(target.texture, viewRectangle, viewPos, WHITE);
                    Rectangle viewBorderRectangle = {viewPos.x, viewPos.y, viewSize.x, viewSize.y};
                    DrawRectangleLinesEx(viewBorderRectangle,2.0f,COLOR_FG);
                // EndShaderMode();

                //sprintf(c,"ANGLE: %f",auxValue);
                //DrawText(c, MARGIN, MARGIN*5, MARGIN, COLOR_FG);

                if(SHOW_FPS)
                {
                    sprintf(c,"FPS %d",GetFPS());
                    DrawText(c, MARGIN, MARGIN, MARGIN, COLOR_FG);
                    sprintf(c,"STARTING_ANIMATION %i",STARTING_ANIMATION);
                    DrawText(c, MARGIN, MARGIN*3, MARGIN, COLOR_FG);
                }
                if(STARTING_ANIMATION && animationState == 0)
                {
                    int startingCoverX = 0.1f*animationStep/GetFrameTime();
                    DrawRectangle(startingCoverX,0,screenWidth,screenHeight,COLOR_BG);
                    if(startingCoverX >= screenWidth)
                    {
                        animationState = 1;
                        animationStep = 0;
                    }
                }
            }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    CloseWindow();
    //--------------------------------------------------------------------------------------

    //cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

    UnloadModel(*deltaModel);

    return 0;
}