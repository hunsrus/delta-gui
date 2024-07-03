#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso
//#define PLATFORM_DESKTOP
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include "rlgl.h"

#include <iostream>
#include <stdio.h>

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif

#define TARGET_FPS 60
#define MARGIN 20

static bool SHOW_FPS = true;

static Color COLOR_BG = {34,34,34,255};
static Color COLOR_FG = {238,238,238,255};

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 600;

    int i;
    char c[100];

	//SetConfigFlags(FLAG_FULLSCREEN_MODE);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "delta gui test");

    // CARGAR LOS MODELOS DESPUÉS DE INICIAR LA VENTANA
	Model* deltaModel = new Model(LoadModel(std::string("../models/delta/delta.obj").c_str()));
	//bodyModel->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/fordfocus128.png").c_str()));
 
    // Define the camera to look into our 3d world
    Camera camera = { {-20.0f, 12.0f, 0.0f}, { 0.0f, 4.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);

    //SetCameraMode(camera, CAMERA_THIRD_PERSON);
	SetCameraMode(camera, CAMERA_ORBITAL);
    HideCursor();

    SetTargetFPS(TARGET_FPS);
    //--------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------
    while (!WindowShouldClose())
    {
        //update image----------------------------------------
        UpdateCamera(&camera);      // Actualizar camara interna y mi camara

	
        //----------------------------------------------------------------------------------
        // Dibuja
        //----------------------------------------------------------------------------------

        ClearBackground(COLOR_BG);  // Clear texture background

        BeginTextureMode(target);       // Enable drawing to texture
            ClearBackground(COLOR_BG);  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                DrawGrid(20,20.0f);
                DrawModel(*deltaModel,(Vector3){0,5,0},20.0f,WHITE);
                //DrawPlane((Vector3){0.0f,0.0f,0.0f},(Vector2){100.0f,100.0f},WHITE);
            EndMode3D();                // End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode();               // End drawing to texture (now we have a texture available for next passes)

        BeginDrawing();
            ClearBackground(COLOR_BG);
                // BeginShaderMode(shader_pixel);
                    // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                    Vector2 viewSize = {(float)target.texture.width/4, (float)target.texture.height/2};
                    Rectangle viewRectangle = {(float)target.texture.width/2-viewSize.x/2, (float)target.texture.height/2-viewSize.y/2, viewSize.x, -viewSize.y};
                    Vector2 viewPos = { screenWidth-viewSize.x-MARGIN, MARGIN};
                    DrawTextureRec(target.texture, viewRectangle, viewPos, WHITE);
                    Rectangle viewBorderRectangle = {viewPos.x, viewPos.y, viewSize.x, viewSize.y};
                    DrawRectangleLinesEx(viewBorderRectangle,3.0f,COLOR_FG);
                // EndShaderMode();

            if(SHOW_FPS)
            {
                sprintf(c,"FPS %d",GetFPS());
                DrawText(c, MARGIN, MARGIN, MARGIN, COLOR_FG);
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