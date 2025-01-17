#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso

//#define PLATFORM_DESKTOP
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include "rlgl.h"
// #include "DeltaKinematics.h"
#include "OctoKinematics.h"

// deteccion de imagenes
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <vector>

#if ARCH_ARM
    #include <pigpio.h>
#endif

// robot dimensions
#define ARM_LENGTH 130.0f
#define ROD_LENGTH 310.0f
#define EFF_RADIUS 35.0f
#define BAS_RADIUS 117.0f
#define BAS_POSITION (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}
#define HOME_Z -170.0f
#define LIM_Z -273.0f

#define TRANS_MULTIPLIER 3
#define STEPS_NUM 8

// pinout definitions
#define PIN_DIR1 12
#define PIN_STEP1 16
#define PIN_DIR2 21
#define PIN_STEP2 20
#define PIN_DIR3 18
#define PIN_STEP3 23
#define PIN_DIR4 22
#define PIN_STEP4 27

#define PIN_MS1 17
#define PIN_MS2 4
#define PIN_MS3 3

#define PIN_REG_PL 7
#define PIN_REG_CP 0
#define PIN_REG_OUT 1

#define REG_BIT_Y0 0
#define REG_BIT_Y1 1
#define REG_BIT_X0 2
#define REG_BIT_X1 3
#define REG_BIT_R3 4
#define REG_BIT_R1 5
#define REG_BIT_R2 6

#define PIN_FC_M1 5
#define PIN_FC_M2 6
#define PIN_FC_M3 13

#define PIN_BOMBA 26

// graphics config
#define GLSL_VERSION 100

#if ARCH_ARM
    #define DISPLAY_WIDTH 320
    #define DISPLAY_HEIGHT 240
    #define BORDER_THICKNESS 2.0f
#else
    #define DISPLAY_WIDTH 320
    #define DISPLAY_HEIGHT 240
    #define BORDER_THICKNESS 2.0f
#endif
#define TARGET_FPS 30
#define MARGIN 20*(DISPLAY_HEIGHT/768.0f)
#define FONT_PIXELS 24*DISPLAY_HEIGHT/240

#define CAMERA_FOV 65
#define DRAW_SCALE 0.5

static bool SHOW_DEBUG_DATA = false;
static bool STARTING_ANIMATION = true;

static Color COLOR_BG = {34,34,34,255};
static Color COLOR_FG = {238,238,238,255};
static Color COLOR_HL = ORANGE;

// image detection definitions
static cv::Mat image;
static bool CAMERA_AVAILABLE = true;
static bool CAPTURE_READY = false;

static bool EXIT = false;

// Determinar el paso actual
static int stepIndex = 0; // BORRAR?

// interfaz de usuario
const int MENUS_AMOUNT = 5;
int currentMenuID = 0;
std::vector<const char*>::iterator highlightedMenu;

typedef struct Screen{
    int id;
    const char* title;
    Screen* parent;
    std::vector<const char*> options;
}Screen;

Image MatToImage(const cv::Mat &mat) {
    // Asegúrate de que la imagen está en formato RGB
    cv::Mat matRGB;
    cv::cvtColor(mat, matRGB, cv::COLOR_BGR2RGB);

    // Crea un Image de Raylib
    Image image = {
        .data = malloc(matRGB.total() * matRGB.elemSize()),
        .width = matRGB.cols,
        .height = matRGB.rows,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8 // O PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 si necesitas alfa
    };

    // Copia los datos de la matriz de OpenCV al array de Raylib
    memcpy(image.data, matRGB.data, matRGB.total() * matRGB.elemSize());

    return image;
}

int captureVideo(void)
{
    //read video
    cv::VideoCapture capture;
    capture.open("/dev/video0");
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 128);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 128);

    double dWidth = capture.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    std::cout << "camera width = " << dWidth << ", height = " << dHeight << std::endl;

    if (!capture.isOpened()) { //check if video device has been initialised
        std::cout << "cannot open camera";
    }

    bool bSuccess = capture.read(image); // read a new frame from video

    if (bSuccess == false)
    {
        std::cout << "Video camera is disconnected" << std::endl;
        CAMERA_AVAILABLE = false;
        return EXIT_FAILURE;
    }

    CAPTURE_READY = true;

    while (!EXIT)
    {
        if(!CAPTURE_READY && CAMERA_AVAILABLE)
        {
            bool bSuccess = capture.read(image); // read a new frame from video

            if (bSuccess == false)
            {
                std::cout << "Video camera is disconnected" << std::endl;
                CAMERA_AVAILABLE = false;
                return EXIT_FAILURE;
            }
            CAPTURE_READY = true;
        }
    }

    return EXIT_SUCCESS;
}

char readRegister(int pin_data, int pin_pl, int pin_cp)
{
    char data = 0;

    // generar un pulso en PL para cargar los datos del registro paralelo al registro de desplazamiento
    #if ARCH_ARM
    gpioWrite(pin_pl, 0);
    gpioWrite(pin_pl, 1);

    // leer los 8 bits del registro desplazándolos al acumulador "data"
    for (int i = 0; i < 8; i++) {
        // leer el bit actual desde el pin de salida
        int bit = gpioRead(pin_data);

        // colocar el bit leído en la posición correspondiente en "data"
        data |= (bit << (7 - i));

        // generar un pulso en el pin CP para avanzar al siguiente bit
        gpioWrite(pin_cp, 1);
        gpioWrite(pin_cp, 0);
    }
    #endif

    return data;
}

int readBit(char data, int bit) {
    return (data >> bit) & 0x01;
    //return (data >> (7 - bit)) & 0x01;
}

int calculateKinematics(double &x,double &y,double &z, OctoKinematics &octoKin)
{
    float low_z = LIM_Z;//-294
    float high_z = LIM_Z+10;

    // dibujar grilla
	double stepDist = 0.002;
	for(int i=-30; i<=30; i=i+3)
    {
	  x = i;
	  y = -30;
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	  x = i;
	  y = 30;
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	}
	for(int i=-30; i<=30; i=i+3)
    {
	  x = -30;
	  y = i;
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	  x = 30;
	  y = i;
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
      if(EXIT) return EXIT_FAILURE;
	}

	// Ubicar componentes
	std::vector<double> vect1_x = {-30.0, -30.0, -30.0, -30.0, -30.0, -30.0};
	std::vector<double> vect1_y = {0.0, -3.0, -6.0, -9.0, -12.0, -15.0};
	std::vector<double> vect2_x = {-15.0, -24.0, -7.0, -1.0, -10.0, -14.0};
	std::vector<double> vect2_y = {-15.0, -8.0, -23.0, -30.0, -7.0, -1.0};
	stepDist = 0.0008;
	for(int i=0; i<=5; i++)
    {
	 x = vect1_x[i];
	 y = vect1_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 gpioWrite(PIN_BOMBA,1);
	 usleep(25000);
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 x = vect2_x[i];
	 y = vect2_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(25000);
     if(EXIT) return EXIT_FAILURE;
	 gpioWrite(PIN_BOMBA,0);
	 usleep(250000);
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
     if(EXIT) return EXIT_FAILURE;
	 usleep(250000);
	}
	for(int i=0; i<=5; i++)
    {
	 x = vect2_x[i];
	 y = vect2_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
     if(EXIT) return EXIT_FAILURE;
	 usleep(250000);
	 gpioWrite(PIN_BOMBA,1);
	 usleep(25000);
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 x = vect1_x[i];
	 y = vect1_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(25000);
     if(EXIT) return EXIT_FAILURE;
	 gpioWrite(PIN_BOMBA,0);
	 usleep(250000);
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
     if(EXIT) return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}

void DrawProgressBarScreen(const char* text, int progress, Font font)
{
    HideCursor();
    float fontSize = font.baseSize/2.0f;
    Vector2 barSize = {DISPLAY_WIDTH*0.8, DISPLAY_HEIGHT*0.1};
    Vector2 barPos = {DISPLAY_WIDTH/2-barSize.x/2, DISPLAY_HEIGHT/2-barSize.y/2};
    Vector2 textPos = {barPos.x,barPos.y-fontSize};
    BeginDrawing();
        ClearBackground(COLOR_BG);
        Rectangle barRec = {barPos.x, barPos.y, barSize.x, barSize.y};
        DrawRectangleLinesEx(barRec, BORDER_THICKNESS, COLOR_FG);
        barRec.width *= progress/100.0f;
        DrawRectangleRec(barRec,COLOR_FG);
        DrawTextEx(font, text, textPos, fontSize, 1, COLOR_FG);
    EndDrawing();
}

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = DISPLAY_WIDTH;
    const int screenHeight = DISPLAY_HEIGHT;

    //SetConfigFlags(FLAG_FULLSCREEN_MODE);
    //SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_UNDECORATED);
    InitWindow(screenWidth, screenHeight, "delta gui test");

    Font font = LoadFontEx("resources/fonts/JetBrainsMono/JetBrainsMono-Bold.ttf", FONT_PIXELS, 0, 250);
    float fontSize = font.baseSize;//DISPLAY_HEIGHT/20;

    HideCursor();
    SetTargetFPS(TARGET_FPS);

    DrawProgressBarScreen("inicializando variables...", 10, font);
    int i;
    char c[100];
    float auxValue = 0;

    int animationStep = 0;
    int animationState = 0;

    char inputRegData = 255;
    bool axis_state_X0 = 0;
    bool axis_state_X1 = 0;
    bool axis_state_Y0 = 0;
    bool axis_state_Y1 = 0;
    bool button_state_R1 = 1;
    bool button_state_R2 = 1;
    bool button_state_R3 = 1;
    int axis_state_X = 0;
    int axis_state_Y = 0;
    int last_axis_state_X = 0;
    int last_axis_state_Y = 0;
    bool last_button_state_R1 = 1;
    bool last_button_state_R2 = 1;
    bool last_button_state_R3 = 1;

    DrawProgressBarScreen("inicializando cinemática...", 20, font);
    OctoKinematics octoKin = OctoKinematics(ARM_LENGTH, ROD_LENGTH, EFF_RADIUS, BAS_RADIUS);
    double x = 0, y = 0, z = HOME_Z;
    double lastX = -1, lastY = -1, lastZ = -1;
    double rod1Phi, rod1Theta;
    double rod2Phi, rod2Theta;
    double rod3Phi, rod3Theta;

    Vector3 auxVector;
    
    Vector3 arm1Pos = Vector3Add(BAS_POSITION,Vector3Scale(Vector3Normalize((Vector3){0.0f,0.0f,-1.0f}), BAS_RADIUS));
    Vector3 arm2Pos = Vector3Add(BAS_POSITION,Vector3Scale(Vector3Normalize((Vector3){1.0f*cos(30*DEG2RAD),0.0f,1.0f*sin(30*DEG2RAD)}), BAS_RADIUS));
    Vector3 arm3Pos = Vector3Add(BAS_POSITION,Vector3Scale(Vector3Normalize((Vector3){-1.0f*cos(30*DEG2RAD),0.0f,1.0f*sin(30*DEG2RAD)}), BAS_RADIUS));
    Vector3 arm1Axis = {-1.0f,0.0f,0.0f};
    Vector3 arm2Axis = {1.0f*cos(60*DEG2RAD),0.0f,-1.0f*sin(60*DEG2RAD)};
    Vector3 arm3Axis = {1.0f*cos(60*DEG2RAD),0.0f,1.0f*sin(60*DEG2RAD)};
    Vector3 rod1Pos = (Vector3){arm2Pos.x,static_cast<float>(arm2Pos.y+ARM_LENGTH*sin(-octoKin.c*DEG2RAD)),arm2Pos.z};
    Vector3 rod2Pos = (Vector3){arm1Pos.x,static_cast<float>(arm1Pos.y+ARM_LENGTH*sin(-octoKin.b*DEG2RAD)),arm1Pos.z};
    Vector3 rod3Pos = (Vector3){arm1Pos.x,static_cast<float>(arm1Pos.y+ARM_LENGTH*sin(-octoKin.a*DEG2RAD)),arm1Pos.z};
    Vector3 baseJoint1, baseJoint2, baseJoint3;
    Vector3 basePos = (Vector3){static_cast<float>(x),static_cast<float>(z),static_cast<float>(y)};
    Vector3 arm1Projection, arm2Projection, arm3Projection;

    DrawProgressBarScreen("cargando menú...", 30, font);
    Screen menu[MENUS_AMOUNT];
    menu[0].id = 0;
    menu[0].title = "Menú principal";
    menu[0].options.push_back("Debug");
    menu[0].options.push_back("Trabajos");
    menu[0].options.push_back("Calibración");
    menu[0].options.push_back("Interfaz");
    menu[0].options.push_back("Salir");
    menu[1].id = 1;
    menu[1].title = "Trabajos";
    menu[1].parent = &menu[0];
    menu[1].options.push_back("Atrás");
    menu[1].options.push_back("Iniciar rutina");
    menu[1].options.push_back("Archivo");
    menu[1].options.push_back("Componentes");
    menu[1].options.push_back("Referencias");
    menu[1].options.push_back("Guardar rutina");
    menu[1].options.push_back("Abrir rutina");
    menu[2].id = 2;
    menu[2].title = "Calibración";
    menu[2].parent = &menu[0];
    menu[2].options.push_back("Atrás");
    menu[3].id = 3;
    menu[3].title = "Interfaz";
    menu[3].parent = &menu[0];
    menu[3].options.push_back("Atrás");
    menu[4].id = 4;
    menu[4].title = "Salir";
    menu[4].parent = &menu[0];
    menu[4].options.push_back("Atrás");
    menu[4].options.push_back("Apagar");
    menu[4].options.push_back("Reiniciar");
    
    highlightedMenu = menu[currentMenuID].options.begin();

    #if ARCH_ARM
        std::cout << "ARCHITECTURE: ARM" << std::endl;
    #else
        std::cout << "ARCHITECTURE: x86_64" << std::endl;
    #endif

    DrawProgressBarScreen("generando modelos 3D...", 40, font);
    // CARGAR LOS MODELOS DESPUÉS DE INICIAR LA VENTANA
    Model* platformModel = new Model(LoadModelFromMesh(GenMeshPoly(20,BAS_RADIUS)));
    //Model* platformModel = new Model(LoadModel(std::string("resources/models/platform/platform.obj").c_str()));
    //platformModel->transform = MatrixScale(1000,1000,1000);
    //platformModel->transform = MatrixMultiply(platformModel->transform, MatrixRotate((Vector3){0,1,0},45*DEG2RAD));
    //platformModel->transform = MatrixMultiply(platformModel->transform, MatrixTranslate(0,-24,0));
    Model* baseModel = new Model(LoadModelFromMesh(GenMeshPoly(10,EFF_RADIUS)));
    //Model* baseModel = new Model(LoadModel(std::string("resources/models/effector/effector.obj").c_str()));
    //baseModel->transform = MatrixScale(1000,1000,1000);
    //baseModel->transform = MatrixMultiply(baseModel->transform, MatrixRotate((Vector3){0,1,0},45*DEG2RAD));
    
    float modelRadius = 16.0f;
    Model* armModel1 = new Model(LoadModelFromMesh(GenMeshCylinder(modelRadius,ARM_LENGTH,10)));
    Model* armModel2 = new Model(LoadModelFromMesh(GenMeshCylinder(modelRadius,ARM_LENGTH,10)));
    Model* armModel3 = new Model(LoadModelFromMesh(GenMeshCylinder(modelRadius,ARM_LENGTH,10)));
    Matrix arm1InitialMatrix = MatrixIdentity();
    Matrix arm2InitialMatrix = MatrixIdentity();
    Matrix arm3InitialMatrix = MatrixIdentity();
    arm1InitialMatrix = MatrixMultiply(arm1InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    arm2InitialMatrix = MatrixMultiply(arm2InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    arm3InitialMatrix = MatrixMultiply(arm3InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    /*
    Model* armModel1 = new Model(LoadModel(std::string("resources/models/arm/simplify_arm.obj").c_str()));
    Model* armModel2 = new Model(LoadModel(std::string("resources/models/arm/simplify_arm.obj").c_str()));
    Model* armModel3 = new Model(LoadModel(std::string("resources/models/arm/simplify_arm.obj").c_str()));
    */
    Model* rodModel1 = new Model(LoadModelFromMesh(GenMeshCylinder(modelRadius,ROD_LENGTH,10)));
    Model* rodModel2 = new Model(LoadModelFromMesh(GenMeshCylinder(modelRadius,ROD_LENGTH,10)));
    Model* rodModel3 = new Model(LoadModelFromMesh(GenMeshCylinder(modelRadius,ROD_LENGTH,10)));
    Matrix rod1InitialMatrix = MatrixIdentity();
    Matrix rod2InitialMatrix = MatrixIdentity();
    Matrix rod3InitialMatrix = MatrixIdentity();
    rod1InitialMatrix = MatrixMultiply(rod1InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    rod2InitialMatrix = MatrixMultiply(rod2InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    rod3InitialMatrix = MatrixMultiply(rod3InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    /*
    Model* rodModel1 = new Model(LoadModel(std::string("resources/models/rod/simplify_rod.obj").c_str()));
    Model* rodModel2 = new Model(LoadModel(std::string("resources/models/rod/simplify_rod.obj").c_str()));
    Model* rodModel3 = new Model(LoadModel(std::string("resources/models/rod/simplify_rod.obj").c_str()));
    */

    armModel1->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = YELLOW;
    armModel2->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = ORANGE;
    armModel3->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BROWN;
    rodModel1->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = YELLOW;
    rodModel2->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = ORANGE;
    rodModel3->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BROWN;

    //Matrix arm1InitialMatrix = MatrixScale(1000,1000,1000);
    arm1InitialMatrix = MatrixMultiply(arm1InitialMatrix,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    //Matrix arm2InitialMatrix = MatrixScale(1000,1000,1000);
    arm2InitialMatrix = MatrixMultiply(arm2InitialMatrix,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    arm2InitialMatrix = MatrixMultiply(arm2InitialMatrix,MatrixRotate((Vector3){0.0f,1.0f,0.0f},-120.0f*DEG2RAD));
    //Matrix arm3InitialMatrix = MatrixScale(1000,1000,1000);
    arm3InitialMatrix = MatrixMultiply(arm3InitialMatrix,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    arm3InitialMatrix = MatrixMultiply(arm3InitialMatrix, MatrixRotate((Vector3){0.0f,1.0f,0.0f},120.0f*DEG2RAD));
    //Matrix rod1InitialMatrix = MatrixScale(1000,1000,1000);
    rod1InitialMatrix = MatrixMultiply(rod1InitialMatrix,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    //Matrix rod2InitialMatrix = MatrixScale(1000,1000,1000);
    rod2InitialMatrix = MatrixMultiply(rod2InitialMatrix,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    rod2InitialMatrix = MatrixMultiply(rod2InitialMatrix,MatrixRotate((Vector3){0.0f,1.0f,0.0f},-120.0f*DEG2RAD));
    //Matrix rod3InitialMatrix = MatrixScale(1000,1000,1000);
    rod3InitialMatrix = MatrixMultiply(rod3InitialMatrix,MatrixRotate((Vector3){0,1,0}, M_PI_2));
    rod3InitialMatrix = MatrixMultiply(rod3InitialMatrix,MatrixRotate((Vector3){0.0f,1.0f,0.0f},120.0f*DEG2RAD));

    DrawProgressBarScreen("definiendo vista 3D...", 50, font);
    // Define the camera to look into our 3d world
    //Camera camera = { {-20.0f, 12.0f, 0.0f}, { 0.0f, 4.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    Camera camera = { {-BAS_RADIUS*3.0f, (ARM_LENGTH+ROD_LENGTH)*1.0f, 0.0f}, {BAS_POSITION.x, BAS_POSITION.y/3.0f, BAS_POSITION.z}, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 179.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DrawProgressBarScreen("cargando shaders...", 60, font);
    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D renderTextureModel = LoadRenderTexture(screenWidth, screenHeight);
    RenderTexture2D renderTextureBackground = LoadRenderTexture(screenWidth, screenHeight);
    // cargar shader para outline 
    Shader shader = LoadShader(TextFormat("resources/shaders/glsl%i/base.vs", GLSL_VERSION), TextFormat("resources/shaders/glsl%i/sobel_colors.fs", GLSL_VERSION));
    // // Get shader locations
    Vector4 normalizedFGColor = ColorNormalize(COLOR_FG);
    Vector4 normalizedBGColor = ColorNormalize(COLOR_BG);
    float edgeColor[3] = {normalizedFGColor.x, normalizedFGColor.y, normalizedFGColor.z};
    float backgroundColor[3] = {normalizedBGColor.x, normalizedBGColor.y, normalizedBGColor.z};
    float shaderResolution[2] = {300, 300};
    int edgeColorLoc = GetShaderLocation(shader, "edgeColor");
    int backgroundColorLoc = GetShaderLocation(shader, "backgroundColor");
    int resolutionLoc = GetShaderLocation(shader, "resolution");
    SetShaderValue(shader, edgeColorLoc, edgeColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, backgroundColorLoc, backgroundColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, resolutionLoc, shaderResolution, SHADER_UNIFORM_VEC2);

    //SetCameraMode(camera, CAMERA_THIRD_PERSON);
	//SetCameraMode(camera, CAMERA_ORBITAL);
    SetCameraMode(camera, CAMERA_CUSTOM);

    DrawProgressBarScreen("inicializando puertos i/o...", 70, font);
    // inicio control de i/o
    #if ARCH_ARM
        if (gpioInitialise() < 0)
        {
            fprintf(stderr, "pigpio initialisation failed\n");
            return 1;
        }
        fprintf(stdout, "pigpio initialisation complete\n");

        // config pinout
        gpioSetMode(PIN_DIR1,PI_OUTPUT);
        gpioSetMode(PIN_STEP1,PI_OUTPUT);
        gpioSetMode(PIN_DIR2,PI_OUTPUT);
        gpioSetMode(PIN_STEP2,PI_OUTPUT);
        gpioSetMode(PIN_DIR3,PI_OUTPUT);
        gpioSetMode(PIN_STEP3,PI_OUTPUT);
        
        gpioSetMode(PIN_MS1,PI_OUTPUT);
        gpioSetMode(PIN_MS2,PI_OUTPUT);
        gpioSetMode(PIN_MS3,PI_OUTPUT);

        gpioSetMode(PIN_REG_PL,PI_OUTPUT);
        gpioSetMode(PIN_REG_CP,PI_OUTPUT);
        gpioSetMode(PIN_REG_OUT,PI_INPUT);

        gpioSetMode(PIN_FC_M1,PI_INPUT);
        gpioSetMode(PIN_FC_M2,PI_INPUT);
        gpioSetMode(PIN_FC_M3,PI_INPUT);

        gpioSetMode(PIN_BOMBA,PI_OUTPUT);

        // config robot pins
        octoKin.set_pin_step_ctrl(PIN_MS1,PIN_MS2,PIN_MS3);
        octoKin.set_pin_motor_1(PIN_STEP1, PIN_DIR1);
        octoKin.set_pin_motor_2(PIN_STEP2, PIN_DIR2);
        octoKin.set_pin_motor_3(PIN_STEP3, PIN_DIR3);
        octoKin.set_pin_limit_sw(PIN_FC_M1, PIN_FC_M2, PIN_FC_M3);
        octoKin.set_axis_direction(1);
        octoKin.set_step_precision(STEPS_NUM);
        octoKin.set_transmission_ratio(TRANS_MULTIPLIER);
        octoKin.set_pulse_width(1500);
        
        // turn suction off
        gpioWrite(PIN_BOMBA,0);

        // effector orientation correction
        int effector_steps = 0;
        std::string nombreArchivo = "initdata";

        std::ifstream archivoEntrada(nombreArchivo); // Abrir archivo en modo lectura
        if (archivoEntrada.is_open()) {
            archivoEntrada >> effector_steps; // Leer el valor del archivo
            archivoEntrada.close();            // Cerrar el archivo
            std::cout << "Correción de efector: " << effector_steps << '\n';
        } else {
            std::cerr << "No se pudo abrir el archivo para leer.\n";
            //return 1; // Error
        }

        while(effector_steps)
        {
            if(effector_steps > 0)
            {
                octoKin.step(PIN_STEP4, PIN_DIR4, 0);
                effector_steps--;
            }
            if(effector_steps < 0)
            {
                octoKin.step(PIN_STEP4, PIN_DIR4, 1);
                effector_steps++;
            }
        }

        std::ofstream archivoSalida(nombreArchivo); // Crear y abrir archivo en modo escritura

    #endif

    DrawProgressBarScreen("secuencia de home...", 80, font);
    // homing sequence
    octoKin.home(0,0,HOME_Z); 
    // move to starting position
    x = 0;
    y = 0;
    z = -220;
    octoKin.inverse_kinematics(x, y, z);
    octoKin.updateKinematics();
    lastX = x;
    lastY = y;
    lastZ = z;

    std::cout << "a: " << octoKin.a << std::endl;
    std::cout << "b: " << octoKin.b << std::endl;
    std::cout << "c: " << octoKin.c << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;

    // DrawProgressBarScreen("iniciando video...", 90, font);
    // // inicio captura de video
    // Texture2D captureTexture;

    // std::thread t1(captureVideo);

    // while(!CAPTURE_READY && CAMERA_AVAILABLE){};
    // if(CAMERA_AVAILABLE)
    //     captureTexture = LoadTextureFromImage(MatToImage(image));

    DrawProgressBarScreen("iniciando hilo de cinemática...", 90, font);
    std::thread t1(calculateKinematics,std::ref(x),std::ref(y),std::ref(z),std::ref(octoKin));

    DrawProgressBarScreen("listo", 100, font);

    //--------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------
    while (!EXIT)
    {
        if(IsKeyDown(KEY_ESCAPE))
		{
			EXIT = true;
		}

        UpdateCamera(&camera);      // Actualizar camara 3D

        shaderResolution[0] += GetMouseWheelMove()*10.0f;
        shaderResolution[1] = shaderResolution[0];
        // if (outlineSize < 1.0f) outlineSize = 1.0f;

        SetShaderValue(shader, resolutionLoc, shaderResolution, SHADER_UNIFORM_VEC2);

        // if(CAPTURE_READY)
        // {
        //     UpdateTexture(captureTexture,MatToImage(image).data);
        //     CAPTURE_READY = false;
        // }

        if(STARTING_ANIMATION)
        {
            animationStep++;
            if(animationState == 1)
            {
                camera.fovy = camera.fovy-0.001f*animationStep/GetFrameTime();
                if(camera.fovy <= CAMERA_FOV) STARTING_ANIMATION = false;
                //if(animationStep > 100) STARTING_ANIMATION = false;
            }
        }else camera.fovy = CAMERA_FOV;

        // LEER INPUTS -----------------------------------------------------
        inputRegData = readRegister(PIN_REG_OUT, PIN_REG_PL, PIN_REG_CP);
        axis_state_X0 = readBit(inputRegData, REG_BIT_X0);
        axis_state_X1 = readBit(inputRegData, REG_BIT_X1);
        axis_state_Y0 = readBit(inputRegData, REG_BIT_Y0);
        axis_state_Y1 = readBit(inputRegData, REG_BIT_Y1);
        button_state_R1 = readBit(inputRegData, REG_BIT_R1);
        button_state_R2 = readBit(inputRegData, REG_BIT_R2);
        button_state_R3 = readBit(inputRegData, REG_BIT_R3);

        axis_state_X = 0;
        axis_state_Y = 0;

        // centro x0=1 x1=0 / y0=1 y1=0
        if(axis_state_X0 == 0 && axis_state_X1 == 0) //izq
        {
            axis_state_X = -1;
        }
        if(axis_state_X0 == 1 && axis_state_X1 == 1) //der
        {
            axis_state_X = 1;
        }
        if(axis_state_Y0 == 1 && axis_state_Y1 == 1) //abajo
        {
            axis_state_Y = -1;
        }
        if(axis_state_Y0 == 0 && axis_state_Y1 == 0) //arriba
        {
            axis_state_Y = 1;
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

        if(IsKeyPressed(KEY_DOWN) || (axis_state_Y == -1 && axis_state_Y != last_axis_state_Y ))
        {
            if(highlightedMenu < std::prev(menu[currentMenuID].options.end()))
                highlightedMenu++;
            else
                highlightedMenu = menu[currentMenuID].options.begin();
        }
        if(IsKeyPressed(KEY_UP) || (axis_state_Y == 1 && axis_state_Y != last_axis_state_Y ))
        {
            if(highlightedMenu > menu[currentMenuID].options.begin())
                highlightedMenu--;
            else
                highlightedMenu = std::prev(menu[currentMenuID].options.end());
        }
        if(IsKeyPressed(KEY_ENTER) || (!button_state_R3 && button_state_R3 != last_button_state_R3 ))
        {
            if((*highlightedMenu) == "Atrás")
            {
                currentMenuID = menu[currentMenuID].parent->id;
                highlightedMenu = menu[currentMenuID].options.begin();
            }else if((*highlightedMenu) == "Debug")
            {
                SHOW_DEBUG_DATA = !SHOW_DEBUG_DATA;   
            }else{
                for(i = 0; i < MENUS_AMOUNT; i++)
                    if((*highlightedMenu) == menu[i].title) currentMenuID = i;
                highlightedMenu = menu[currentMenuID].options.begin();
            }
        }

        if(IsKeyPressed(KEY_F1))
        {
            SHOW_DEBUG_DATA = !SHOW_DEBUG_DATA;
        }

        if(IsKeyDown(KEY_SPACE))
        {
            auxValue += 1.0f;
            if(auxValue >= 360) auxValue = 0;
        }

        //z = -ROD_LENGTH+sin(GetTime()*0.1f)*20.0f;
        //x = sin(GetTime()*1.0f)*30.0f;
        //y = cos(GetTime()*1.0f)*30.0f;

        if(lastX != octoKin.x || lastY != octoKin.y || lastZ != octoKin.z) // calcula solo si hubo variaciones
        {
            // Cálculos de cinemática
            // octoKin.inverse_kinematics(x, y, z);
            // #if ARCH_ARM
            // octoKin.updateKinematics();
            // #endif

            rod1Pos = (Vector3){arm1Pos.x,static_cast<float>(arm1Pos.y-ARM_LENGTH*sin(-octoKin.b*DEG2RAD)),static_cast<float>(arm1Pos.z-ARM_LENGTH*cos(-octoKin.b*DEG2RAD))};
            rod2Pos = (Vector3){static_cast<float>(arm2Pos.x+ARM_LENGTH*cos(-octoKin.c*DEG2RAD)*cos(30*DEG2RAD)),static_cast<float>(arm2Pos.y-ARM_LENGTH*sin(-octoKin.c*DEG2RAD)),static_cast<float>(arm2Pos.z+ARM_LENGTH*cos(-octoKin.c*DEG2RAD)*sin(30*DEG2RAD))};
            rod3Pos = (Vector3){static_cast<float>(arm3Pos.x-ARM_LENGTH*cos(-octoKin.a*DEG2RAD)*cos(30*DEG2RAD)),static_cast<float>(arm3Pos.y-ARM_LENGTH*sin(-octoKin.a*DEG2RAD)),static_cast<float>(arm3Pos.z+ARM_LENGTH*cos(-octoKin.a*DEG2RAD)*sin(30*DEG2RAD))};

            basePos = (Vector3){static_cast<float>(octoKin.x),static_cast<float>(BAS_POSITION.y+octoKin.z),static_cast<float>(octoKin.y)};

            // rod 1 angles
            arm1Projection = rod1Pos;
            arm1Projection.y = 0;
            arm1Projection = Vector3Normalize(arm1Projection);
            baseJoint1 = Vector3Scale(arm1Projection,EFF_RADIUS);
            baseJoint1 = Vector3Add(basePos,baseJoint1);

            Vector3 rod1Direction = Vector3Subtract(rod1Pos,baseJoint1);

            rod1Theta = atan2(rod1Direction.z,rod1Direction.x)+90*DEG2RAD;
            rod1Phi = Vector3Angle(rod1Direction,(Vector3){0,1.0f,0})+90*DEG2RAD;

            // rod 2 angles
            arm2Projection = rod2Pos;
            arm2Projection.y = 0;
            arm2Projection = Vector3Normalize(arm2Projection);
            baseJoint2 = Vector3Scale(arm2Projection,EFF_RADIUS);
            baseJoint2 = Vector3Add(basePos,baseJoint2);

            Vector3 rod2Direction = Vector3Subtract(rod2Pos,baseJoint2);

            rod2Theta = atan2(rod2Direction.z,rod2Direction.x)-30*DEG2RAD;
            rod2Phi = Vector3Angle(rod2Direction,(Vector3){0,1.0f,0})+90*DEG2RAD;

            // rod 3 angles
            arm3Projection = rod3Pos;
            arm3Projection.y = 0;
            arm3Projection = Vector3Normalize(arm3Projection);
            baseJoint3 = Vector3Scale(arm3Projection,EFF_RADIUS);
            baseJoint3 = Vector3Add(basePos,baseJoint3);

            Vector3 rod3Direction = Vector3Subtract(rod3Pos,baseJoint3);
            auxVector = Vector3Scale(arm3Projection,EFF_RADIUS);
            auxVector.y = baseJoint3.y;

            rod3Theta = atan2(rod3Direction.z,rod3Direction.x)-150*DEG2RAD;
            rod3Phi = Vector3Angle(rod3Direction,(Vector3){0,1.0f,0})+90*DEG2RAD;

            // transformations

            //// arms

            armModel1->transform = arm1InitialMatrix;
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixRotate(arm1Axis,-octoKin.b*DEG2RAD));
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixTranslate(arm1Pos.x,arm1Pos.y,arm1Pos.z));

            armModel2->transform = arm2InitialMatrix;
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate(arm2Axis,-octoKin.c*DEG2RAD));
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixTranslate(arm2Pos.x,arm2Pos.y,arm2Pos.z));

            armModel3->transform = arm3InitialMatrix;
            armModel3->transform = MatrixMultiply(armModel3->transform,MatrixRotate(arm3Axis,-octoKin.a*DEG2RAD));
            armModel3->transform = MatrixMultiply(armModel3->transform,MatrixTranslate(arm3Pos.x,arm3Pos.y,arm3Pos.z));

            //// rods

            rodModel1->transform = rod1InitialMatrix;
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixRotate(arm1Axis,rod1Phi));
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixRotate((Vector3){0,-1,0},rod1Theta));
            rodModel1->transform = MatrixMultiply(rodModel1->transform,MatrixTranslate(rod1Pos.x, rod1Pos.y, rod1Pos.z));

            rodModel2->transform = rod2InitialMatrix;
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixRotate(arm2Axis,rod2Phi));
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixRotate((Vector3){0,-1,0},rod2Theta));
            rodModel2->transform = MatrixMultiply(rodModel2->transform,MatrixTranslate(rod2Pos.x, rod2Pos.y, rod2Pos.z));

            rodModel3->transform = rod3InitialMatrix;
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixRotate(arm3Axis,rod3Phi));
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixRotate((Vector3){0,-1,0},rod3Theta));
            rodModel3->transform = MatrixMultiply(rodModel3->transform,MatrixTranslate(rod3Pos.x, rod3Pos.y, rod3Pos.z));
        }

        lastX = octoKin.x;
        lastY = octoKin.y;
        lastZ = octoKin.z;

        last_axis_state_X = axis_state_X;
        last_axis_state_Y = axis_state_Y;
        last_button_state_R1 = button_state_R1;
        last_button_state_R2 = button_state_R2;
        last_button_state_R3 = button_state_R3;

        //armModel->transform = MatrixRotateXYZ((Vector3){ 0, 0, x });
        //----------------------------------------------------------------------------------
        // Dibuja
        //----------------------------------------------------------------------------------

        BeginTextureMode(renderTextureModel);       // Enable drawing to texture
            ClearBackground((Color){0,0,0,0});  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                DrawModel(*platformModel,Vector3Scale(BAS_POSITION,DRAW_SCALE),DRAW_SCALE,WHITE);
                // DrawModel(*baseModel,Vector3Scale(basePos,DRAW_SCALE),DRAW_SCALE,WHITE);
                DrawModel(*armModel1, Vector3Zero(), DRAW_SCALE, WHITE);
                DrawModel(*armModel2, Vector3Zero(), DRAW_SCALE, WHITE);
                DrawModel(*armModel3, Vector3Zero(), DRAW_SCALE, WHITE);
                DrawModel(*rodModel1, Vector3Zero(), DRAW_SCALE, WHITE);
                DrawModel(*rodModel2, Vector3Zero(), DRAW_SCALE, WHITE);
                DrawModel(*rodModel3, Vector3Zero(), DRAW_SCALE, WHITE);
                //DrawCircle3D(BAS_POSITION,BAS_RADIUS,(Vector3){1.0f,0.0f,0.0f},90.0f,WHITE);
            EndMode3D();                // End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode();               // End drawing to texture (now we have a texture available for next passes)
        BeginTextureMode(renderTextureBackground);
        ClearBackground(COLOR_BG);  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                DrawGrid((int)BAS_RADIUS/1.5f,BAS_RADIUS/2.0f);
                DrawModel(*baseModel,Vector3Scale(basePos,DRAW_SCALE),DRAW_SCALE,COLOR_HL);
            EndMode3D();
        EndTextureMode();

        BeginDrawing();
            ClearBackground(COLOR_BG);
            
            Vector2 viewSize = {(float)renderTextureModel.texture.width/3, (float)renderTextureModel.texture.height/2};
            Rectangle viewRectangle = {(float)renderTextureModel.texture.width/2-viewSize.x/2, (float)renderTextureModel.texture.height/2-viewSize.y/2, viewSize.x, -viewSize.y};
            Vector2 viewPos = { screenWidth-viewSize.x-MARGIN, (int)(MARGIN*2+fontSize)};
            DrawTextureRec(renderTextureBackground.texture, viewRectangle, viewPos, WHITE);
            BeginShaderMode(shader);
                // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                DrawTextureRec(renderTextureModel.texture, viewRectangle, viewPos, WHITE);
            EndShaderMode();
            Rectangle viewBorderRectangle = {viewPos.x, viewPos.y, viewSize.x, viewSize.y};
            DrawRectangleLinesEx(viewBorderRectangle,BORDER_THICKNESS,COLOR_FG);

            // Vector2 captureViewPos = { viewPos.x, viewPos.y+viewSize.y+MARGIN};
            // DrawTextureEx(captureTexture, captureViewPos, 0, viewSize.x/captureTexture.width,WHITE);
            // Rectangle captureViewRectangle = {captureViewPos.x, captureViewPos.y, captureTexture.width*(viewSize.x/captureTexture.width), captureTexture.height*(viewSize.x/captureTexture.width)};
            // DrawRectangleLinesEx(captureViewRectangle,BORDER_THICKNESS,COLOR_FG);

            i = 2;
            DrawTextEx(font,menu[currentMenuID].title,(Vector2){MARGIN,MARGIN},fontSize,1,COLOR_FG);
            for (std::vector<const char*>::iterator it = menu[currentMenuID].options.begin(); it != menu[currentMenuID].options.end(); it++)
            {
                Vector2 optionPos = {MARGIN, MARGIN*i+fontSize*(i-1)};
                Vector2 optionSize = {viewPos.x-MARGIN*2, fontSize};
                Rectangle optionRectangle = {optionPos.x, optionPos.y, optionSize.x, optionSize.y};
                DrawRectangleLinesEx(optionRectangle,BORDER_THICKNESS,COLOR_FG);
                sprintf(c," %s",(*it));
                if(it == highlightedMenu)
                {
                    DrawRectangleRec(optionRectangle, COLOR_FG);
                    DrawTextEx(font,c,optionPos,fontSize,1,COLOR_BG);
                    if((*it) != "Atrás")
                        DrawTextEx(font,">",Vector2Add(optionPos,(Vector2){optionSize.x-MARGIN-fontSize/2,0}),fontSize,1,COLOR_BG);
                    else
                        DrawTextEx(font,"<",Vector2Add(optionPos,(Vector2){optionSize.x-MARGIN-fontSize/2,0}),fontSize,1,COLOR_BG);
                }else
                {
                    DrawTextEx(font,c,optionPos,fontSize,1,COLOR_FG);
                    if((*it) != "Atrás")
                        DrawTextEx(font,">",Vector2Add(optionPos,(Vector2){optionSize.x-MARGIN-fontSize/2,0}),fontSize,1,COLOR_FG);
                    else
                        DrawTextEx(font,"<",Vector2Add(optionPos,(Vector2){optionSize.x-MARGIN-fontSize/2,0}),fontSize,1,COLOR_FG);
                }
                i++;
            }

            if(SHOW_DEBUG_DATA)
            {
                sprintf(c,"FPS %d",GetFPS());
                DrawTextEx(font,c,(Vector2){screenWidth-MARGIN-3*fontSize,MARGIN},fontSize,1,ORANGE);
                sprintf(c, "X0\tX1\tY0\tY1\tR1\tR2\tR3");
                DrawTextEx(font,c,(Vector2){MARGIN,screenHeight-MARGIN-fontSize*2},fontSize,1,COLOR_FG);
                sprintf(c, " %i\t %i\t %i\t %i\t %i\t %i\t %i",axis_state_X0,axis_state_X1,axis_state_Y0,axis_state_Y1,button_state_R1,button_state_R2,button_state_R3);
                DrawTextEx(font,c,(Vector2){MARGIN,screenHeight-MARGIN-fontSize},fontSize,1,COLOR_FG);
                // sprintf(c,"STARTING_ANIMATION %i",STARTING_ANIMATION);
                // DrawTextEx(font,c,(Vector2){MARGIN,MARGIN*2+fontSize},fontSize,1,COLOR_FG);
                // sprintf(c,"SHADER_RESOLUTION %.2f",shaderResolution[0]);
                // DrawTextEx(font,c,(Vector2){MARGIN,screenHeight-MARGIN-fontSize},fontSize,1,COLOR_FG);
                // sprintf(c,"A:\t%+03.2f\tB:\t%+03.2f\tC:\t%+03.2f",octoKin.a, octoKin.b, octoKin.c);
                // DrawTextEx(font,c,(Vector2){MARGIN,MARGIN*4+fontSize*3},fontSize,1,COLOR_FG);
                // sprintf(c,"X:\t%+03.2f\tY:\t%+03.2f\tZ:\t%+03.2f",x, y, z);
                // DrawTextEx(font,c,(Vector2){MARGIN,MARGIN*5+fontSize*4},fontSize,1,COLOR_FG);
            }
            if(STARTING_ANIMATION && animationState == 0)
            {
                /*
                int startingCoverX = 0.1f*animationStep/GetFrameTime();
                DrawRectangle(startingCoverX,0,screenWidth,screenHeight,COLOR_BG);
                if(startingCoverX >= screenWidth)
                {
                    animationState = 1;
                    animationStep = 0;
                }
                */
                int alpha = 255;
                if(GetFrameTime() > 0)
                    alpha = 255-0.2f*animationStep/GetFrameTime();
                if(alpha <= 0)
                {
                    animationState = 1;
                    animationStep = 0;
                }else{
                    DrawRectangle(0,0,screenWidth,screenHeight,(Color){COLOR_BG.r,COLOR_BG.g,COLOR_BG.b,alpha});
                }
            }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    CloseWindow();
    //--------------------------------------------------------------------------------------

    //cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

    UnloadModel(*platformModel);
    UnloadModel(*baseModel);
    UnloadModel(*armModel1);
    UnloadModel(*rodModel1);

    UnloadShader(shader);

    fprintf(stdout, "[INFO] THREAD: Joining kinematics thread...");
    fflush(stdout);
	while(!t1.joinable()){}
	t1.join();
	fprintf(stdout, " Done.\n");

    return 0;
}
