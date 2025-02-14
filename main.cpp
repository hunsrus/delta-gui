#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso

//#define PLATFORM_DESKTOP
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include "rlgl.h"
// #include "DeltaKinematics.h"
#include "OctoKinematics.h"

// deteccion de imagenes
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <iomanip> // para setprecision()
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <list>
#include <dirent.h>
#include <cstring>

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
#define LIM_Z -230.0f

#define TRANS_MULTIPLIER 3
static unsigned int STEPS_NUM = 16;
#define NUMERIC_PRECISION 4 // cantidad de decimales

// pinout definitions
#define PIN_DIR1 12
#define PIN_STEP1 16
#define PIN_DIR2 21
#define PIN_STEP2 20
#define PIN_DIR3 7
#define PIN_STEP3 1
#define PIN_DIR4 0
#define PIN_STEP4 22

#define PIN_MS1 27
#define PIN_MS2 17
#define PIN_MS3 4

#define PIN_REG_PL 3
#define PIN_REG_CP 18
#define PIN_REG_OUT 23

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

#define PIN_BOMBA 19
#define PIN_ENABLE 26

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
static float FONT_PIXELS = 24*DISPLAY_HEIGHT/240;
static unsigned int OPTIONS_PER_WINDOW = 6;
#define BUTTON_SIZE (DISPLAY_HEIGHT-MARGIN*3-MARGIN*(OPTIONS_PER_WINDOW-1)-FONT_PIXELS)/OPTIONS_PER_WINDOW
static float SCREEN_DIVISION_RATIO = 1.0f/3.0f;

#define CAMERA_FOV 65
#define DRAW_SCALE 0.5

static bool SHOW_DEBUG_DATA = false;
static bool STARTING_ANIMATION = false;
static bool SHOW_3D_VIEW = true;
static bool SHOW_MENU_INFO = false;
static bool SHOW_FIELD_VALUES = false;

#define MCGREEN CLITERAL(Color){150,182,171,255}   // Verde Colin McRae
#define POP_BLUE CLITERAL(Color){99,177,188,255}
#define GRIS CLITERAL(Color){34,34,34,255}
static Color COLOR_BG = {34,34,34,255};
static Color COLOR_FG = {238,238,238,255};
static Color COLOR_HL = ORANGE;

Font font;

static bool EXIT = false;
static bool STATUS_MOTOR_ENABLED = true;
static bool STATUS_VACUUM_PUMP = false;
static bool MODE_MANUAL = false;
static bool JOB_RUNNING = false;
static bool JOB_SHOULD_STOP = false;
static bool EXECUTING_INSTRUCTION = false;

std::vector<std::string> CURRENT_JOB;
std::list<std::string> MANUAL_QUEUE;

static std::string PATH_FILES = "../tests/";
static double STEP_SIZE = 0.002;
static float MANUAL_INCREMENT = 0.5f;
static Vector3 POS_PCB_REF1 = Vector3Zero();
static Vector3 POS_PCB_REF2 = Vector3Zero();

//----------------------------------------------------------------------------------
// INTERFAZ
//----------------------------------------------------------------------------------

// opciones de menú
typedef struct Option{
    unsigned int id;
    std::string text;
    std::string value = "";
}Option;

// estructura del menú
typedef struct Menu{
    const char* title;
    Menu* parent = NULL;
    std::vector<Option> options;
}Menu;

// estructura de tema gráfico
typedef struct Theme
{
    Color background = MCGREEN;
    Color foreground = WHITE;
    Color accent = RED;
}Theme;

Menu* CURRENT_MENU;
std::vector<Option>::iterator HIGHLIGHTED_OPTION;

void goBackOneMenu(void);
void DrawProgressBarScreen(const char* text, int progress, Font font);
void DrawProgressBarIndicator(const char* text, int progress, Font font);

//----------------------------------------------------------------------------------
// DETECCIÓN DE IMÁGENES
//----------------------------------------------------------------------------------
/*
static cv::Mat image;
static bool CAMERA_AVAILABLE = true;
static bool CAPTURE_READY = false;

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
*/

//----------------------------------------------------------------------------------
// MANEJO DE ARCHIVOS
//----------------------------------------------------------------------------------

// estructura para almacenar la información de cada componente
struct Componente {
    std::string reference;
    std::string value;
    std::string package;
    double posx;
    double posy;
    double rotation;
};

typedef struct Feeder
{
    int id;
    Vector3 push;
    Vector3 approach;
    Vector3 pick;
    std::string component;
}Feeder;

static std::vector<Feeder> feeders;
static std::vector<std::string> CURRENT_COMPONENTS;

std::vector<Componente> parsearArchivo(const std::string& nombreArchivo);
int executeInstruction(std::string instruction, OctoKinematics &octoKin);
std::vector<std::string> generateJob(std::vector<Componente> componentes);
std::vector<std::string> listarArchivos(const std::string& rutaCarpeta);

//----------------------------------------------------------------------------------
// UTILIDADES
//----------------------------------------------------------------------------------
float mapear(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

char readRegister(int pin_data, int pin_pl, int pin_cp);
int readBit(char data, int bit);

int writeConfigFile(const std::string& config_file_path);
int configFileParser(std::string config_file_path);

int calculateKinematics(double &x,double &y,double &z, OctoKinematics &octoKin)
{
    float low_z = LIM_Z;//-294
    float high_z = LIM_Z+10;

    while(!EXIT)
    {
        if(JOB_RUNNING)
        {
            for (const auto& instruction : CURRENT_JOB) 
            {
                EXECUTING_INSTRUCTION = true;
                executeInstruction(instruction,octoKin);
                EXECUTING_INSTRUCTION = false;
                if(JOB_SHOULD_STOP || EXIT)
                {
                    JOB_RUNNING = false;
                    JOB_SHOULD_STOP = false;
                    break;
                }
            }
        }else if(MODE_MANUAL)
        {
            STEP_SIZE = 0.01f;
            JOB_SHOULD_STOP = false;
            if(!MANUAL_QUEUE.empty())
            {
                EXECUTING_INSTRUCTION = true;
                executeInstruction(MANUAL_QUEUE.front(),octoKin);
                MANUAL_QUEUE.pop_front();
                EXECUTING_INSTRUCTION = false;
            }
        }
    }

    return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = DISPLAY_WIDTH;
    const int screenHeight = DISPLAY_HEIGHT;

    //SetConfigFlags(FLAG_FULLSCREEN_MODE);
    //SetConfigFlags(FLAG_MSAA_4X_HINT);
    //SetConfigFlags(FLAG_WINDOW_UNDECORATED);
    InitWindow(screenWidth, screenHeight, "delta gui test");

    font = LoadFontEx("resources/fonts/JetBrainsMono/JetBrainsMono-Bold.ttf", FONT_PIXELS, 0, 250);
    float fontSize = font.baseSize;//DISPLAY_HEIGHT/20;

    HideCursor();
    SetTargetFPS(TARGET_FPS);

    DrawProgressBarScreen("inicializando variables...", 10, font);
    int i;
    char c[255];
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

    std::vector<std::string> archivos;
    std::string currentPosFile = "";
    int selectedFeederID;

    if(!configFileParser("../config.conf"))
        std::cout << "Lectura de archivo de configuración exitosa" << std::endl;
    else
        std::cout << "Error en lectura de archivo de configuración" << std::endl;

    int THEMES_COUNT = 9;
    Theme themes[THEMES_COUNT] = {{MCGREEN, WHITE, (Color){92,65,93,255}},
    //{(Color){34,34,34,255},(Color){99,177,188,255}, RED},
    // {(Color){34,34,34,255},(Color){238,238,238,255},(Color){48,128,203,255}},
    {(Color){34,34,34,255},(Color){238,238,238,255},ORANGE},
    {(Color){46,52,64,255},(Color){229,233,240,255},(Color){221,115,115,255}},
    {(Color){3,57,75,255},(Color){187,231,250,255},(Color){181,137,0,255}},
    {(Color){78,70,67,255},(Color){254,167,0,255},(Color){244,43,3,255}},
    {(Color){82,76,70,255},(Color){234,233,233,255},(Color){214,64,69,255}},
    {(Color){43,43,43,255},(Color){155,155,155,255},(Color){182,70,95,255}},
    {BLACK, WHITE, (Color){240,58,71,255}},
    {WHITE, BLACK, (Color){240,58,71,255}}};

    int CURRENT_THEME = 1;
    COLOR_BG = themes[CURRENT_THEME].background;
    COLOR_FG = themes[CURRENT_THEME].foreground;
    COLOR_HL = themes[CURRENT_THEME].accent;

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

    std::vector<Menu*> menus;

    Menu *auxMenu;
    auxMenu = new Menu();
    auxMenu->title = "Menú principal";
    auxMenu->parent = NULL;
    // auxMenu->options.push_back((Option){0,"Debug"});
    auxMenu->options.push_back((Option){1,"Trabajos"});
    auxMenu->options.push_back((Option){2,"Calibrar"});
    auxMenu->options.push_back((Option){3,"Control"});
    auxMenu->options.push_back((Option){4,"Interfaz"});
    auxMenu->options.push_back((Option){5,"Salir"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Trabajos";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"Iniciar rutina"});
    auxMenu->options.push_back((Option){2,"Cargar archivo"});
    auxMenu->options.push_back((Option){3,"Asignar feeders"});
    auxMenu->options.push_back((Option){4,"Guardar rutina"});
    auxMenu->options.push_back((Option){5,"Abrir rutina"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Asignar feeders";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    for( const auto& feeder : feeders)
    {
        auxMenu->options.push_back((Option){feeder.id,"Feeder "+std::to_string(feeder.id), feeder.component});
    }
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Calibrar";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"Zona PCB"});
    auxMenu->options.push_back((Option){2,"Feeders"});
    auxMenu->options.push_back((Option){3,"Guardar"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Zona PCB";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"PCB Ref1"});
    auxMenu->options.push_back((Option){2,"PCB Ref2"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Feeders";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    for( const auto& feeder : feeders)
    {
        auxMenu->options.push_back((Option){feeder.id,"Feeder "+std::to_string(feeder.id)});
    }
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Control";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"Girar"});
    auxMenu->options.push_back((Option){2,"Succión"});
    auxMenu->options.push_back((Option){3,"Mover"});
    auxMenu->options.push_back((Option){4,"Paso", std::to_string(MANUAL_INCREMENT)});
    auxMenu->options.push_back((Option){5,"Deshabilitar"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Interfaz";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"Letra",std::to_string((int)fontSize)});
    auxMenu->options.push_back((Option){3,"Botones",std::to_string(OPTIONS_PER_WINDOW)});
    auxMenu->options.push_back((Option){4,"Tema",std::to_string(CURRENT_THEME)});
    auxMenu->options.push_back((Option){5,"División",std::to_string(SCREEN_DIVISION_RATIO)});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Salir";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"Apagar"});
    auxMenu->options.push_back((Option){2,"Reiniciar"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Cargar archivo";
    auxMenu->parent = menus.at(1);
    auxMenu->options.push_back((Option){0,"Atrás"});
    menus.push_back(auxMenu);
    auxMenu = new Menu();
    auxMenu->title = "Asignar feeders";
    auxMenu->parent = menus.at(1);
    auxMenu->options.push_back((Option){0,"Atrás"});
    menus.push_back(auxMenu);

    auxMenu = new Menu();
    auxMenu->title = "Feeder config";
    auxMenu->parent = menus.at(0);
    auxMenu->options.push_back((Option){0,"Atrás"});
    auxMenu->options.push_back((Option){1,"Push"});
    auxMenu->options.push_back((Option){2,"Approach"});
    auxMenu->options.push_back((Option){3,"Pick"});
    menus.push_back(auxMenu);
    
    CURRENT_MENU = menus.at(0);
    HIGHLIGHTED_OPTION = CURRENT_MENU->options.begin();

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

    Vector3 motor1Pos = Vector3Scale(arm1Pos,0.8f);
    Vector3 motor2Pos = Vector3Scale(arm2Pos,0.8f);
    Vector3 motor3Pos = Vector3Scale(arm3Pos,0.8f);
    motor1Pos.y = arm1Pos.y+16.0f;
    motor2Pos.y = arm2Pos.y+16.0f;
    motor3Pos.y = arm3Pos.y+16.0f;
    Model* motorModel1 = new Model(LoadModelFromMesh(GenMeshCube(32.0f,32.0f,32.0f)));
    Model* motorModel2 = new Model(LoadModelFromMesh(GenMeshCube(32.0f,32.0f,32.0f)));
    Model* motorModel3 = new Model(LoadModelFromMesh(GenMeshCube(32.0f,32.0f,32.0f)));
    motorModel1->transform = arm1InitialMatrix;
    motorModel1->transform = MatrixMultiply(motorModel1->transform,MatrixTranslate(motor1Pos.x,motor1Pos.y,motor1Pos.z));
    motorModel2->transform = arm2InitialMatrix;
    motorModel2->transform = MatrixMultiply(motorModel2->transform,MatrixTranslate(motor2Pos.x,motor2Pos.y,motor2Pos.z));
    motorModel3->transform = arm3InitialMatrix;
    motorModel3->transform = MatrixMultiply(motorModel3->transform,MatrixTranslate(motor3Pos.x,motor3Pos.y,motor3Pos.z));

    DrawProgressBarScreen("definiendo vista 3D...", 50, font);
    // Define the camera to look into our 3d world
    //Camera camera = { {-20.0f, 12.0f, 0.0f}, { 0.0f, 4.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    Camera camera = { {0.0f, (ARM_LENGTH+ROD_LENGTH)*1.0f, BAS_RADIUS*3.0f}, {BAS_POSITION.x, BAS_POSITION.y/3.0f, BAS_POSITION.z}, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 179.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DrawProgressBarScreen("cargando shaders...", 60, font);
    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D renderTextureModel = LoadRenderTexture(screenWidth, screenHeight);
    RenderTexture2D renderTextureBackground = LoadRenderTexture(screenWidth, screenHeight);
    RenderTexture2D renderTextureForeground = LoadRenderTexture(screenWidth, screenHeight);
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

    Vector2 viewSize = {DISPLAY_WIDTH*SCREEN_DIVISION_RATIO, (float)renderTextureModel.texture.height/2};
    Rectangle viewRectangle = {(float)renderTextureModel.texture.width/2-viewSize.x/2, (float)renderTextureModel.texture.height/2-viewSize.y/2, viewSize.x, -viewSize.y};
    Vector2 viewPos = { screenWidth-viewSize.x-MARGIN, (int)(MARGIN*2+fontSize)};
    Vector2 dataViewPos = { viewPos.x, viewPos.y+viewSize.y+MARGIN};
    Rectangle dataViewRectangle = {dataViewPos.x, dataViewPos.y, viewSize.x, DISPLAY_HEIGHT-viewSize.y-fontSize-MARGIN*4};
    float dataViewFontSize = (dataViewRectangle.height-MARGIN*4)/3;
    Font dataViewFont = LoadFontEx("resources/fonts/JetBrainsMono/JetBrainsMono-Bold.ttf", dataViewFontSize, 0, 250);
    

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
        gpioSetMode(PIN_ENABLE,PI_OUTPUT);

        // config robot pins
        octoKin.set_pin_step_ctrl(PIN_MS1,PIN_MS2,PIN_MS3);
        octoKin.set_pin_motor_1(PIN_STEP1, PIN_DIR1);
        octoKin.set_pin_motor_2(PIN_STEP2, PIN_DIR2);
        octoKin.set_pin_motor_3(PIN_STEP3, PIN_DIR3);
        octoKin.set_pin_limit_sw(PIN_FC_M1, PIN_FC_M2, PIN_FC_M3);
        octoKin.set_axis_direction(1);
        octoKin.set_step_precision(STEPS_NUM);
        octoKin.set_transmission_ratio(TRANS_MULTIPLIER);
        octoKin.set_pulse_width(1);
        
        // turn suction off
        gpioWrite(PIN_BOMBA,0);
        // enable motors
        gpioWrite(PIN_ENABLE,1);
        STATUS_MOTOR_ENABLED = 0;

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
    // octoKin.inverse_kinematics(x, y, z);
    // octoKin.updateKinematics();
    octoKin.linear_move(x, y, z, 0.1f, 1000);

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

        if(MODE_MANUAL)
        {
            if(!EXECUTING_INSTRUCTION)
            {
                #if ARCH_ARM
                    if(axis_state_X != 0 || axis_state_Y != 0)
                #else
                    if(IsKeyDown(KEY_A) || IsKeyDown(KEY_D) || IsKeyDown(KEY_S) || IsKeyDown(KEY_W))
                #endif
                {
                    std::string instruction = "L";
                    
                    if(IsKeyDown(KEY_A) || axis_state_X == -1)
                    {
                        sprintf(c, "X%.4f",octoKin.x-MANUAL_INCREMENT);
                        instruction += c;
                    }
                    if(IsKeyDown(KEY_D) || axis_state_X == 1)
                    {
                        sprintf(c, "X%.4f",octoKin.x+MANUAL_INCREMENT);
                        instruction += c;
                    }
                    if(IsKeyDown(KEY_S) || axis_state_Y == -1)
                    {
                        sprintf(c, "Y%.4f",octoKin.y-MANUAL_INCREMENT);
                        instruction += c;
                    }
                    if(IsKeyDown(KEY_W) || axis_state_Y == 1)
                    {
                        sprintf(c, "Y%.4f",octoKin.y+MANUAL_INCREMENT);
                        instruction += c;
                    }
                    MANUAL_QUEUE.push_back(instruction);
                }
                if(IsKeyDown(KEY_LEFT_SHIFT) || !button_state_R1)
                {
                    sprintf(c, "LZ%.4f",octoKin.z+MANUAL_INCREMENT);
                    MANUAL_QUEUE.push_back(c);
                }
                if(IsKeyDown(KEY_LEFT_CONTROL) || !button_state_R2)
                {
                    sprintf(c, "LZ%.4f",octoKin.z-MANUAL_INCREMENT);
                    MANUAL_QUEUE.push_back(c);
                }
            }
        }else
        {
            if(IsKeyPressed(KEY_DOWN) || (axis_state_Y == -1 && axis_state_Y != last_axis_state_Y ))
            {
                if(HIGHLIGHTED_OPTION < std::prev(CURRENT_MENU->options.end()))
                    HIGHLIGHTED_OPTION++;
                else
                    HIGHLIGHTED_OPTION = CURRENT_MENU->options.begin();
            }
            if(IsKeyPressed(KEY_UP) || (axis_state_Y == 1 && axis_state_Y != last_axis_state_Y ))
            {
                if(HIGHLIGHTED_OPTION > CURRENT_MENU->options.begin())
                    HIGHLIGHTED_OPTION--;
                else
                    HIGHLIGHTED_OPTION = std::prev(CURRENT_MENU->options.end());
            }
        }
        

        if(IsKeyPressed(KEY_LEFT) || (axis_state_X == -1 && axis_state_X != last_axis_state_X ))
        {
            if(HIGHLIGHTED_OPTION->text == "Letra")
            {
                if(FONT_PIXELS > 16)
                {
                    FONT_PIXELS -= 1;
                    UnloadFont(font);
                    font = LoadFontEx("resources/fonts/JetBrainsMono/JetBrainsMono-Bold.ttf", FONT_PIXELS, 0, 250);
                    fontSize = font.baseSize;//DISPLAY_HEIGHT/20;
                    HIGHLIGHTED_OPTION->value = std::to_string((int)fontSize); 
                }
            }else if(HIGHLIGHTED_OPTION->text == "Botones")
            {
                if(OPTIONS_PER_WINDOW > 4)
                {
                    OPTIONS_PER_WINDOW -= 1;
                    HIGHLIGHTED_OPTION->value = std::to_string(OPTIONS_PER_WINDOW);
                }
            }else if(HIGHLIGHTED_OPTION->text == "Tema")
            {
                if (CURRENT_THEME > 0) CURRENT_THEME--;
                else CURRENT_THEME = THEMES_COUNT-1;
                
                COLOR_BG = themes[CURRENT_THEME].background;
                COLOR_FG = themes[CURRENT_THEME].foreground;
                COLOR_HL = themes[CURRENT_THEME].accent;

                normalizedFGColor = ColorNormalize(COLOR_FG);
                normalizedBGColor = ColorNormalize(COLOR_BG);
                edgeColor[0] = normalizedFGColor.x;
                edgeColor[1] = normalizedFGColor.y;
                edgeColor[2] = normalizedFGColor.z;
                backgroundColor[0] = normalizedBGColor.x;
                backgroundColor[1] = normalizedBGColor.y;
                backgroundColor[2] = normalizedBGColor.z;
                SetShaderValue(shader, edgeColorLoc, edgeColor, SHADER_UNIFORM_VEC3);
                SetShaderValue(shader, backgroundColorLoc, backgroundColor, SHADER_UNIFORM_VEC3);
                SetShaderValue(shader, resolutionLoc, shaderResolution, SHADER_UNIFORM_VEC2);
                
                HIGHLIGHTED_OPTION->value = std::to_string(CURRENT_THEME);
            }else if(HIGHLIGHTED_OPTION->text == "División")
            {
                SCREEN_DIVISION_RATIO -= 0.1;
                HIGHLIGHTED_OPTION->value = std::to_string(SCREEN_DIVISION_RATIO);
            }else if(HIGHLIGHTED_OPTION->text == "Paso")
            {
                if(MANUAL_INCREMENT > 0.2f)
                {
                    MANUAL_INCREMENT -= 0.2f;
                    HIGHLIGHTED_OPTION->value = std::to_string(MANUAL_INCREMENT);
                }
            }else if(CURRENT_MENU->title == "Asignar feeders")
            {
                if(HIGHLIGHTED_OPTION->value.empty())
                {
                    HIGHLIGHTED_OPTION->value = CURRENT_COMPONENTS.back();
                }else
                {
                    for (std::vector<std::string>::iterator it = CURRENT_COMPONENTS.begin(); it <= CURRENT_COMPONENTS.end(); it++)
                    {
                        if(HIGHLIGHTED_OPTION->value == (*it))
                        {
                            if(it > CURRENT_COMPONENTS.begin())
                                HIGHLIGHTED_OPTION->value = *(std::prev(it));
                            else
                                HIGHLIGHTED_OPTION->value = CURRENT_COMPONENTS.back();
                            break;
                        }
                    }
                }
                for(auto& feeder : feeders)
                {
                    if(feeder.id == HIGHLIGHTED_OPTION->id)
                        feeder.component = HIGHLIGHTED_OPTION->value;
                }
            }
        }
        if(IsKeyPressed(KEY_RIGHT) || (axis_state_X == 1 && axis_state_X != last_axis_state_X ))
        {
            if(HIGHLIGHTED_OPTION->text == "Letra")
            {
                if(FONT_PIXELS < 42)
                {
                    FONT_PIXELS += 1;
                    UnloadFont(font);
                    font = LoadFontEx("resources/fonts/JetBrainsMono/JetBrainsMono-Bold.ttf", FONT_PIXELS, 0, 250);
                    fontSize = font.baseSize;//DISPLAY_HEIGHT/20;
                    HIGHLIGHTED_OPTION->value = std::to_string((int)fontSize);
                }
            }else if(HIGHLIGHTED_OPTION->text == "Botones")
            {
                if(OPTIONS_PER_WINDOW < 8)
                {
                    OPTIONS_PER_WINDOW += 1;
                    HIGHLIGHTED_OPTION->value = std::to_string(OPTIONS_PER_WINDOW);
                }
            }else if(HIGHLIGHTED_OPTION->text == "Tema")
            {
                if (CURRENT_THEME < THEMES_COUNT-1) CURRENT_THEME++;
                else CURRENT_THEME = 0;
                
                COLOR_BG = themes[CURRENT_THEME].background;
                COLOR_FG = themes[CURRENT_THEME].foreground;
                COLOR_HL = themes[CURRENT_THEME].accent;

                normalizedFGColor = ColorNormalize(COLOR_FG);
                normalizedBGColor = ColorNormalize(COLOR_BG);
                edgeColor[0] = normalizedFGColor.x;
                edgeColor[1] = normalizedFGColor.y;
                edgeColor[2] = normalizedFGColor.z;
                backgroundColor[0] = normalizedBGColor.x;
                backgroundColor[1] = normalizedBGColor.y;
                backgroundColor[2] = normalizedBGColor.z;
                SetShaderValue(shader, edgeColorLoc, edgeColor, SHADER_UNIFORM_VEC3);
                SetShaderValue(shader, backgroundColorLoc, backgroundColor, SHADER_UNIFORM_VEC3);
                SetShaderValue(shader, resolutionLoc, shaderResolution, SHADER_UNIFORM_VEC2);
                
                HIGHLIGHTED_OPTION->value = std::to_string(CURRENT_THEME);
            }else if(HIGHLIGHTED_OPTION->text == "División")
            {
                SCREEN_DIVISION_RATIO += 0.1;
                HIGHLIGHTED_OPTION->value = std::to_string(SCREEN_DIVISION_RATIO);
            }else if(HIGHLIGHTED_OPTION->text == "Paso")
            {
                if(MANUAL_INCREMENT < 5.0f)
                {
                    MANUAL_INCREMENT += 0.2f;
                    HIGHLIGHTED_OPTION->value = std::to_string(MANUAL_INCREMENT);
                }
            }else if(CURRENT_MENU->title == "Asignar feeders")
            {
                if(HIGHLIGHTED_OPTION->value.empty())
                {
                    HIGHLIGHTED_OPTION->value = CURRENT_COMPONENTS.front();
                }else
                {
                    for (std::vector<std::string>::iterator it = CURRENT_COMPONENTS.begin(); it <= CURRENT_COMPONENTS.end(); it++)
                    {
                        if(HIGHLIGHTED_OPTION->value == (*it))
                        {
                            if(it < std::prev(CURRENT_COMPONENTS.end()))
                                HIGHLIGHTED_OPTION->value = *(std::next(it));
                            else
                                HIGHLIGHTED_OPTION->value = CURRENT_COMPONENTS.front();
                            break;
                        }
                    }
                }
                for(auto& feeder : feeders)
                {
                    if(feeder.id == HIGHLIGHTED_OPTION->id)
                        feeder.component = HIGHLIGHTED_OPTION->value;
                }
            }
        }
        
        // ACCIONES AL PRESIONAR ENTER
        if(IsKeyPressed(KEY_ENTER) || (!button_state_R3 && button_state_R3 != last_button_state_R3 ))
        {
            if(HIGHLIGHTED_OPTION->text == "Atrás")
            {
                goBackOneMenu();
            }else if(HIGHLIGHTED_OPTION->text == "Debug")
            {
                SHOW_DEBUG_DATA = !SHOW_DEBUG_DATA;   
            }else if(HIGHLIGHTED_OPTION->text == "Deshabilitar")
            {
                STATUS_MOTOR_ENABLED = !STATUS_MOTOR_ENABLED;
                gpioWrite(PIN_ENABLE,STATUS_MOTOR_ENABLED);
            }else if(HIGHLIGHTED_OPTION->text == "Mover")
            {
                MODE_MANUAL = !MODE_MANUAL;
                JOB_SHOULD_STOP = true;
            }else if(HIGHLIGHTED_OPTION->text == "Succión")
            {
                JOB_SHOULD_STOP = true;
                STATUS_VACUUM_PUMP = !STATUS_VACUUM_PUMP;
                gpioWrite(PIN_BOMBA,STATUS_VACUUM_PUMP);
            }else if(HIGHLIGHTED_OPTION->text == "PCB Ref1")
            {
                if(MODE_MANUAL)
                {
                    POS_PCB_REF1 = (Vector3){octoKin.x, octoKin.y, octoKin.z};
                }
                MODE_MANUAL = !MODE_MANUAL;
            }else if(HIGHLIGHTED_OPTION->text == "PCB Ref2")
            {
                if(MODE_MANUAL)
                {
                    POS_PCB_REF2 = (Vector3){octoKin.x, octoKin.y, octoKin.z};
                }
                MODE_MANUAL = !MODE_MANUAL;
            }else if(HIGHLIGHTED_OPTION->text == "Push")
            {
                if(MODE_MANUAL)
                {
                    for(auto& feeder : feeders)
                    {
                        if(feeder.id == selectedFeederID)
                            feeder.push = (Vector3){octoKin.x, octoKin.y, octoKin.z};
                    }
                }
                MODE_MANUAL = !MODE_MANUAL;
            }else if(HIGHLIGHTED_OPTION->text == "Approach")
            {
                if(MODE_MANUAL)
                {
                    for(auto& feeder : feeders)
                    {
                        if(feeder.id == selectedFeederID)
                            feeder.approach = (Vector3){octoKin.x, octoKin.y, octoKin.z};
                    }
                }
                MODE_MANUAL = !MODE_MANUAL;
            }else if(HIGHLIGHTED_OPTION->text == "Pick")
            {
                if(MODE_MANUAL)
                {
                    for(auto& feeder : feeders)
                    {
                        if(feeder.id == selectedFeederID)
                            feeder.pick = (Vector3){octoKin.x, octoKin.y, octoKin.z};
                    }
                }
                MODE_MANUAL = !MODE_MANUAL;
            }else if(HIGHLIGHTED_OPTION->text == "Guardar")
            {
                if(CURRENT_MENU->title == "Calibrar")
                {
                    writeConfigFile("../config.conf");
                }
            }else if(HIGHLIGHTED_OPTION->text == "Iniciar rutina")
            {
                MODE_MANUAL = false;
                JOB_SHOULD_STOP = false;
                JOB_RUNNING = true;
            }else
            {
                if(CURRENT_MENU->title == "Cargar archivo")
                {
                    currentPosFile = HIGHLIGHTED_OPTION->text;

                    static std::vector<Componente> components = parsearArchivo("../tests/"+currentPosFile);
                    CURRENT_JOB = generateJob(components);

                    goBackOneMenu();
                }else if(CURRENT_MENU->title == "Feeders")
                {
                    selectedFeederID = HIGHLIGHTED_OPTION->id;

                    for (const auto& menu : menus) 
                    {
                        if(menu->title == "Feeder config")
                        {
                            CURRENT_MENU = menu;
                            HIGHLIGHTED_OPTION = CURRENT_MENU->options.begin();
                        }
                    }
                }else
                {
                    for (const auto& menu : menus) 
                    {
                        if(HIGHLIGHTED_OPTION->text == menu->title)
                        {
                            CURRENT_MENU = menu;
                            HIGHLIGHTED_OPTION = CURRENT_MENU->options.begin();
                        }
                    }
                }
            }

            if(CURRENT_MENU->title == "Menú principal")
            {
                SHOW_3D_VIEW = true;
                SHOW_MENU_INFO = false;
                SHOW_FIELD_VALUES = false;
            }else if(CURRENT_MENU->title == "Interfaz")
            {
                SHOW_3D_VIEW = false;
                SHOW_MENU_INFO = false;
                SHOW_FIELD_VALUES = true;
            }else if(CURRENT_MENU->title == "Trabajos")
            {
                SHOW_3D_VIEW = true;
                SHOW_MENU_INFO = false;
                SHOW_FIELD_VALUES = false;
            }else if(CURRENT_MENU->title == "Control")
            {
                SHOW_3D_VIEW = true;
                SHOW_MENU_INFO = false;
                SHOW_FIELD_VALUES = true;
            }else if(CURRENT_MENU->title == "Cargar archivo")
            {
                SHOW_3D_VIEW = false;
                SHOW_MENU_INFO = false;
                SHOW_FIELD_VALUES = false;

                archivos = listarArchivos(PATH_FILES);
                CURRENT_MENU->options.clear();
                CURRENT_MENU->options.push_back((Option){0,"Atrás"});
                unsigned int fileCount = 1;
                for (const auto& archivo : archivos) {
                    CURRENT_MENU->options.push_back((Option){fileCount,archivo.c_str()});
                    fileCount++;
                }
                HIGHLIGHTED_OPTION = CURRENT_MENU->options.begin();
            }else if(CURRENT_MENU->title == "Asignar feeders")
            {
                SHOW_3D_VIEW = false;
                SHOW_MENU_INFO = false;
                SHOW_FIELD_VALUES = true;
            }
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

            // calcula la posición de la articulación entre bicep y antebrazo
            rod1Pos = (Vector3){arm1Pos.x,
                static_cast<float>(arm1Pos.y-ARM_LENGTH*sin(-octoKin.c*DEG2RAD)),
                static_cast<float>(arm1Pos.z-ARM_LENGTH*cos(-octoKin.c*DEG2RAD))};
            rod2Pos = (Vector3){static_cast<float>(arm2Pos.x+ARM_LENGTH*cos(-octoKin.b*DEG2RAD)*cos(30*DEG2RAD)),
                static_cast<float>(arm2Pos.y-ARM_LENGTH*sin(-octoKin.b*DEG2RAD)),
                static_cast<float>(arm2Pos.z+ARM_LENGTH*cos(-octoKin.b*DEG2RAD)*sin(30*DEG2RAD))};
            rod3Pos = (Vector3){static_cast<float>(arm3Pos.x-ARM_LENGTH*cos(-octoKin.a*DEG2RAD)*cos(30*DEG2RAD)),
                static_cast<float>(arm3Pos.y-ARM_LENGTH*sin(-octoKin.a*DEG2RAD)),
                static_cast<float>(arm3Pos.z+ARM_LENGTH*cos(-octoKin.a*DEG2RAD)*sin(30*DEG2RAD))};
            
            // copia la posición del efector basada en el cálculo de cinemática
            basePos = (Vector3){static_cast<float>(octoKin.x),static_cast<float>(BAS_POSITION.y+octoKin.z),static_cast<float>(-octoKin.y)};

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
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixRotate(arm1Axis,-octoKin.c*DEG2RAD));
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixTranslate(arm1Pos.x,arm1Pos.y,arm1Pos.z));

            armModel2->transform = arm2InitialMatrix;
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate(arm2Axis,-octoKin.b*DEG2RAD));
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
                float refArrowsLength = 40.0f;
                float refArrowsRadius = 10.0f;
                Vector3 refArrowsPos = (Vector3){-BAS_RADIUS,0.0f,0.0f};
                DrawCylinderEx((Vector3){refArrowsPos.x,refArrowsPos.y+refArrowsLength,refArrowsPos.z},
                    (Vector3){refArrowsPos.x,refArrowsPos.y+refArrowsLength*2,refArrowsPos.z},
                    refArrowsRadius,
                    0.0f,
                    10,
                    BLUE
                );
                DrawLine3D(refArrowsPos, (Vector3){refArrowsPos.x,refArrowsPos.y+refArrowsLength,refArrowsPos.z},BLUE);

                DrawCylinderEx((Vector3){refArrowsPos.x+refArrowsLength,refArrowsPos.y,refArrowsPos.z},
                    (Vector3){refArrowsPos.x+refArrowsLength*2,refArrowsPos.y,refArrowsPos.z},
                    refArrowsRadius,
                    0.0f,
                    10,
                    RED
                );
                DrawLine3D(refArrowsPos, (Vector3){refArrowsPos.x+refArrowsLength,refArrowsPos.y,refArrowsPos.z},RED);

                DrawCylinderEx((Vector3){refArrowsPos.x,refArrowsPos.y,refArrowsPos.z+refArrowsLength},
                    (Vector3){refArrowsPos.x,refArrowsPos.y,refArrowsPos.z+refArrowsLength*2},
                    refArrowsRadius,
                    0.0f,
                    10,
                    GREEN
                );
                DrawLine3D(refArrowsPos, (Vector3){refArrowsPos.x,refArrowsPos.y,refArrowsPos.z+refArrowsLength},GREEN);
            EndMode3D();
        EndTextureMode();
        BeginTextureMode(renderTextureForeground);
        ClearBackground((Color){0,0,0,0});  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                if(STATUS_MOTOR_ENABLED)
                {
                    DrawModel(*motorModel1, Vector3Zero(), DRAW_SCALE, COLOR_HL);
                    DrawModel(*motorModel2, Vector3Zero(), DRAW_SCALE, COLOR_HL);
                    DrawModel(*motorModel3, Vector3Zero(), DRAW_SCALE, COLOR_HL);
                }else
                {
                    DrawModel(*motorModel1, Vector3Zero(), DRAW_SCALE, COLOR_FG);
                    DrawModel(*motorModel2, Vector3Zero(), DRAW_SCALE, COLOR_FG);
                    DrawModel(*motorModel3, Vector3Zero(), DRAW_SCALE, COLOR_FG);
                }
            EndMode3D();
        EndTextureMode();

        BeginDrawing();
            ClearBackground(COLOR_BG);
            
            if(SHOW_3D_VIEW)
            {
                DrawTextureRec(renderTextureBackground.texture, viewRectangle, viewPos, WHITE);
                BeginShaderMode(shader);
                    // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                    DrawTextureRec(renderTextureModel.texture, viewRectangle, viewPos, WHITE);
                EndShaderMode();
                DrawTextureRec(renderTextureForeground.texture, viewRectangle, viewPos, WHITE);
                Rectangle viewBorderRectangle = {viewPos.x, viewPos.y, viewSize.x, viewSize.y};
                DrawRectangleLinesEx(viewBorderRectangle,BORDER_THICKNESS,COLOR_FG);
            }

            // Vector2 captureViewPos = { viewPos.x, viewPos.y+viewSize.y+MARGIN};
            // DrawTextureEx(captureTexture, captureViewPos, 0, viewSize.x/captureTexture.width,WHITE);
            // Rectangle captureViewRectangle = {captureViewPos.x, captureViewPos.y, captureTexture.width*(viewSize.x/captureTexture.width), captureTexture.height*(viewSize.x/captureTexture.width)};
            // DrawRectangleLinesEx(captureViewRectangle,BORDER_THICKNESS,COLOR_FG);

            i = 2;
            DrawTextEx(font,CURRENT_MENU->title,(Vector2){MARGIN,MARGIN},fontSize,1,COLOR_FG);
            std::vector<Option>::iterator startingOption = CURRENT_MENU->options.begin();
            if(HIGHLIGHTED_OPTION->id >= OPTIONS_PER_WINDOW)
            {
                // TODO: dividir por la cantidad de opciones para calcular el offset
                startingOption = CURRENT_MENU->options.begin()+OPTIONS_PER_WINDOW;
            }
            for (std::vector<Option>::iterator it = startingOption; it != CURRENT_MENU->options.end(); it++)
            {
                Vector2 optionPos = {MARGIN, MARGIN*i+BUTTON_SIZE*(i-1)-(BUTTON_SIZE-fontSize)};
                if(i == 2)  // la primera opción arranca debajo del título del menú
                    optionPos = {MARGIN, MARGIN*i+fontSize*(i-1)};
                Vector2 optionSize = {viewPos.x-MARGIN*2, BUTTON_SIZE};
                if(!SHOW_3D_VIEW && !SHOW_MENU_INFO && !SHOW_FIELD_VALUES)
                    optionSize.x = screenWidth-MARGIN*2;
                Rectangle optionRectangle = {optionPos.x, optionPos.y, optionSize.x, optionSize.y};
                // dibujo el recuadro de la opción
                DrawRectangleLinesEx(optionRectangle,BORDER_THICKNESS,COLOR_FG);
                // el color por defecto del texto es el color principal de foreground
                Color optionTextColor = COLOR_FG;
                if(it == HIGHLIGHTED_OPTION)
                {
                    // la opción seleccionada se grafica con los colores invertidos
                    optionTextColor = COLOR_BG;
                    DrawRectangleRec(optionRectangle, COLOR_FG);
                }
                // cargo el texto de la opción en un string
                sprintf(c," %s",it->text.c_str());
                // reposiciono el texto para que quede centrado en el recuadro
                optionPos.y += (BUTTON_SIZE-fontSize)/2;
                // dibujo el texto
                DrawTextEx(font,c,optionPos,fontSize,1,optionTextColor);
                // defino la posición del ícono de acción
                optionPos.x += optionSize.x-MARGIN-fontSize/2;
                // dibujo el ícono de acción
                if(it->text != "Atrás")
                    DrawTextEx(font,">",optionPos,fontSize,1,optionTextColor);
                else
                    DrawTextEx(font,"<",optionPos,fontSize,1,optionTextColor);
                
                if(SHOW_FIELD_VALUES)
                {
                    optionPos.x = MARGIN+optionSize.x+MARGIN;
                    sprintf(c,"%s",it->value.c_str());
                    DrawTextEx(font,c,optionPos,fontSize,1,COLOR_FG);
                }
                i++;
            }

            if(CURRENT_MENU->title == "Calibrar")
            {
                DrawRectangleLinesEx(dataViewRectangle,BORDER_THICKNESS,COLOR_FG);
                sprintf(c," X %.4f",octoKin.x);
                DrawTextEx(dataViewFont,c,Vector2Add(dataViewPos,(Vector2){0,MARGIN}),dataViewFontSize,1,COLOR_FG);
                sprintf(c," Y %.4f",octoKin.y);
                DrawTextEx(dataViewFont,c,Vector2Add(dataViewPos,(Vector2){0,MARGIN*2+dataViewFontSize}),dataViewFontSize,1,COLOR_FG);
                sprintf(c," Z %.4f",octoKin.z);
                DrawTextEx(dataViewFont,c,Vector2Add(dataViewPos,(Vector2){0,MARGIN*3+dataViewFontSize*2}),dataViewFontSize,1,COLOR_FG);
            }
            
            Vector2 statusBarPos = {DISPLAY_WIDTH-DISPLAY_WIDTH*SCREEN_DIVISION_RATIO-MARGIN,MARGIN};
            Vector2 statusBarSize = {DISPLAY_WIDTH*SCREEN_DIVISION_RATIO, MARGIN+fontSize};
            Vector2 iconPos = statusBarPos;
            if(JOB_RUNNING) DrawTextEx(font,"R",iconPos,fontSize,1,COLOR_HL);
            else DrawTextEx(font,"R",iconPos,fontSize,1,COLOR_FG);
            iconPos.x += fontSize;
            if(MODE_MANUAL) DrawTextEx(font,"M",iconPos,fontSize,1,COLOR_HL);
            else DrawTextEx(font,"M",iconPos,fontSize,1,COLOR_FG);
            iconPos.x += fontSize;
            if(!STATUS_MOTOR_ENABLED) DrawTextEx(font,"E",iconPos,fontSize,1,COLOR_HL);
            else DrawTextEx(font,"E",iconPos,fontSize,1,COLOR_FG);
            iconPos.x += fontSize;
            if(STATUS_VACUUM_PUMP) DrawTextEx(font,"B",iconPos,fontSize,1,COLOR_HL);
            else DrawTextEx(font,"B",iconPos,fontSize,1,COLOR_FG);

            if(JOB_SHOULD_STOP && EXECUTING_INSTRUCTION)
            {   
                DrawRectangle(statusBarPos.x,statusBarPos.y,statusBarSize.x,statusBarSize.y,COLOR_BG);
                DrawTextEx(font, "Deteniendo...", statusBarPos, fontSize*0.5f, 1, COLOR_FG);
            }

            if(SHOW_DEBUG_DATA)
            {
                sprintf(c,"FPS %d",GetFPS());
                DrawTextEx(font,c,(Vector2){screenWidth-MARGIN-3*fontSize,MARGIN},fontSize,1,COLOR_HL);
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
    UnloadFont(font);

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

//----------------------------------------------------------------------------------
// DEFINICION DE FUNCIONES
//----------------------------------------------------------------------------------

void goBackOneMenu(void)
{
    CURRENT_MENU = CURRENT_MENU->parent;
    HIGHLIGHTED_OPTION = CURRENT_MENU->options.begin();
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

void DrawProgressBarIndicator(const char* text, int progress, Font font)
{
    float fontSize = font.baseSize/2.0f;
    Vector2 barSize = {DISPLAY_WIDTH*SCREEN_DIVISION_RATIO, fontSize};
    Vector2 barPos = {DISPLAY_WIDTH-DISPLAY_WIDTH*SCREEN_DIVISION_RATIO-MARGIN, MARGIN+fontSize};
    Vector2 textPos = {barPos.x,barPos.y-fontSize};
    BeginDrawing();
        // ClearBackground(COLOR_BG);
        DrawRectangle(barPos.x,MARGIN,barSize.x,MARGIN*2+fontSize,COLOR_BG);
        Rectangle barRec = {barPos.x, barPos.y, barSize.x, barSize.y};
        DrawRectangleLinesEx(barRec, BORDER_THICKNESS, COLOR_FG);
        barRec.width *= progress/100.0f;
        DrawRectangleRec(barRec,COLOR_FG);
        DrawTextEx(font, text, textPos, fontSize, 1, COLOR_FG);
    EndDrawing();
}

char readRegister(int pin_data, int pin_pl, int pin_cp)
{
    char data = 0b01110101; // para que la pc lea los botones sin presionar y el joystick centrado

    // generar un pulso en PL para cargar los datos del registro paralelo al registro de desplazamiento
    #if ARCH_ARM
    data = 0;

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

// Función para parsear el archivo
std::vector<Componente> parsearArchivo(const std::string& nombreArchivo) {
    int progress = 0;

    std::vector<Componente> componentes;
    std::ifstream archivo(nombreArchivo);
    if (archivo.is_open()) {
        std::string linea;
        // Saltar las líneas de comentario y encabezado
        while (std::getline(archivo, linea)) {
            if (linea.find('#') == 0 || linea.empty()) {
                continue;
            }
            // Parsear la línea
            std::istringstream iss(linea);
            Componente componente;
            iss >> componente.reference >> componente.value >> componente.package
            >> componente.posx >> componente.posy >> componente.rotation;
            componentes.push_back(componente);
            DrawProgressBarIndicator("Parseando...", progress, font);
            if(progress < 90) progress += 10;
        }
        archivo.close();
    } else {
        std::cerr << "Error al abrir el archivo." << std::endl;
    }
    return componentes;
}

int executeInstruction(std::string instruction, OctoKinematics &octoKin)
{
    std::string format;

    if(instruction[0] == 'L')       // movimiento lineal
    {
        double x = octoKin.x;
        double y = octoKin.y;
        double z = octoKin.z;

        format = "%."+std::to_string(NUMERIC_PRECISION)+"f";
        format = "LX"+format+"Y"+format+"Z"+format;

        int has_x = 0, has_y = 0, has_z = 0;  // Bandera para coordenadas presentes

        // Buscar coordenada X
        char *x_pos = strchr((char*)instruction.c_str(), 'X');
        if (x_pos) {
            x = atof(x_pos + 1);  // Convertir valor después de 'X'
            has_x = 1;  // Indicar que X fue encontrada
        }

        // Buscar coordenada Y
        char *y_pos = strchr((char*)instruction.c_str(), 'Y');
        if (y_pos) {
            y = atof(y_pos + 1);  // Convertir valor después de 'Y'
            has_y = 1;  // Indicar que Y fue encontrada
        }

        // Buscar coordenada Z
        char *z_pos = strchr((char*)instruction.c_str(), 'Z');
        if (z_pos) {
            z = atof(z_pos + 1);  // Convertir valor después de 'Z'
            has_z = 1;  // Indicar que Z fue encontrada
        }

        // Mostrar resultados
        // if (has_x) printf("Coordenada X: %.4f\n", x);
        // if (has_y) printf("Coordenada Y: %.4f\n", y);
        // if (has_z) printf("Coordenada Z: %.4f\n", z);
        
        
        octoKin.linear_move(x, y, z, STEP_SIZE, 0);
    }else if(instruction[0] == 'D') // delay
    {
        char *d_pos = strchr((char*)instruction.c_str(), 'D');
        int us = atoi(d_pos + 1);  // Convertir valor después de 'D'
        usleep(us);
        
    }else if(instruction[0] == 'B') // controlar bomba
    {
        if(instruction[1] == '1') gpioWrite(PIN_BOMBA,1);
        else if(instruction[1] == '0') gpioWrite(PIN_BOMBA,0);
        else
        {
            fprintf(stderr, "invalid instruction: %s\n",instruction);
            return EXIT_FAILURE;
        }
    }else if(instruction[0] == 'E') // girar efector
    {
        bool dir = -1;

        if(instruction[1] == 'H') dir = 1;
        else if(instruction[1] == 'A') dir = 0;
        else
        {
            fprintf(stderr, "invalid instruction: %s\n",instruction);
            return EXIT_FAILURE;
        }

        // TODO: parsear argumento de grados de rotación

    }else if(instruction[0] == 'S') // cambiar tamaño del paso (STEP_SIZE) para movimiento lineal
    {
        char *s_pos = strchr((char*)instruction.c_str(), 'S');
        STEP_SIZE = atof(s_pos + 1);
    }

    return EXIT_SUCCESS;
}

std::vector<std::string> generateJob(std::vector<Componente> componentes)
{
    char aux_x[20], aux_y[20], aux_z[20];
    std::string format = "%."+std::to_string(NUMERIC_PRECISION)+"f";
    int progress = 0;
    int componentCount = 0;
    DrawProgressBarIndicator("Convirtiendo...", progress, font);

    Feeder aux_feeder = feeders.at(0);

    float anglePCB = Vector3Angle(Vector3Subtract(POS_PCB_REF2,POS_PCB_REF1),(Vector3){1.0f,0.0f,0.0f});
    Matrix transformPCB = MatrixIdentity();
    transformPCB = MatrixMultiply(transformPCB,MatrixRotate((Vector3){0.0f,1.0f,0.0f}, anglePCB));
    transformPCB = MatrixMultiply(transformPCB,MatrixTranslate(POS_PCB_REF1.x, POS_PCB_REF1.y, POS_PCB_REF1.z));

    std::vector<std::string> job;
    std::string instruction;
    
    instruction = "S0.02";
    job.push_back(instruction);
    instruction = "LX0Y0Z"+std::to_string(LIM_Z+30);
    job.push_back(instruction);
    instruction = "D250000";
    job.push_back(instruction);

    for (const auto& componente : componentes)
    {
        // guardar tipo de componente único
        bool componentAlreadyLoaded = false;
        for (const auto& unique_component : CURRENT_COMPONENTS)
        {
            if(componente.value == unique_component) componentAlreadyLoaded = true;
        }
        if(!componentAlreadyLoaded) CURRENT_COMPONENTS.push_back(componente.value);

        // cargar feeder correspondiente
        bool feederAssigned = false;
        for(auto& feeder : feeders)
        {
            if(feeder.component == componente.value)
            {
                aux_feeder = feeder;
                feederAssigned = true;
            }
        }

        if(!feederAssigned)
        {
            job.clear();
            fprintf(stderr, "[ERROR] component '%s' not assigned to feeder\n", componente.value.c_str());
            return job;
        }

        // paso grueso
        job.push_back("S0.02");
        // posición de approach
        instruction = "LX"+std::to_string(aux_feeder.approach.x)+"Y"+std::to_string(aux_feeder.approach.y)+"Z"+std::to_string(aux_feeder.approach.z);
        job.push_back(instruction);
        // paso fino
        job.push_back("S0.002");
        // posición de push
        instruction = "LX"+std::to_string(aux_feeder.push.x)+"Y"+std::to_string(aux_feeder.push.y)+"Z"+std::to_string(aux_feeder.push.z);
        job.push_back(instruction);
        // posición de pick manteniendo altura
        instruction = "LX"+std::to_string(aux_feeder.pick.x)+"Y"+std::to_string(aux_feeder.pick.y);
        job.push_back(instruction);

        // prender bomba
        job.push_back("B1");
        
        // baja hacia posición de pick
        instruction += "Z"+std::to_string(aux_feeder.pick.z);
        job.push_back(instruction);

        // delay para juntar
        job.push_back("D250000");

        // vuelve a altura de approach para salir
        instruction = "LZ"+std::to_string(aux_feeder.approach.z);
        job.push_back(instruction);
        // paso grueso
        job.push_back("S0.02");

        // secuencia de poner componente ---------------------------------------------------------------------------------------------
        // transformar marco de referencia robot->pcb
        Vector3 posComponentePCB = {componente.posx, componente.posy, 0.0f};
        Vector3 posComponenteRobot = Vector3Transform(posComponentePCB, transformPCB);

        sprintf(aux_x, format.c_str(), posComponenteRobot.x);
        sprintf(aux_y, format.c_str(), posComponenteRobot.y);
        sprintf(aux_z, format.c_str(), posComponenteRobot.z);
        
        // posición de componente manteniendo z de approach
        instruction = "LX";
        instruction.append(aux_x);
        instruction += "Y";
        instruction.append(aux_y);
        job.push_back(instruction);

        // delay
        job.push_back("D250000");

        // paso fino
        job.push_back("S0.002");
        // bajo en posición de componente hasta placa pcb
        instruction = "LZ";
        instruction.append(aux_z);
        job.push_back(instruction);
        // delay
        job.push_back("D100000");
        // apagar bomba
        job.push_back("B0");
        // delay
        job.push_back("1000000");

        // sube a z de approach
        sprintf(aux_z, format.c_str(), aux_feeder.approach.z);
        instruction = "LZ";
        instruction.append(aux_z);
        job.push_back(instruction);

        // delay
        job.push_back("D250000");

        componentCount++;
        progress = mapear(componentCount,0,componentes.size(),0,100);
        DrawProgressBarIndicator("Convirtiendo...", progress, font);
    }

    return job;
}

// Función para listar los archivos en una carpeta
std::vector<std::string> listarArchivos(const std::string& rutaCarpeta) {
    std::vector<std::string> archivos;
    DIR* dir;
    struct dirent* ent;

    // Abrir la carpeta
    if ((dir = opendir(rutaCarpeta.c_str())) != NULL) {
        // Leer los archivos en la carpeta
        while ((ent = readdir(dir)) != NULL) {
            // Verificar si es un archivo regular (no carpeta)
            if (ent->d_type == DT_REG) {
                archivos.push_back(ent->d_name);
            }
        }
        closedir(dir);
    } else {
        std::cerr << "Error al abrir la carpeta." << std::endl;
    }
    return archivos;
}

int configFileParser(std::string config_file_path) {
    std::ifstream configFile(config_file_path);
    std::string line, param_name, param_content;
    bool inside_feeder_block = false;
    Feeder aux_feeder;

    if (configFile.is_open()) {
        while (std::getline(configFile, line)) {
            // Eliminar espacios en blanco al inicio y final
            line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1);
            line.erase(0, line.find_first_not_of(" \t\n\r\f\v"));

            if (line.empty() || line[0] == '#') continue;

            if (inside_feeder_block)
            {
                if (line == "}") { // Fin del bloque FEEDER
                    feeders.push_back(aux_feeder);
                    inside_feeder_block = false;
                    continue;
                }

                size_t div_pos = line.find('=');
                if (div_pos == std::string::npos) {
                    std::cout << "Error en formato dentro de FEEDER: " << line << std::endl;
                    return EXIT_FAILURE;
                }

                std::string inner_param = line.substr(0, div_pos);
                std::string inner_value = line.substr(div_pos + 1);

                // Eliminar espacios alrededor del parámetro interno
                inner_param.erase(inner_param.find_last_not_of(" \t") + 1);
                inner_param.erase(0, inner_param.find_first_not_of(" \t"));

                if (inner_param == "ID") {
                    aux_feeder.id = std::stoi(inner_value);
                }
                else if (inner_param == "PUSH" || inner_param == "APPROACH" || inner_param == "PICK") {
                    std::istringstream iss(inner_value);
                    std::vector<float> coords;
                    std::string token;
                    
                    while (iss >> token) {
                        coords.push_back(std::stof(token));
                    }
                    
                    if (coords.size() != 3) {
                        std::cout << "Formato inválido para " << inner_param << ". Se necesitan 3 valores" << std::endl;
                        return EXIT_FAILURE;
                    }

                    if (inner_param == "PUSH") aux_feeder.push = {coords[0], coords[1], coords[2]};
                    else if (inner_param == "APPROACH") aux_feeder.approach = {coords[0], coords[1], coords[2]};
                    else aux_feeder.pick = {coords[0], coords[1], coords[2]};
                }
                else {
                    std::cout << "Parámetro desconocido en FEEDER: " << inner_param << std::endl;
                    return EXIT_FAILURE;
                }
            } else
            {
                if (line.find("FEEDER={") != std::string::npos) {
                    inside_feeder_block = true;
                    continue;
                }

                size_t div_pos = line.find('=');
                if (div_pos == std::string::npos) {
                    std::cout << "Error en formato de línea: " << line << std::endl;
                    return EXIT_FAILURE;
                }

                param_name = line.substr(0, div_pos);
                param_content = line.substr(div_pos + 1);

                param_name.erase(param_name.find_last_not_of(" \t") + 1);
                param_name.erase(0, param_name.find_first_not_of(" \t"));

                if (param_name == "POS_PCB_REF1")
                {
                    size_t first_space = param_content.find(' ');
                    size_t second_space = param_content.find(' ', first_space + 1);

                    if (first_space == std::string::npos || second_space == std::string::npos) {
                        std::cout << "Formato inválido para " << param_name << ". Se esperaban tres valores." << std::endl;
                        return EXIT_FAILURE;
                    }

                    float x = std::stof(param_content.substr(0, first_space));
                    float y = std::stof(param_content.substr(first_space + 1, second_space - first_space - 1));
                    float z = std::stof(param_content.substr(second_space + 1));

                    POS_PCB_REF1 = {x, y, z};
                }else if(param_name == "POS_PCB_REF2")
                {
                    size_t first_space = param_content.find(' ');
                    size_t second_space = param_content.find(' ', first_space + 1);

                    if (first_space == std::string::npos || second_space == std::string::npos) {
                        std::cout << "Formato inválido para " << param_name << ". Se esperaban tres valores." << std::endl;
                        return EXIT_FAILURE;
                    }

                    float x = std::stof(param_content.substr(0, first_space));
                    float y = std::stof(param_content.substr(first_space + 1, second_space - first_space - 1));
                    float z = std::stof(param_content.substr(second_space + 1));

                    POS_PCB_REF2 = {x, y, z};
                }else if (param_name == "PATH_FILES")
                {
                    // PATH_FILES = param_content;
                } else if (param_name == "STEPS_NUM")
                {
                    STEPS_NUM = std::stoi(param_content);
                } else if (param_name == "NUMERIC_PRECISION")
                {
                    // NUMERIC_PRECISION = std::stoi(param_content);
                } else
                {
                    std::cout << "Parámetro desconocido: " << line << std::endl;
                    return EXIT_FAILURE;
                }
            }
        }
        configFile.close();
    } else {
        std::cout << "Error al abrir el archivo de configuración" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

// int configFileParser(std::string config_file_path)
// {
//     std::ifstream configFile(config_file_path);
//     std::string line, param_name, param_content;

//     if (configFile.is_open()) {
//         while (std::getline(configFile, line)) {
//             // Ignorar líneas vacías y comentarios
//             if (line.empty() || line[0] == '#')
//                 continue;

//             size_t div_pos = line.find_last_of("=");
//             if (div_pos == std::string::npos) {
//                 std::cout << "Error en formato de línea: " << line << std::endl;
//                 return EXIT_FAILURE;
//             }

//             param_name = line.substr(0, div_pos);
//             param_content = line.substr(div_pos + 1);

//             if (param_name == "POS_FEEDER" || param_name == "POS_PCB")
//             {
//                 size_t first_space = param_content.find(' ');
//                 size_t second_space = param_content.find(' ', first_space + 1);

//                 if (first_space == std::string::npos || second_space == std::string::npos) {
//                     std::cout << "Formato inválido para " << param_name << ". Se esperaban tres valores." << std::endl;
//                     return EXIT_FAILURE;
//                 }

//                 float x = std::stof(param_content.substr(0, first_space));
//                 float y = std::stof(param_content.substr(first_space + 1, second_space - first_space - 1));
//                 float z = std::stof(param_content.substr(second_space + 1));

//                 if (param_name == "POS_FEEDER")
//                     POS_FEEDER = {x, y, z};
//                 else
//                     POS_PCB = {x, y, z};
//             }
//             else if (param_name == "PATH_FILES") {
//                 // PATH_FILES = param_content;
//             }
//             else if (param_name == "STEPS_NUM") {
//                 // STEPS_NUM = std::stoi(param_content);
//             }
//             else if (param_name == "NUMERIC_PRECISION") {
//                 // NUMERIC_PRECISION = std::stoi(param_content);
//             }
//             else {
//                 std::cout << "Parámetro desconocido: " << line << std::endl;
//                 return EXIT_FAILURE;
//             }
//         }
//         configFile.close();
//     } else {
//         std::cout << "Error al abrir el archivo de configuración" << std::endl;
//         return EXIT_FAILURE;
//     }
//     return EXIT_SUCCESS;
// }

int writeConfigFile(const std::string& config_file_path)
{
    std::ofstream output_file(config_file_path);
    
    if (!output_file.is_open()) {
        std::cerr << "Error al abrir archivo para escritura: " << config_file_path << std::endl;
        return EXIT_FAILURE;
    }

    // Escribe cada parámetro con el formato especificado
    output_file << "PATH_FILES=" << PATH_FILES << "\n";
    
    output_file << "POS_PCB_REF1=" 
              << std::fixed << std::setprecision(NUMERIC_PRECISION) << POS_PCB_REF1.x << " "
              << POS_PCB_REF1.y << " "
              << POS_PCB_REF1.z << "\n";
    output_file << "POS_PCB_REF2=" 
              << std::fixed << std::setprecision(NUMERIC_PRECISION) << POS_PCB_REF2.x << " "
              << POS_PCB_REF2.y << " "
              << POS_PCB_REF2.z << "\n";
    
    output_file << "STEPS_NUM=" << STEPS_NUM << "\n";
    output_file << "NUMERIC_PRECISION=" << NUMERIC_PRECISION << "\n";

    for (const auto& feeder : feeders)
    {
        output_file << "POS_FEEDER={\n" 
            << "ID=" << feeder.id << "\n"
            << "PUSH="  << std::fixed << std::setprecision(NUMERIC_PRECISION) << feeder.push.x << " " << feeder.push.y << " " << feeder.push.z << "\n"
            << "APPROACH="  << std::fixed << std::setprecision(NUMERIC_PRECISION) << feeder.approach.x << " " << feeder.approach.y << " " << feeder.approach.z << "\n"
            << "PICK="  << std::fixed << std::setprecision(NUMERIC_PRECISION) << feeder.pick.x << " " << feeder.pick.y << " " << feeder.pick.z << "\n"
            << "}\n";
    }

    output_file.close();
    return EXIT_SUCCESS;
}