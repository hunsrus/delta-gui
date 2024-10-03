#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso

// detección de la arquitectura
#if defined(__aarch64__) || defined(_M_ARM64) || !defined(__x86_64__) || !defined(__x86_64__)
    #define ARCH_ARM true
#else
    #define ARCH_ARM false
#endif
//#define PLATFORM_DESKTOP
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include "rlgl.h"
#include "DeltaKinematics.h"

// deteccion de imagenes
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#if ARCH_ARM
    #include "stepper.h"
#endif

// versión de glsl para shaders
#define GLSL_VERSION 100


#if ARCH_ARM
    #define DISPLAY_WIDTH 320
    #define DISPLAY_HEIGHT 240
#else
    #define DISPLAY_WIDTH 1024
    #define DISPLAY_HEIGHT 768
#endif
#define TARGET_FPS 30
#define MARGIN 20*(DISPLAY_HEIGHT/768.0f)

#define CAMERA_FOV 90

#define ARM_LENGTH 90.0f
#define ROD_LENGTH 166.0f
#define BASS_TRI 35.0f
#define PLATFORM_TRI 42.0f
#define PLATFORM_POS (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}

#define STEPS_PER_REV 200 // 1.8 grados

static bool SHOW_FPS = true;
static bool STARTING_ANIMATION = false;

static Color COLOR_BG = {34,34,34,255};
static Color COLOR_FG = {238,238,238,255};

static cv::Mat image;
static bool CAMERA_AVAILABLE = true;
static bool CAPTURE_READY = false;

static bool EXIT = false;

// Determinar el paso actual
static int stepIndex = 0;

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

// Definir la secuencia de pasos para un motor paso a paso unipolar de 4 bobinas
int stepSequence[4][4] = {
    {0, 0, 1, 1}, // Paso 1
    {1, 0, 0, 1}, // Paso 2
    {1, 1, 0, 0}, // Paso 3
    {0, 1, 1, 0}  // Paso 4
};

// Función para establecer el estado de las bobinas
void setCoils(int coil1, int coil2, int coil3, int coil4) {
    #if ARCH_ARM
        // gpioWrite(PIN_BOB1, coil1);
        // gpioWrite(PIN_BOB2, coil2);
        // gpioWrite(PIN_BOB3, coil3);
        // gpioWrite(PIN_BOB4, coil4);
    #else
        //printf("Bobinas: %d %d %d %d\n", coil1, coil2, coil3, coil4);
    #endif
}

// Función para mover el motor de un ángulo actual a un ángulo objetivo
void moveToAngle(double currentAngle, double targetAngle) {
    // Calcular la diferencia de ángulo
    double angleDiff = targetAngle - currentAngle;

    // Calcular el número de pasos necesarios
    int steps = (int)round(angleDiff * STEPS_PER_REV / 360.0);

    // Determinar la dirección del movimiento
    bool direction = (steps > 0) ? 1 : 0;

    // Hacer los pasos necesarios
    #if ARCH_ARM
        do4Steps(abs(steps), direction);
    #else
    /*
        for (int i = 0; i < abs(steps); i++) {
            stepIndex += direction;
            if (stepIndex > 3) stepIndex = 0;
            if (stepIndex < 0) stepIndex = 3;

            // Activar las bobinas para el paso actual
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);

            // Añadir un pequeño retraso para permitir que el motor se mueva (ajustar según sea necesario)
            usleep(1000); // 1000 microsegundos = 1 milisegundo
        }
    */
    #endif
}

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = DISPLAY_WIDTH;
    const int screenHeight = DISPLAY_HEIGHT;

    int i;
    char c[100];
    float auxValue = 0;

    int animationStep = 0;
    int animationState = 0;

    DeltaKinematics dk = DeltaKinematics(ARM_LENGTH, ROD_LENGTH, BASS_TRI, PLATFORM_TRI);
    double x = 0, y = 0, z = -ROD_LENGTH/2.0f;
    double lastX = -1, lastY = -1, lastZ = -1;
    double lastA, lastB, lastC;
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
    Vector3 rod1Pos = (Vector3){arm2Pos.x,static_cast<float>(arm2Pos.y+ARM_LENGTH*sin(dk.b*DEG2RAD)),arm2Pos.z};
    Vector3 rod2Pos = (Vector3){arm1Pos.x,static_cast<float>(arm1Pos.y+ARM_LENGTH*sin(dk.a*DEG2RAD)),arm1Pos.z};
    Vector3 rod3Pos = (Vector3){arm1Pos.x,static_cast<float>(arm1Pos.y+ARM_LENGTH*sin(dk.c*DEG2RAD)),arm1Pos.z};
    Vector3 baseJoint1, baseJoint2, baseJoint3;
    Vector3 basePos = (Vector3){static_cast<float>(x),static_cast<float>(z),static_cast<float>(y)};
    Vector3 arm1Projection, arm2Projection, arm3Projection;

	//SetConfigFlags(FLAG_FULLSCREEN_MODE);
    //SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "delta gui test");

    #if ARCH_ARM
        std::cout << "ARCHITECTURE: ARM" << std::endl;
    #else
        std::cout << "ARCHITECTURE: x86_64" << std::endl;
    #endif

    // CARGAR LOS MODELOS DESPUÉS DE INICIAR LA VENTANA
    Model* platformModel = new Model(LoadModelFromMesh(GenMeshPoly(10,PLATFORM_TRI)));
    //Model* platformModel = new Model(LoadModel(std::string("resources/models/platform/platform.obj").c_str()));
    //platformModel->transform = MatrixScale(1000,1000,1000);
    //platformModel->transform = MatrixMultiply(platformModel->transform, MatrixRotate((Vector3){0,1,0},45*DEG2RAD));
    //platformModel->transform = MatrixMultiply(platformModel->transform, MatrixTranslate(0,-24,0));
    Model* baseModel = new Model(LoadModelFromMesh(GenMeshPoly(10,BASS_TRI)));
    //Model* baseModel = new Model(LoadModel(std::string("resources/models/effector/effector.obj").c_str()));
    //baseModel->transform = MatrixScale(1000,1000,1000);
    //baseModel->transform = MatrixMultiply(baseModel->transform, MatrixRotate((Vector3){0,1,0},45*DEG2RAD));
    Model* armModel1 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,ARM_LENGTH,4.0f)));
    Model* armModel2 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,ARM_LENGTH,4.0f)));
    Model* armModel3 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,ARM_LENGTH,4.0f)));
    Matrix arm1InitialMatrix = MatrixIdentity();
    Matrix arm2InitialMatrix = MatrixIdentity();
    Matrix arm3InitialMatrix = MatrixIdentity();
    arm1InitialMatrix = MatrixMultiply(arm1InitialMatrix,MatrixTranslate(0,ARM_LENGTH/2.0f,0));
    arm2InitialMatrix = MatrixMultiply(arm2InitialMatrix,MatrixTranslate(0,ARM_LENGTH/2.0f,0));
    arm3InitialMatrix = MatrixMultiply(arm3InitialMatrix,MatrixTranslate(0,ARM_LENGTH/2.0f,0));
    arm1InitialMatrix = MatrixMultiply(arm1InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    arm2InitialMatrix = MatrixMultiply(arm2InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    arm3InitialMatrix = MatrixMultiply(arm3InitialMatrix,MatrixRotate((Vector3){0,0,1}, -M_PI_2));
    /*
    Model* armModel1 = new Model(LoadModel(std::string("resources/models/arm/simplify_arm.obj").c_str()));
    Model* armModel2 = new Model(LoadModel(std::string("resources/models/arm/simplify_arm.obj").c_str()));
    Model* armModel3 = new Model(LoadModel(std::string("resources/models/arm/simplify_arm.obj").c_str()));
    */
    Model* rodModel1 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,ROD_LENGTH,4.0f)));
    Model* rodModel2 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,ROD_LENGTH,4.0f)));
    Model* rodModel3 = new Model(LoadModelFromMesh(GenMeshCube(4.0f,ROD_LENGTH,4.0f)));
    Matrix rod1InitialMatrix = MatrixIdentity();
    Matrix rod2InitialMatrix = MatrixIdentity();
    Matrix rod3InitialMatrix = MatrixIdentity();
    rod1InitialMatrix = MatrixMultiply(rod1InitialMatrix,MatrixTranslate(0,ROD_LENGTH/2.0f,0));
    rod2InitialMatrix = MatrixMultiply(rod2InitialMatrix,MatrixTranslate(0,ROD_LENGTH/2.0f,0));
    rod3InitialMatrix = MatrixMultiply(rod3InitialMatrix,MatrixTranslate(0,ROD_LENGTH/2.0f,0));
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
    rodModel1->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = RED;
    rodModel2->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GREEN;
    rodModel3->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BLUE;

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

    // Define the camera to look into our 3d world
    //Camera camera = { {-20.0f, 12.0f, 0.0f}, { 0.0f, 4.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    Camera camera = { {-PLATFORM_TRI*10, (ARM_LENGTH+ROD_LENGTH)*1.5f, 0.0f}, {PLATFORM_POS.x, PLATFORM_POS.y/1.5f, PLATFORM_POS.z}, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 180.0f;
    camera.projection = CAMERA_PERSPECTIVE;

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
    float shaderResolution[2] = {840, 840};
    int edgeColorLoc = GetShaderLocation(shader, "edgeColor");
    int backgroundColorLoc = GetShaderLocation(shader, "backgroundColor");
    int resolutionLoc = GetShaderLocation(shader, "resolution");
    SetShaderValue(shader, edgeColorLoc, edgeColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, backgroundColorLoc, backgroundColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, resolutionLoc, shaderResolution, SHADER_UNIFORM_VEC2);


    //SetCameraMode(camera, CAMERA_THIRD_PERSON);
	//SetCameraMode(camera, CAMERA_ORBITAL);
    SetCameraMode(camera, CAMERA_CUSTOM);
    HideCursor();
    SetTargetFPS(TARGET_FPS);

    // inicio control de i/o
    #if ARCH_ARM
        if (gpioInitialise() < 0)
        {
            fprintf(stderr, "pigpio initialisation failed\n");
            return 1;
        }
        fprintf(stdout, "pigpio initialisation complete\n");

        gpioSetMode(DS,PI_OUTPUT);
        gpioSetMode(SHCP,PI_OUTPUT);
        gpioSetMode(STCP,PI_OUTPUT);
        gpioSetMode(OE,PI_OUTPUT);
        gpioSetMode(MR, PI_OUTPUT);
        gpioSetMode(VACUUM, PI_OUTPUT);

        gpioWrite(MR, 1);
        gpioWrite(OE, 0);
        gpioWrite(DS, 0);
        gpioWrite(SHCP, 0);
        gpioWrite(STCP, 0);
        gpioWrite(VACUUM, 0);

        waitMicroSeconds = 500; 
        pulseCount = 5;

        // vueltas de testeo
        /*
        for(int i=0;i<50;i++){
            stepIndex = 0;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
            stepIndex++;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
            stepIndex++;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
            stepIndex++;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
        }
        usleep(10000);
        for(int i=0;i<50;i++){
            stepIndex = 3;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
            stepIndex--;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
            stepIndex--;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
            stepIndex--;
            setCoils(stepSequence[stepIndex][0], stepSequence[stepIndex][1], stepSequence[stepIndex][2], stepSequence[stepIndex][3]);
            usleep(10000);
        }
        usleep(10000);
        setCoils(1, 1, 1, 1);
        */
    #endif

    // inicio captura de video
    Texture2D captureTexture;

    std::thread t1(captureVideo);

    while(!CAPTURE_READY && CAMERA_AVAILABLE){};
    if(CAMERA_AVAILABLE)
        captureTexture = LoadTextureFromImage(MatToImage(image));

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

        if(CAPTURE_READY)
        {
            UpdateTexture(captureTexture,MatToImage(image).data);
            CAPTURE_READY = false;
        }

        if(STARTING_ANIMATION)
        {
            animationStep++;
            if(animationState == 1)
            {
                camera.fovy = camera.fovy-0.0001f*animationStep/GetFrameTime();
                if(camera.fovy <= CAMERA_FOV) STARTING_ANIMATION = false;
                //if(animationStep > 100) STARTING_ANIMATION = false;
            }
        }else camera.fovy = CAMERA_FOV;

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

        z = -ROD_LENGTH+sin(GetTime()*2.0f)*40.0f;

        if(lastX != x || lastY != y || lastZ != z) // calcula solo si hubo variaciones
        {
            // Cálculos de cinemática
            dk.inverse(x,y,z);

            rod1Pos = (Vector3){arm1Pos.x,static_cast<float>(arm1Pos.y-ARM_LENGTH*sin(dk.a*DEG2RAD)),static_cast<float>(arm1Pos.z-ARM_LENGTH*cos(dk.a*DEG2RAD))};
            rod2Pos = (Vector3){static_cast<float>(arm2Pos.x+ARM_LENGTH*cos(dk.b*DEG2RAD)*cos(30*DEG2RAD)),static_cast<float>(arm2Pos.y-ARM_LENGTH*sin(dk.b*DEG2RAD)),static_cast<float>(arm2Pos.z+ARM_LENGTH*cos(dk.b*DEG2RAD)*sin(30*DEG2RAD))};
            rod3Pos = (Vector3){static_cast<float>(arm3Pos.x-ARM_LENGTH*cos(dk.c*DEG2RAD)*cos(30*DEG2RAD)),static_cast<float>(arm3Pos.y-ARM_LENGTH*sin(dk.c*DEG2RAD)),static_cast<float>(arm3Pos.z+ARM_LENGTH*cos(dk.c*DEG2RAD)*sin(30*DEG2RAD))};

            basePos = (Vector3){static_cast<float>(x),static_cast<float>(PLATFORM_POS.y+z),static_cast<float>(y)};

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

            armModel1->transform = arm1InitialMatrix;
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixRotate(arm1Axis,dk.a*DEG2RAD));
            armModel1->transform = MatrixMultiply(armModel1->transform,MatrixTranslate(arm1Pos.x,arm1Pos.y,arm1Pos.z));

            armModel2->transform = arm2InitialMatrix;
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixRotate(arm2Axis,dk.b*DEG2RAD));
            armModel2->transform = MatrixMultiply(armModel2->transform,MatrixTranslate(arm2Pos.x,arm2Pos.y,arm2Pos.z));

            armModel3->transform = arm3InitialMatrix;
            armModel3->transform = MatrixMultiply(armModel3->transform,MatrixRotate(arm3Axis,dk.c*DEG2RAD));
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

            if(fabs(dk.a - lastA) > 1.8)
            {
                motorID = 0;
                moveToAngle(lastA, dk.a);

                lastA = dk.a;
            }
            if(fabs(dk.b - lastB) > 1.8)
            {
                motorID = 1;
                moveToAngle(lastB, dk.b);

                lastB = dk.b;
            }
            if(fabs(dk.c - lastC) > 1.8)
            {
                motorID = 2;
                moveToAngle(lastC, dk.c);

                lastC = dk.c;
            }
        }

        lastX = x;
        lastY = y;
        lastZ = z;

        //armModel->transform = MatrixRotateXYZ((Vector3){ 0, 0, x });
        //----------------------------------------------------------------------------------
        // Dibuja
        //----------------------------------------------------------------------------------

        BeginTextureMode(renderTextureModel);       // Enable drawing to texture
            ClearBackground((Color){0,0,0,0});  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                DrawModel(*platformModel,PLATFORM_POS,1.0f,WHITE);
                DrawModel(*baseModel,basePos,1.0f,WHITE);
                DrawModel(*armModel1, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*armModel2, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*armModel3, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*rodModel1, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*rodModel2, Vector3Zero(), 1.0f, WHITE);
                DrawModel(*rodModel3, Vector3Zero(), 1.0f, WHITE);
                //DrawCircle3D(PLATFORM_POS,PLATFORM_TRI,(Vector3){1.0f,0.0f,0.0f},90.0f,WHITE);
            EndMode3D();                // End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode();               // End drawing to texture (now we have a texture available for next passes)
        BeginTextureMode(renderTextureBackground);
        ClearBackground(COLOR_BG);  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
                DrawGrid((int)PLATFORM_TRI/1.5f,PLATFORM_TRI);
            EndMode3D();
        EndTextureMode();

        BeginDrawing();
            ClearBackground(COLOR_BG);
            
            Vector2 viewSize = {(float)renderTextureModel.texture.width/4, (float)renderTextureModel.texture.height/2};
            Rectangle viewRectangle = {(float)renderTextureModel.texture.width/2-viewSize.x/2, (float)renderTextureModel.texture.height/2-viewSize.y/2, viewSize.x, -viewSize.y};
            Vector2 viewPos = { screenWidth-viewSize.x-MARGIN, MARGIN};
            DrawTextureRec(renderTextureBackground.texture, viewRectangle, viewPos, WHITE);
            BeginShaderMode(shader);
                // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                DrawTextureRec(renderTextureModel.texture, viewRectangle, viewPos, WHITE);
            EndShaderMode();
            Rectangle viewBorderRectangle = {viewPos.x, viewPos.y, viewSize.x, viewSize.y};
            DrawRectangleLinesEx(viewBorderRectangle,2.0f,COLOR_FG);

            Vector2 captureViewPos = { viewPos.x, viewPos.y+viewSize.y+MARGIN};
            DrawTextureEx(captureTexture, captureViewPos, 0, viewSize.x/captureTexture.width,WHITE);
            Rectangle captureViewRectangle = {captureViewPos.x, captureViewPos.y, captureTexture.width*(viewSize.x/captureTexture.width), captureTexture.height*(viewSize.x/captureTexture.width)};
            DrawRectangleLinesEx(captureViewRectangle,2.0f,COLOR_FG);

            //sprintf(c,"ANGLE: %f",auxValue);
            //DrawText(c, MARGIN, MARGIN*5, MARGIN, COLOR_FG);

            if(SHOW_FPS)
            {
                sprintf(c,"FPS %d",GetFPS());
                DrawText(c, MARGIN, MARGIN, MARGIN, COLOR_FG);
                sprintf(c,"STARTING_ANIMATION %i",STARTING_ANIMATION);
                DrawText(c, MARGIN, MARGIN*3, MARGIN, COLOR_FG);
                sprintf(c,"SHADER_RESOLUTION %.2f",shaderResolution[0]);
                DrawText(c, MARGIN, MARGIN*5, MARGIN, COLOR_FG);
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

    std::cout << "Joining capture thread...";
	while(!t1.joinable()){}
	t1.join();
	std::cout << " Done." << std::endl;

    return 0;
}
