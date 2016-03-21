/* Includes */
#include <vector>
#include <algorithm>
using namespace std;

#include "GL/freeglut.h"

extern "C" {
#include "OBJLoader\glm.h"
}

#include "BMPLoader\BMPToTexture.h"

#include "TerrainLoader\imageloader.h"
#include "TerrainLoader\vec3f.h"
#include "TerrainLoader\Terrain.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "btBulletDynamicsCommon.h"
#include "BulletCollision\CollisionShapes\btHeightfieldTerrainShape.h"

/* Window Size */
int windowWidth = 1440;
int windowHeight = 900;

/* Point2D Object */
typedef struct Point2D {
    float x;
    float y;
} Point2D;

/* Point3D Object */
typedef struct Point3D {
    float x;
    float y;
    float z;
} Point3D;

/* Camera */
float camera[6] = { 0.0f, -20.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // posX, posY, posZ, focX, focY, focZ
vector<Point3D> path;

/* Sky */
GLuint skyDisplayList;

/* Car */
GLuint carBodyModelDisplayList;
GLuint carRimModelDisplayList;
btRaycastVehicle* vehicle;
btRigidBody* chassis;
btRaycastVehicle::btVehicleTuning tuning;

/* Texture Object */
typedef struct Texture {
    GLuint id;
    unsigned char* data;
} Texture;
Texture skyFrontTex, skyBackTex, skyLeftTex, skyRightTex, skyTopTex, skyBottomTex, speedometerBGTex, carRimTex, floorTex;

/* World */
GLuint terrainDisplayList;
Terrain* terrain;
btHeightfieldTerrainShape* terrainShape;
vector<float> heights;

/* Pressed key state */
bool keyState[4] = { false, false, false, false };
//                     'W',   'S',   'A',   'D'

/* Physics */
btDynamicsWorld* world;
btDispatcher* dispatcher;
btBroadphaseInterface* broadPhase;
btDefaultCollisionConfiguration* configuration;
btSequentialImpulseConstraintSolver* solver;

/* GLUT initialization */
void initializeGLUT(int* argc, char* argv[]) {
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);

    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - windowWidth) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - windowHeight) / 2);
}

/* OpenGL Initialization */
void initializeOpenGL() {
    // Depth Testing
    glEnable(GL_DEPTH_TEST);

    // Shade Model
    glShadeModel(GL_SMOOTH);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

btRigidBody* createBox(float w, float h, float d, float x, float y, float z, float mass) {
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(x, y, z));
    btDefaultMotionState* motionState = new btDefaultMotionState(t);
    btBoxShape* shape = new btBoxShape(btVector3(w / 2.0, h / 2.0, d / 2.0));
    btVector3 inertia(0, 0, 0);
    if (mass != 0) shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, shape, inertia);
    btRigidBody* body = new btRigidBody(info);
    return body;
}

void render(btRigidBody* b) {
    btBoxShape* shape = (btBoxShape*)b->getCollisionShape();
    btTransform t;
    b->getMotionState()->getWorldTransform(t);
    float mat[16];
    t.getOpenGLMatrix(mat);
    btVector3 vec = shape->getHalfExtentsWithoutMargin();
    glPushMatrix();
    glColor3f(0, 0, 1);
    glMultMatrixf(mat);
    glCallList(carBodyModelDisplayList);
    glPopMatrix();
}

void renderCylinder(float* m) {
    glColor3f(0, 1, 0);
    glPushMatrix();
    glMultMatrixf(m);
    glRotatef(90, 0, 1, 0);
    glCallList(carRimModelDisplayList);
    glPopMatrix();
}

/* Initialize Physics Engine */
void initializePhysicsEngine() {
    configuration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(configuration);
    broadPhase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();

    world = new btDiscreteDynamicsWorld(dispatcher, broadPhase, solver, configuration);
    world->setGravity(btVector3(0, -10, 0));

    terrain = loadTerrain("Textures/heightmap.bmp", 255.0f);
    for (int x = 0; x < 512; x++) {
        for (int z = 0; z < 512; z++) {
            heights.push_back(terrain->getHeight(x, z));
        }
    }
    terrainShape = new btHeightfieldTerrainShape(512, 512, &(heights[0]), 255, 1, true, false);
    float scale = 100.0f / max(terrain->width(), terrain->length());
    terrainShape->setLocalScaling(btVector3(scale * 5, 0.25, scale * 5));

    btTransform t;
    t.setIdentity();
    t.getBasis().setEulerZYX(0, -M_PI / 2.0, 0);

    btMotionState* motion = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(0.0, motion, terrainShape);
    btRigidBody* body = new btRigidBody(info);
    world->addRigidBody(body);

    btVehicleRaycaster* caster = new btDefaultVehicleRaycaster(world);
    chassis = createBox(1.5f, 1.0f, 3.5f, 0.0f, -23.0f, 0.0f, 800.0f);
    world->addRigidBody(chassis);

    Point3D initialPath;
    initialPath.x = 0.0f;
    initialPath.y = -23.0f;
    initialPath.z = 0.0f;
    path.push_back(initialPath);

    vehicle = new btRaycastVehicle(tuning, chassis, caster);

    vehicle->addWheel(btVector3(-0.8, 0.0, 1.34), btVector3(0, -1, 0), btVector3(-1, 0, 0), 0.6, 0.335, tuning, true);
    vehicle->addWheel(btVector3(0.8, 0.0, 1.34), btVector3(0, -1, 0), btVector3(-1, 0, 0), 0.6, 0.335, tuning, true);
    vehicle->addWheel(btVector3(-0.85, 0.0, -1.42), btVector3(0, -1, 0), btVector3(-1, 0, 0), 0.65, 0.335, tuning, false);
    vehicle->addWheel(btVector3(0.85, 0.0, -1.42), btVector3(0, -1, 0), btVector3(-1, 0, 0), 0.65, 0.335, tuning, false);

    world->addVehicle(vehicle);
    vehicle->setCoordinateSystem(0, 1, 2);

    for (int i = 0; i < vehicle->getNumWheels(); i++) {
        btWheelInfo& wheel = vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = 15;
        wheel.m_wheelsDampingRelaxation = 10;
        wheel.m_wheelsDampingCompression = 2;
        wheel.m_frictionSlip = 5000;
        wheel.m_rollInfluence = 0.01f;
        wheel.m_maxSuspensionTravelCm = 15;
    }
}

/* Convert degree to radian */
float degreeToRadian(float angle) {
    return angle*(M_PI / 180.0f);
}

/* Key pressed function */
void keyPressed(unsigned char key, int mouseX, int mouseY) {
    if (key == 'w') keyState[0] = true;
    if (key == 's') keyState[1] = true;
    if (key == 'a') keyState[2] = true;
    if (key == 'd') keyState[3] = true;
}

/* Key up function */
void keyUp(unsigned char key, int mouseX, int mouseY) {
    if (key == 'w') keyState[0] = false;
    if (key == 's') keyState[1] = false;
    if (key == 'a') keyState[2] = false;
    if (key == 'd') keyState[3] = false;
}

/* Update Camera Position */
void updateCameraPosition() {
    btVector3 carPosition = chassis->getCenterOfMassPosition();
    if ((path[path.size() - 1].x != carPosition.getX()) && (path[path.size() - 1].y != carPosition.getY()) && (path[path.size() - 1].z != carPosition.getZ())) {
        Point3D newPosition;
        newPosition.x = carPosition.getX();
        newPosition.y = carPosition.getY();
        newPosition.z = carPosition.getZ();

        path.push_back(newPosition);
    }

    if (path.size() > 60) {
        path.erase(path.begin());
    }

    camera[0] = path[0].x;
    camera[1] = path[0].y + 3.0f;
    camera[2] = path[0].z;

    camera[3] = carPosition.getX();
    camera[4] = carPosition.getY();
    camera[5] = carPosition.getZ();
}

/* Keyboard input handler */
void handleKeyboardInput() {
    vehicle->applyEngineForce(0, 0);
    vehicle->applyEngineForce(0, 1);
    vehicle->applyEngineForce(0, 2);
    vehicle->applyEngineForce(0, 3);
    vehicle->setSteeringValue(0, 0);
    vehicle->setSteeringValue(0, 1);

    if (!(keyState[0] && keyState[1])) {
        if (keyState[0] == true) {
            vehicle->applyEngineForce(3000, 0);
            vehicle->applyEngineForce(3000, 1);
            vehicle->applyEngineForce(3000, 2);
            vehicle->applyEngineForce(3000, 3);
        }

        if (keyState[1] == true) {
            vehicle->applyEngineForce(-1600, 0);
            vehicle->applyEngineForce(-1600, 1);
            vehicle->applyEngineForce(-1600, 2);
            vehicle->applyEngineForce(-1600, 3);
        }
    }

    if (!(keyState[2] && keyState[3])) {
        if (keyState[2] == true) {
            vehicle->setSteeringValue(0.5, 0);
            vehicle->setSteeringValue(0.5, 1);
        }

        if (keyState[3] == true) {
            vehicle->setSteeringValue(-0.5, 0);
            vehicle->setSteeringValue(-0.5, 1);
        }
    }

    updateCameraPosition();
}

/* Position The Light */
void positionLight() {
    float lightPosition[4] = { -320.0f, 320.0f, 320.0f, 0.0f };
    float lightAmbient[4] = { 0.8f, 0.8f, 0.8f, 1.0f };
    float lightDiffuse[4] = { 0.8f, 0.8f, 0.8f, 1.0f };
    float lightSpecular[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

/* Load All Textures */
void loadAllTextures() {
    int skyFrontTexWidth = 512, skyFrontTexHeight = 512;
    skyFrontTex.data = BmpToTexture("Textures/skyFront.bmp", &skyFrontTexWidth, &skyFrontTexHeight);
    glGenTextures(1, &skyFrontTex.id);

    int skyBackTexWidth = 512, skyBackTexHeight = 512;
    skyBackTex.data = BmpToTexture("Textures/skyBack.bmp", &skyBackTexWidth, &skyBackTexHeight);
    glGenTextures(1, &skyBackTex.id);

    int skyLeftTexWidth = 512, skyLeftTexHeight = 512;
    skyLeftTex.data = BmpToTexture("Textures/skyLeft.bmp", &skyLeftTexWidth, &skyLeftTexHeight);
    glGenTextures(1, &skyLeftTex.id);

    int skyRightTexWidth = 512, skyRightTexHeight = 512;
    skyRightTex.data = BmpToTexture("Textures/skyRight.bmp", &skyRightTexWidth, &skyRightTexHeight);
    glGenTextures(1, &skyRightTex.id);

    int skyTopTexWidth = 512, skyTopTexHeight = 512;
    skyTopTex.data = BmpToTexture("Textures/skyTop.bmp", &skyTopTexWidth, &skyTopTexHeight);
    glGenTextures(1, &skyTopTex.id);

    int skyBottomTexWidth = 512, skyBottomTexHeight = 512;
    skyBottomTex.data = BmpToTexture("Textures/skyBottom.bmp", &skyBottomTexWidth, &skyBottomTexHeight);
    glGenTextures(1, &skyBottomTex.id);

    int speedometerBGTexWidth = 400, speedometerBGTexHeight = 400;
    speedometerBGTex.data = BmpToTexture("Textures/speedometer.bmp", &speedometerBGTexWidth, &speedometerBGTexHeight);
    glGenTextures(1, &speedometerBGTex.id);

    int carRimWidth = 512, carRimHeight = 512;
    carRimTex.data = BmpToTexture("Textures/rim.bmp", &carRimWidth, &carRimHeight);
    glGenTextures(1, &carRimTex.id);

    int floorTexWidth = 1024, floorTexHeight = 1024;
    floorTex.data = BmpToTexture("Textures/ground.bmp", &floorTexWidth, &floorTexHeight);
    glGenTextures(1, &floorTex.id);
}

/* Sky Display List */
void initSkyDisplayList() {
    skyDisplayList = glGenLists(1);
    glNewList(skyDisplayList, GL_COMPILE);
    float size = 320.0f;

    // Draw
    for (int i = 1; i <= 6; i++) {
        glEnable(GL_TEXTURE_2D);

        if (i == 1) {
            glBindTexture(GL_TEXTURE_2D, skyFrontTex.id);
        } else if (i == 2) {
            glBindTexture(GL_TEXTURE_2D, skyBackTex.id);
        } else if (i == 3) {
            glBindTexture(GL_TEXTURE_2D, skyLeftTex.id);
        } else if (i == 4) {
            glBindTexture(GL_TEXTURE_2D, skyRightTex.id);
        } else if (i == 5) {
            glBindTexture(GL_TEXTURE_2D, skyTopTex.id);
        } else if (i == 6) {
            glBindTexture(GL_TEXTURE_2D, skyBottomTex.id);
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

        if (i == 1) {
            glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, skyFrontTex.data);
        } else if (i == 2) {
            glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, skyBackTex.data);
        } else if (i == 3) {
            glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, skyLeftTex.data);
        } else if (i == 4) {
            glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, skyRightTex.data);
        } else if (i == 5) {
            glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, skyTopTex.data);
        } else if (i == 6) {
            glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, skyBottomTex.data);
        }

        glPushMatrix();
        glScalef(size, size, size);

        if (i == 1) {
            glTranslatef(0.0f, 0.0f, 1.0f);
        } else if (i == 2) {
            glTranslatef(0.0f, 0.0f, -1.0f);
            glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
        } else if (i == 3) {
            glTranslatef(1.0f, 0.0f, 0.0f);
            glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
        } else if (i == 4) {
            glTranslatef(-1.0f, 0.0f, 0.0f);
            glRotatef(270.0f, 0.0f, 1.0f, 0.0f);
        } else if (i == 5) {
            glTranslatef(0.0f, 1.0f, 0.0f);
            glRotatef(270.0f, 1.0f, 0.0f, 0.0f);
        } else if (i == 6) {
            glTranslatef(0.0f, -1.0f, 0.0f);
            glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        }

        glBegin(GL_POLYGON);
        glTexCoord2i(1, 1); glVertex3f(1.0025f, 1.0025f, 0.0f);
        glTexCoord2i(1, 0); glVertex3f(1.0025f, -1.0025f, 0.0f);
        glTexCoord2i(0, 0); glVertex3f(-1.0025f, -1.0025f, 0.0f);
        glTexCoord2i(0, 1); glVertex3f(-1.0025f, 1.0025f, 0.0f);
        glEnd();
        glPopMatrix();

        glDisable(GL_TEXTURE_2D);
    }
    glEndList();
}

/* Car Body Display List */
void initCarBodyDisplayList() {
    carBodyModelDisplayList = glGenLists(1);
    glNewList(carBodyModelDisplayList, GL_COMPILE);
    // Load Model
    GLMmodel* carBodyModel = glmReadOBJ("Models/porsche.obj");
    glmUnitize(carBodyModel);
    glmFacetNormals(carBodyModel);
    glmVertexNormals(carBodyModel, 90.0f);
    glmScale(carBodyModel, 2.5);

    // Draw
    glPushMatrix();
    glColor3f(1.0f, 1.0f, 1.0f);
    glmDraw(carBodyModel, GLM_SMOOTH | GLM_MATERIAL);
    glPopMatrix();

    // Unload Model
    glmDelete(carBodyModel);
    glEndList();
}

/* Terrain Display List */
void initTerrainDisplayList() {
    terrainDisplayList = glGenLists(1);
    glNewList(terrainDisplayList, GL_COMPILE);
    // Draw (based on heightmap)
    terrain = loadTerrain("Textures/heightmap.bmp", 255.0f);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, floorTex.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, floorTex.data);

    glPushMatrix();
    float scale = 100.0f / max(terrain->width(), terrain->length());
    glTranslatef(0, -33, 0);
    glScalef(scale*-5, 0.25, scale * 5);
    glTranslatef(-(float)(terrain->width()) / 2, 0, -(float)(terrain->length()) / 2);

    glColor4f(0.0f, 0.6f, 0.0f, 1.0f);
    for (int z = 0; z < terrain->length() - 1; z++) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int x = 0; x < terrain->width(); x++) {
            Vec3f normal = terrain->getNormal(x, z);
            glNormal3f(normal[0], normal[1], normal[2]);
            glTexCoord2f(10 * x / (float)terrain->width(), 10 * z / (float)terrain->length()); glVertex3f(x, terrain->getHeight(x, z), z);

            normal = terrain->getNormal(x, z + 1);
            glNormal3f(normal[0], normal[1], normal[2]);
            glTexCoord2f(10 * x / (float)terrain->width(), 10 * (z + 1) / (float)terrain->length()); glVertex3f(x, terrain->getHeight(x, z + 1), z + 1);
        }
        glEnd();
    }
    glPopMatrix();

    glDisable(GL_TEXTURE_2D);
    delete terrain;
    glEndList();
}

/* Initialize Car Rim Display List */
void initCarRimDisplayList() {
    carRimModelDisplayList = glGenLists(1);
    glNewList(carRimModelDisplayList, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.1f, 0.1f, 0.1f);

    // Draw
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, carRimTex.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, carRimTex.data);

    glPushMatrix();
    glScalef(0.335f, 0.335f, 1.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.125f);
    glBegin(GL_POLYGON);
    for (float i = 360.0f; i > 0.0f; i -= 0.2f) {
        float angle = degreeToRadian(i);
        float x = sin(angle);
        float y = cos(angle);
        glTexCoord2f(sin(angle)*0.496f + 0.5f, cos(angle)*0.496f + 0.5f); glVertex2f(x, y);
    }
    glEnd();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.125f);
    glBegin(GL_POLYGON);
    for (float i = 0.0f; i < 360.0f; i += 0.2f) {
        float angle = degreeToRadian(i);
        float x = sin(angle);
        float y = cos(angle);
        glTexCoord2f(sin(angle)*0.496f + 0.5f, cos(angle)*0.496f + 0.5f); glVertex2f(x, y);
    }
    glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glBegin(GL_QUAD_STRIP);
    for (float i = 0.0f; i <= 365.0f; i += 0.2f) {
        float angle = degreeToRadian(i);
        float x = sin(angle);
        float y = cos(angle);
        glVertex3f(x, y, -0.125f);
        glVertex3f(x, y, 0.125f);
    }
    glEnd();
    glPopMatrix();
    glPopMatrix();
    glEndList();
}

/* Initialize All Display Lists */
void initAllDisplayLists() {
    initSkyDisplayList();
    initCarBodyDisplayList();
    initCarRimDisplayList();
    initTerrainDisplayList();
}

/* Draw Car Wheels */
void drawWheels() {
    float mat[16];
    for (int i = 0; i < vehicle->getNumWheels(); i++) {
        vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(mat);
        renderCylinder(mat);
    }
}

/* Draw Speedometer */
void drawSpeedometer() {
    float f = windowHeight < windowWidth ? windowHeight : windowWidth;
    float centerX = 110.0 / 768.0*f, centerY = 110.0 / 768.0*f;

    // Disable Lighting
    glDisable(GL_LIGHTING);

    // Draw
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0f, windowWidth, 0.0f, windowHeight, -1.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Background
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, speedometerBGTex.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 400, 400, 0, GL_RGB, GL_UNSIGNED_BYTE, speedometerBGTex.data);

    glBegin(GL_POLYGON);
    for (float i = 360.0f; i > 0.0f; i -= 0.2f) {
        float angle = degreeToRadian(i);
        float x = centerX + sin(angle)*(100.0f / 768.0*f);
        float y = centerY + cos(angle)*(100.0f / 768.0*f);
        glTexCoord2f(sin(angle)*0.497f + 0.5f, cos(angle)*0.497f + 0.5f); glVertex2f(x, y);
    }
    glEnd();

    glDisable(GL_TEXTURE_2D);

    // Needle
    Point2D left, top, right;
    left.x = -3.5f / 768.0*f;
    left.y = -15.0f / 768.0*f;
    top.x = 0.0f;
    top.y = 85.0f / 768.0*f;
    right.x = 3.5f / 768.0*f;
    right.y = -15.0f / 768.0*f;

    glPushMatrix();
    glTranslatef(centerX, centerY, 0.1f);

    float speed = abs(vehicle->getCurrentSpeedKmHour()) * 0.75f;
    if (speed > 130.0f) speed = 130.0f;

    glRotatef(127.5f - speed / 130.0f*275.5f, 0.0f, 0.0f, 1.0f);

    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_TRIANGLES);
    glVertex2f(right.x, right.y);
    glVertex2f(top.x, top.y);
    glVertex2f(left.x, left.y);
    glEnd();
    glPopMatrix();
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

/* Display function */
void display() {
    handleKeyboardInput();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Position the Camera
    gluLookAt(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], 0.0f, 1.0f, 0.0f);

    // Set up the light
    positionLight();

    // Draw All Objects
    glCallList(skyDisplayList);
    glCallList(terrainDisplayList);
    render(chassis);
    drawWheels();
    drawSpeedometer();

    glutSwapBuffers();
}

/* Idle function */
void idle() {
    world->stepSimulation(1.0 / 60.0);

    // Redisplay
    glutPostRedisplay();
}

/* Reshape function */
void reshape(int w, int h) {
    windowWidth = w;
    windowHeight = h;
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (h == 0) h = 1;
    gluPerspective(53.0f, w / (GLfloat)h, 1.0f, 5000.0f);

    glMatrixMode(GL_MODELVIEW);

    glutPostRedisplay();
}

/* Deinitialize All Objects */
void unloadAllObjects() {
    delete world;

    delete[] skyFrontTex.data;
    delete[] skyBackTex.data;
    delete[] skyLeftTex.data;
    delete[] skyRightTex.data;
    delete[] skyTopTex.data;
    delete[] skyBottomTex.data;
    delete[] speedometerBGTex.data;
    delete[] carRimTex.data;
    delete[] floorTex.data;

    delete vehicle;
    delete chassis;

    delete terrain;
    delete terrainShape;

    delete dispatcher;
    delete broadPhase;
    delete configuration;
    delete solver;
}

/* Main function */
int main(int argc, char* argv[]) {
    initializeGLUT(&argc, argv);

    glutCreateWindow("Driving Simulator");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyPressed);
    glutKeyboardUpFunc(keyUp);

    initializeOpenGL();
    loadAllTextures();
    initAllDisplayLists();
    initializePhysicsEngine();
    atexit(unloadAllObjects);

    glutMainLoop();

    return 0;
}