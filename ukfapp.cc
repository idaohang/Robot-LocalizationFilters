/***********************************************************
 * A Template for building OpenGL applications using GLUT
 *
 * Author: Perspective @ cprogramming.com
 * Date  : Jan, 2005
 *
 * Description: 
 * This code initializes an OpenGL ready window
 * using GLUT.  Some of the most common callbacks
 * are registered to empty or minimal functions.
 *
 * This code is intended to be a quick starting point
 * when building GLUT applications.
 *
 ***********************************************************/
 
//#include <windows.h>
#include <random>
#include <algorithm>
#include <iostream>
#include <GL/glu.h>
#include <GL/glut.h>

#include "UnscentedKalmanFilter.h"

using namespace filter;

std::random_device rng;
std::normal_distribution<> distribution(0, 50);

bool showArrow = false;

double theta = 0;
double velocity;

constexpr int MSEC = 1000 / 30;   // 30Hz
double arrowStart[3], arrowEnd[3];

double projection[16];
double modelview[16];
int viewport[16];

double w, h, d = 3000.0;

double robotPos[3] = {0 ,0, 2400 };

double ballRadius = 50;
double ballPos[3] = {0, 50, 0};
double ballVelocity[3] = { 0 , 0, 0 };
double ballAcceleration = -1000.0;
unsigned char ballCol[3] = {255, 180, 0};

typedef UnscentedKalmanFilter<4, 2>     FILTER;
FILTER ukf;

double measurePos[3] = { 0, 10, 0 };
double filterPos[3] = { 0, 10, 0 };

// ------------ unscented kalman filter parameters -----------------
float kappa = 0;
float alpha = 0.001;
float pnFactor = 100;
float covFactor = 100;
float noiseFactor = 10000;
float filterAcceleration = -0.5;

FILTER::VectorN trans(FILTER::VectorN state) 
{
    double velocity = state(2);
    double theta = state(3);

    // update position
    state(0) += velocity * cos(theta);
    state(1) += velocity * sin(theta);

    // update velocity
    velocity += filterAcceleration;
    velocity = std::max(0.0, velocity);
    
    state(2) = velocity;
    return state;
}
FILTER::VectorM stm(FILTER::VectorN state) 
{
    FILTER::VectorM v;
    v << state(0), state(1);
    return v;
}

void initUnscentedKalmanFilter() {
    ukf.SetStateTransition(trans);
    ukf.SetMeasureExtraction(stm);

    FILTER::MatrixNN cov(FILTER::MatrixNN::Identity() * covFactor);
    cov << 
        500, 0, 0, 0,
        0, 500, 0, 0,
        0, 0, 5000, 0,
        0, 0, 0, 10
    ;
    ukf.SetStateCovariance(cov);

    FILTER::MatrixMM noise(FILTER::MatrixMM::Identity() * noiseFactor);
    noise << 
        20000, 0,
        0, 1000
    ;
    ukf.SetMeasureCovariance(noise);

    FILTER::MatrixNN pn = FILTER::MatrixNN::Identity() * pnFactor;
    ukf.SetProcessNoise(pn);

    ukf.SetUnscentedParameters(kappa, alpha);

    std::cout << "kappa:\t" << kappa << std::endl;
    std::cout << "alpha:\t" << alpha << std::endl;
    std::cout << "cov:\t" << covFactor << std::endl;
    std::cout << "noise:\t" << noiseFactor << std::endl;
    std::cout << "pn:\t" << pnFactor << std::endl;
    std::cout << "accel:\t" << filterAcceleration << std::endl;
    std::cout << "----------------------------" << std::endl;
}
 
/* process menu option 'op' */
void menu(int op) {

    switch(op) {
        case 'Q':
        case 'q':
            exit(0);
    }
}
 
/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {

    switch(key) {
        case 'Q':
        case 'q':
        case  27:   // ESC
            exit(0);
        case 'k':
            kappa += 0.01;
            break;
        case 'K':
            kappa -= 0.01;
            break;
        case 'a':
            alpha += 0.001;
            break;
        case 'A':
            alpha -= 0.001;
            break;
        case 'c':
            covFactor *= 10;
            break;
        case 'C':
            covFactor /= 10;
            break;
        case 'n':
            noiseFactor *= 10;
            break;
        case 'N':
            noiseFactor /= 10;
            break;
        case 'p':
            pnFactor *= 10;
            break;
        case 'P':
            pnFactor /= 10;
            break;
        case 'e':
            filterAcceleration += 0.1;
            break;
        case 'E':
            filterAcceleration -= 0.1;
            break;
    }
}
 
/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {
 
}
 
/* executed when a special key is pressed */
void keyboardSpecialDown(int k, int x, int y) {
 
}
 
/* executed when a special key is released */
void keyboardSpecialUp(int k, int x, int y) {
 
}
 
/* reshaped window */
void reshape(int width, int height) {

    w = width;
    h = height;

    GLfloat fieldOfView = 90.0f;
    glViewport (0, 0, (GLsizei) width, (GLsizei) height);
    glGetIntegerv(GL_VIEWPORT, viewport);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fieldOfView, (GLfloat) width/(GLfloat) height, 1.0, 3000.0);

    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
 
/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
    gluUnProject(x, h - y, 1.0, modelview, projection, viewport, &arrowStart[0], &arrowStart[1], &arrowStart[2]);
    if (!showArrow) {
        ballPos[0] = arrowStart[0];
        ballPos[2] = arrowStart[2];
        velocity = 0;
    }
    else {
        double xdiff = (arrowStart[0] - ballPos[0]);
        double ydiff = (arrowStart[2] - ballPos[2]);
        velocity = std::sqrt(xdiff * xdiff + ydiff * ydiff);
        theta = std::atan2(ydiff, xdiff);
        ballVelocity[0] = velocity * cos(theta);
        ballVelocity[2] = velocity * sin(theta);
    }
    showArrow = false;
}
 
/* executed when the mouse moves to position ('x', 'y') */
void mouseMotion(int x, int y) {
    showArrow = true;
    gluUnProject(x, h - y, 1.0, modelview, projection, viewport, &arrowEnd[0], &arrowEnd[1], &arrowEnd[2]);
}
 
void drawGround(float width, float height) {
    glColor3ub(50, 144, 50);
    glBegin(GL_QUADS);
        glVertex3f( width / 2, 0.0, -height / 2);
        glVertex3f( width / 2, 0.0,  height / 2);
        glVertex3f(-width / 2, 0.0,  height / 2);
        glVertex3f(-width / 2, 0.0, -height / 2);
    glEnd();
}

void drawArrow() {
    glColor3ub(255, 255, 255);
    glBegin(GL_LINES);
        glVertex3dv(arrowStart);
        glVertex3dv(arrowEnd);
    glEnd();
}

void drawRobot() {
    glColor3ub(255, 255, 255);
    glPushMatrix();
        glTranslatef(robotPos[0], robotPos[1], robotPos[2]);
        glutSolidCone(100, 400, 16, 16);
    glPopMatrix();
}

void drawBall() {
    // real ball
    glColor3ubv(ballCol);
    glPushMatrix();
        glTranslatef(ballPos[0], ballPos[1], ballPos[2]);
        glutSolidSphere(ballRadius, 16, 16);
    glPopMatrix();

    // measurement
    glColor3ub(255, 0, 0);
    glPushMatrix();
        glTranslatef(measurePos[0], measurePos[1], measurePos[2]);
        glutWireSphere(ballRadius, 16, 16);
    glPopMatrix();

    // estimated
    glColor3ub(0, 255, 255);
    glPushMatrix();
        glTranslatef(filterPos[0], filterPos[1], filterPos[2]);
        glutWireSphere(ballRadius, 16, 16);
    glPopMatrix();
}

/* render the scene */
void draw() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(0.0, d, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

    /* render the scene here */
    drawGround(3000.0, 5000.0);

    if (showArrow) drawArrow();
    
    drawRobot();
    drawBall();

    glFlush();
    glutSwapBuffers();
}
 
/* executed when program is idle */
void idle() { 
    glutPostRedisplay();
}
 
/* initialize OpenGL settings */
void initGL(int width, int height) {

    reshape(width, height);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
}

void update(int msec) {
    // update ball pos, velocity and acceleration
    ballPos[0] += ballVelocity[0] * (msec) / 1000.0;
    ballPos[2] += ballVelocity[2] * (msec) / 1000.0;

    velocity += ballAcceleration * (msec) / 1000.0;
    velocity = std::max(0.0, velocity);
    ballVelocity[0] = velocity * cos(theta);
    ballVelocity[2] = velocity * sin(theta);

    // ------------------------------------
    measurePos[0] = ballPos[0] + distribution(rng);
    measurePos[2] = ballPos[2] + distribution(rng);

    FILTER::VectorM measurement;
    measurement << measurePos[0], measurePos[2];

    ukf.Update(measurement);
    FILTER::VectorN state = ukf.GetCurrentState();
    filterPos[0] = state(0);
    filterPos[2] = state(1);

    // ------------------------------------

    initUnscentedKalmanFilter();
    glutTimerFunc(MSEC, update, MSEC);
}
 
/* initialize GLUT settings, register callbacks, enter main loop */
int main(int argc, char** argv) {

    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Unscented Kalman Filter Demo");

    // register glut call backs
    glutKeyboardFunc(keyboardDown);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(keyboardSpecialDown);
    glutSpecialUpFunc(keyboardSpecialUp);
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMotion);
    glutReshapeFunc(reshape);
    glutDisplayFunc(draw);  
    glutIdleFunc(idle);
    glutIgnoreKeyRepeat(true); // ignore keys held down

    glutTimerFunc(MSEC, update, MSEC);

    // create a sub menu 
    int subMenu = glutCreateMenu(menu);
    glutAddMenuEntry("Do nothing", 0);
    glutAddMenuEntry("Really Quit", 'q');

    // create main "right click" menu
    glutCreateMenu(menu);
    glutAddSubMenu("Sub Menu", subMenu);
    glutAddMenuEntry("Quit", 'q');
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    initGL(800, 600);
    initUnscentedKalmanFilter();

    FILTER::VectorN state;
    state << 0, 0, 0, 0;
    ukf.SetState(state);

    glutMainLoop();
    return 0;
}
