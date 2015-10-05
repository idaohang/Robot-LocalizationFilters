#include <random>
#include <iostream>
#include <GL/glut.h>
#include "KalmanFilter.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * Use four dimensions to denote the state variable
 * 1. x
 * 2. y
 * 3. dx
 * 4. dy
 **/
// 4 dimension, 2 measurements (x, y)
KalmanFilter<6, 2> kf;

struct Ball {
    float r;
    float x, y;
    float vx, vy;
    float ax, ay;
};

int counter = 0;
double t = 25.0 / 1000.0;

Ball ball;
Ball estimate;

std::default_random_engine generator;
std::normal_distribution<double> x_distribution(0.0,3.0);
std::normal_distribution<double> y_distribution(0.0,1.0);
std::normal_distribution<double> vx_distribution(0.0,2.0);
std::normal_distribution<double> vy_distribution(0.0,1.0);

/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) 
{

    switch(key) {
    case 'Q':
    case 'q':
    case  27:   // ESC
    exit(0);
    }
}
 
/* reshaped window */
void reshape(int width, int height) 
{

    GLfloat fieldOfView = 90.0f;
    glViewport (0, 0, (GLsizei) width, (GLsizei) height);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fieldOfView, (GLfloat) width/(GLfloat) height, 0.1, 500.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void grid(int limit, float width) 
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUADS);
    glColor3f(0.5, 0.5, 0.5);
    for (int i = -limit; i < limit; i++) 
    {
        for (int j = -limit; j < limit; j++) 
        {
            glVertex3f(i * width, 0, j * width);
            glVertex3f((i + 1) * width, 0, j * width);
            glVertex3f((i + 1) * width, 0, (j + 1) * width);
            glVertex3f(i * width, 0, (j + 1) * width);
        }
    }
    glEnd();
}
 
/* render the scene */
void display() 
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* render the scene here */
    gluLookAt(
            ball.x, 4.0, 10.0,
            ball.x, 0.0, 0.0,
            0.0, 1.0, 0.0
    );

    grid(200, 0.5);

    glPushMatrix();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glColor3f(0.96, 0.41, 0.04);
        glTranslatef(ball.x, ball.r / 2, ball.y);
        glutSolidSphere(ball.r, 16, 16);
    glPopMatrix();

    glPushMatrix();
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(1.0, 1.0, 1.0);
        glTranslatef(estimate.x, estimate.r / 2, estimate.y);
        glutSolidSphere(estimate.r, 8, 8);
    glPopMatrix();

    glFlush();
    glutSwapBuffers();
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

void initState()
{
    ball.r = 0.5;
    ball.x = 0; 
    ball.y = 0; 

    estimate.r = 0.6;
    estimate.x = 0;
    estimate.y = 0;

    Vector6d state, motion_vector;
    Matrix6d trans, cov;
    state << estimate.x, estimate.y, estimate.vx, estimate.vy, estimate.ax, estimate.ay;
    motion_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    trans << 
        1,  0,  t,  0,  0,  0,
        0,  1,  0,  t,  0,  0,
        0,  0,  1,  0,  t,  0,
        0,  0,  0,  1,  0,  t,
        0,  0,  0,  0,  1,  0,
        0,  0,  0,  0,  0,  1;
    cov <<
        1,  0,  0,  0,  0,  0,
        0,  1,  0,  0,  0,  0,
        0,  0,  1,  0,  0,  0,
        0,  0,  0,  1,  1,  0,
        0,  0,  0,  0,  1,  0,
        0,  0,  0,  0,  0,  1;
    Matrix<double, 2, 6> measurement;
    Matrix<double, 2, 2> noise;
    measurement << 
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
    noise <<
        25,  0,
        0,  25;

    // needed to be filled in
    kf.SetState(state);
    kf.SetStateTransition(trans);
    kf.SetStateCovariance(cov);
    kf.SetMoveVector(motion_vector);
    kf.SetMeasureExtraction(measurement);
    kf.SetMeasureCovariance(noise);
}

void update(int usused) 
{
    counter++;
    if (counter < 50)
    {
        ball.vx = 0; ball.ax = 0;
        ball.vy = 0; ball.ay = 0;
    }
    else if (counter == 50)
    {
        ball.vx = 10.0;     ball.ax = -2.0;
        ball.vy = 0.0;      ball.ay = 0.0;
    }
    else {
        if (ball.vx > 0) 
        {
            std::cout << "update" << std::endl;
            ball.vx += ball.ax * t;
            ball.vy += ball.ay * t;
            ball.x += ball.vx * t;
            ball.y += ball.vy * t;
        }
    }

    Vector2d measurement;
    measurement << 
        ball.x + x_distribution(generator), 
        ball.y + y_distribution(generator);

    kf.Update(measurement);
    Vector6d current = kf.GetCurrentState();
    estimate.x = current(0);
    estimate.y = current(1);
    estimate.vx = current(2);
    estimate.vy = current(3);
    estimate.ax = current(4);
    estimate.ay = current(5);
    std::cout << ball.vx << "\tvs.\t" << estimate.vx << std::endl;
    
    if (estimate.vx < 0) {
        std::cout << "RESET------------------------------------------------" << std::endl;
        current(2) = 0;
        current(4) = 0;
        kf.SetState(current);
    }
#if DEBUG
    std::cout << "ESTIMATE: " << estimate.x << ", " << estimate.y << std::endl;
#endif
    glutTimerFunc(25, update, 0);
}
 
/* initialize GLUT settings, register callbacks, enter main loop */
int main(int argc, char** argv) 
{

    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("KalmanFilter");

    // register glut call backs
    glutKeyboardFunc(keyboardDown);
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);  

    // initialize
    initGL(800, 600);
    initState();

    glutTimerFunc(25, update, 0);
    glutMainLoop();
    return 0;
}
