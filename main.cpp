#include <iostream>
#include <GL/glut.h>
#include "KalmanFilter.h"

/**
 * Use four dimensions to denote the state variable
 * 1. x
 * 2. y
 * 3. dx
 * 4. dy
 **/
KalmanFilter<4> kf;

struct Ball {
    float r;
    float x, y;
    float vx, vy;
};

Ball ball;
Ball estimate;

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
            0.0, 4.0, 10.0,
            0.0, 0.0, 0.0,
            0.0, 1.0, 0.0
    );

    grid(30, 0.5);

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
    ball.x = 0; ball.vx = 5.0;
    ball.y = 0; ball.vy = 0.0;

    estimate.r = 0.6;
    estimate.x = 0; estimate.vx = 5.0;
    estimate.y = 0; estimate.vy = 0.0;

    // needed to be filled in
    /*
    kf.SetState();
    kf.SetTransitionMatrix();
    kf.SetUncertaintyCovariance();
    kf.SetMotionVector();
    kf.SetMeasureMatrix();
    kf.SetMeasureNoise();
    */
}

void update(int usused) 
{
    ball.x += ball.vx * 25.0 / 1000.0;
    ball.y += ball.vy * 25.0 / 1000.0;

    kf.Predict();
    // kf.Update();
    
    // std::cout << ball.x << ", " << ball.y << std::endl;
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
