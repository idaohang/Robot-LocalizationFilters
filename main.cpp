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
 
/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {

    switch(key) {
    case 'Q':
    case 'q':
    case  27:   // ESC
    exit(0);
    }
}
 
/* reshaped window */
void reshape(int width, int height) {

    GLfloat fieldOfView = 90.0f;
    glViewport (0, 0, (GLsizei) width, (GLsizei) height);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fieldOfView, (GLfloat) width/(GLfloat) height, 0.1, 500.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
 
/* render the scene */
void display() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* render the scene here */

    glFlush();
    glutSwapBuffers();
}
 
/* initialize OpenGL settings */
void initGL(int width, int height) {

    reshape(width, height);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
}

void update(int usused) 
{
    // kf.Predict();
    // kf.Update();
}
 
/* initialize GLUT settings, register callbacks, enter main loop */
int main(int argc, char** argv) {

    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("GLUT Template");

    // register glut call backs
    glutKeyboardFunc(keyboardDown);
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);  
    initGL(800, 600);

    glutTimerFunc(25, update, 0);

    glutMainLoop();
    return 0;
}
