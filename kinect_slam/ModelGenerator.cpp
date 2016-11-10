#include <stdlib.h>
#include <stdio.h>
#include <GL\glew.h>
#include "FrameReader.h"

#include <zpr.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>





FrameReader* reader_pointer;


void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 27:
		exit(0);
		break;
	default:
		break;
	}
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 1.0, 30.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

}


void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glBegin(GL_POINTS);
	
	glPushMatrix();
	reader_pointer->displayOpenGL();
	glPopMatrix();
	glEnd();
	glFlush();
}

void initOpenGL(int argc, char** argv)
{

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(250, 250);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);
	glClearDepth(1.0f);         // Depth Buffer Setup
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -5);

	glShadeModel(GL_SMOOTH);       // Enable Smooth Shading
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);    // Black Background
	glClearDepth(1.0f);         // Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);       // Enables Depth Testing
	glDepthFunc(GL_LEQUAL);        // The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations









	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -5);

	glutDisplayFunc(display);



	
	GLenum err = glewInit();//guo-使用glgenbuffer初始化glew
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed, something is seriously wrong. */
		printf("Error: %s\n", glewGetErrorString(err));
	}



	zprInit();
	//zprSelectionFunc(display);     /* Selection mode draw function */
	//zprPickFunc(pick);              /* Pick event client callback   */

	//glutReshapeFunc(reshape);

	glutKeyboardFunc(keyboard);
	
}


int test_main(int argc, char** argv)
{
	
	//************************************


	if (argc < 4)
	{
		std::cout << "need more parameters" << std::endl;
		exit(-1);
	}

	std::string image_path = argv[1];
	std::string trajectory_path = argv[2];
	std::string camera_info_path = argv[3];

	FrameReader reader(image_path + "\\depth\\", image_path + "\\rgb\\", trajectory_path, camera_info_path);
	reader_pointer = &reader;


	std::thread generate_model_thread(&FrameReader::generate_model, &reader);
	initOpenGL(argc, argv);


	glutMainLoop();

	
	//openGL_thread.join();
	return 0;
}