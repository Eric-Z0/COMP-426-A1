#include "FlockLibrary.h"
#include <string>
#include <GL\glew.h>
#include <GL\freeglut.h>

void drawBoids(void) {

	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.961f, 1.0f, 0.98f, 1);

	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		glColor3f(flocks[fl_index].flock_color[0], flocks[fl_index].flock_color[1], flocks[fl_index].flock_color[2]);

		for (int bo_index = 0; bo_index < flocks[fl_index].boids.size(); bo_index++)
		{
			glMatrixMode(GL_MODELVIEW); // switch to the drawing perspective
			glLoadIdentity();			// reset the drwaing perspective
			glPushMatrix();

			glTranslatef((flocks[fl_index].boids[bo_index].position[0]), (flocks[fl_index].boids[bo_index].position[1]), 0.0f);

			glRotatef(flocks[fl_index].boids[bo_index].rotate_angle, 0.0f, 0.0f, 1.0f);

			glTranslatef(flocks[fl_index].boids[bo_index].position[0]*-1, flocks[fl_index].boids[bo_index].position[1]*-1, 0.0f);

			glBegin(GL_TRIANGLES);

			glVertex2f(flocks[fl_index].boids[bo_index].tri_v0[0], flocks[fl_index].boids[bo_index].tri_v0[1]);
			glVertex2f(flocks[fl_index].boids[bo_index].tri_v1[0], flocks[fl_index].boids[bo_index].tri_v1[1]);
			glVertex2f(flocks[fl_index].boids[bo_index].tri_v2[0], flocks[fl_index].boids[bo_index].tri_v2[1]);

			glEnd();

			glPopMatrix();

		}
	}

	glutSwapBuffers();
	glutPostRedisplay();

}


int main(int argc, char** argv) 
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE);
	glutInitWindowSize(800, 600);                   
	glutInitWindowPosition(0, 0);					
	glutCreateWindow("COMP 426 Assignment 1");

	generateFlocks(num_of_flocks, num_of_boids);
	
	glutDisplayFunc(drawBoids);
	
	flockRun();	// Solution one: conditional variable used
	//flockRunX();	// Solution two: no lock usage

	int graphicsUpdateT = 0;
	int flocksUpdateT = 0;

	while (true)
	{
		auto current_time = GetTickCount64();

		if (current_time >= graphicsUpdateT)
		{
			glutMainLoopEvent();
		}

		if (current_time >= flocksUpdateT)
		{
			Flock::flocksTimeToUpdate = true;
		}
	}
}
