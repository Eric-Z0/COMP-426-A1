#include "Boid.h"

int Boid::num_of_boids = 0;
//float Boid::speed = 0.0025f * 0.1f; // parameter for flockRunX()
float Boid::speed = 0.006f;			// parameter for flockRun()

Boid::Boid()
{
	boid_sn = num_of_boids;
	num_of_boids++;
}

Boid::Boid(float *vec_v0, float *vec_v1, float *vec_v2, float *col_vec)
{
	for (int i = 0; i < 2; i++)
	{
		tri_v0[i] = vec_v0[i];
		tri_v1[i] = vec_v1[i];
		tri_v2[i] = vec_v2[i];
		color[i] = col_vec[i];
	}

	position[0] = (tri_v0[0] + tri_v1[0] + tri_v2[0]) / float(3);
	position[1] = (tri_v0[1] + tri_v1[1] + tri_v2[1]) / float(3);

	boid_sn = num_of_boids;
	num_of_boids++;
}

