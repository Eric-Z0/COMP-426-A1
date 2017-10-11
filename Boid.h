#pragma once
#include <math.h>
#include <random>
#include <iostream>

class Boid {

public:
	static float speed;
	static int num_of_boids;
	int boid_sn;
	bool posUpdated = false;
	bool collision = false;

	float color[3] = { 0.0f, 0.0f, 0.0f };
	float tri_v0[3] = { 0.0f, 0.0f, 0.0f };
	float tri_v1[3] = { 0.0f, 0.0f, 0.0f };
	float tri_v2[3] = { 0.0f, 0.0f, 0.0f };
	float position[3] = { 0.0f, 0.0f };
	float velocity[3] = { 0.0f, 0.0f };
	float rotate_angle = 0.0f;

	Boid();
	Boid(float *vec_v0, float *vec_v1, float *vec_v2, float *col_vec);

};
