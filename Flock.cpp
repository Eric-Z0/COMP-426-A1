#include "Flock.h"

// Initialization for class attributes
int Flock::num_of_flocks = 0;
int Flock::flock_counter = 0;
bool Flock::flocksTimeToUpdate = false;

Flock::Flock()
{
	flock_sn = num_of_flocks;
	num_of_flocks++;
}

void Flock::setFlockColor(float color_set[])
{
	flock_color[0] = color_set[0];
	flock_color[1] = color_set[1];
	flock_color[2] = color_set[2];
}

void Flock::addBoid(Boid &b)
{
	boids.push_back(b);
}

void Flock::calFlockCenter()
{
	float newFlockCenter[2] = { 0.0f, 0.0f };

	for (int bo_index = 0; bo_index < boids.size(); bo_index++)
	{
		newFlockCenter[0] += boids[bo_index].position[0];
		newFlockCenter[1] += boids[bo_index].position[1];
	}

	flockCenter[0] = newFlockCenter[0] / boids.size();
	flockCenter[1] = newFlockCenter[1] / boids.size();
}

bool Flock::boidIsAroundFlockCenter(int boid_id)
{
	if (boids[boid_id].position[0] <= (0.3f + flockCenter[0]) && boids[boid_id].position[0] >= (-0.3f + flockCenter[0]))
	{
		if (boids[boid_id].position[1] <= (0.3f + flockCenter[1]) && boids[boid_id].position[1] >= (-0.3f + flockCenter[1]))
		{
			return true;
		}
		else
			return false;
	}
	else
		return false;
}








