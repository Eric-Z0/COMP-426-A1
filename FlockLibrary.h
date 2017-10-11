#pragma once
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "Flock.h"


int num_of_boids = 15;
int num_of_flocks = 6;
std::vector<Flock> flocks;

// flock radius for flock collison detection
static float const flockRadius = 0.54f;

// Parameters for a triangle shaped boid
static float const tri_base = 0.024f;
static float const tri_hei = 0.036f;

// force weight
static float const sep_weight = 1.0f;
static float const ali_weight = 1.0f;
static float const coh_weight = 1.0f;

// Four coordinates for border detection
float static x_min = float(-1 + tri_hei);
float static x_max = float(1 - tri_hei);
float static y_min = float(-1 + tri_hei);
float static y_max = float(1 - tri_hei);

// Parameters for sever different colors
// 1. purple 2. red 3. green 4. yellow 5. orange 6. pink 7. aqua
float const color_set[21] = {
	0.5f, 0.0f, 1.0f,
	1.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f,
	1.0f, 1.0f, 0.0f,
	1.0f, 0.5f, 0.0f,
	1.0f, 0.4f, 0.7f,
	0.5f, 1.0f, 0.8f
};

// mutex for boidMove
std::mutex mutex_boidMove;
std::mutex mutex_flockMove;
std::unique_lock<std::mutex> lock;

// conditional variable let flock thread communicate with boid thread
std::condition_variable cond;


float calDistance(float vec_1[], float vec_2[])
{
	float distance = 0;

	float r_x = vec_1[0] - vec_2[0];
	float r_y = vec_1[1] - vec_2[1];

	distance = pow((pow(r_x, 2) + pow(r_y, 2)), 0.5);

	return distance;
}

// Extra: calculate the angle for a triangle shaped boid to turn
void calculateRoatateAngle(Boid boid, float &roatateAngle)
{
	double pi = atan(1.0) * 4;

	float vec_assit[2] = { 0.0f, 0.0f };
	float new_v2[2] = { 0.0f, 0.0f };

	vec_assit[0] = (boid.tri_v0[0] + boid.tri_v1[0]) / 2;
	vec_assit[1] = (boid.tri_v0[1] + boid.tri_v1[1]) / 2;

	new_v2[0] = boid.tri_v2[0] - vec_assit[0];
	new_v2[1] = boid.tri_v2[1] - vec_assit[1];

	float product_vec = new_v2[0] * boid.velocity[1] - new_v2[1] * boid.velocity[0];
	float factor = 1;

	if (product_vec < 0)
		factor = -1;

	float dot_product = (new_v2[0] * boid.velocity[0]) + (new_v2[1] * boid.velocity[1]);

	float magnitude1 = pow((pow(new_v2[0], 2) + pow(new_v2[1], 2)), 0.5);
	float magnitude2 = pow((pow(boid.velocity[0], 2) + pow(boid.velocity[1], 2)), 0.5);

	float angle_rad = factor * acos(dot_product / (magnitude1 * magnitude2));
	roatateAngle = angle_rad / pi * 180;
}

void generateFlocks(int num_of_flock, int num_of_boid)
{
	// The modification value to randomly put a standard boid to a new place on the screen
	float pos_mod_v = 0.0f;
	// A float array to store the modification value for x,y coordinates of a boid
	float posMod[3] = { 0.0f, 0.0f };

	// Create random integer number generator
	std::mt19937 Rng;
	Rng.seed(std::random_device()());
	std::uniform_int_distribution<std::mt19937::result_type> random_angle(0, 360);

	for (int i = 0; i < num_of_flock; i++)
	{
		Flock flock;

		float flock_color[3] = { color_set[i * 3], color_set[i * 3 + 1], color_set[i * 3 + 2] };
		flock.setFlockColor(flock_color);

		for (int j = 0; j < num_of_boid; j++)
		{
			float boidO_vo[3] = { 0.0f, 0.0f, 0.0f };
			float boidO_v1[3] = { tri_base, 0.0f, 0.0f };
			float boidO_v2[3] = { tri_base / 2, tri_hei, 0.0f };

			for (int k = 0; k < 2; k++)
			{
				do {
					pos_mod_v = sin(random_angle(Rng)) * 0.8; // need to modify the value
				} while (pos_mod_v >(1 - tri_hei) || pos_mod_v < (-1 + tri_hei));

				posMod[k] = pos_mod_v;
			}

			for (int g = 0; g < 2; g++)
			{
				boidO_vo[g] += posMod[g];
				boidO_v1[g] += posMod[g];
				boidO_v2[g] += posMod[g];
			}

			Boid boid(boidO_vo, boidO_v1, boidO_v2, flock_color);
			flock.addBoid(boid);
		}

		flocks.push_back(flock);
	}

	// Randomly set a target postion for each flock
	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		flocks[fl_index].flockTargetPos[0] = sin(random_angle(Rng)) * 0.85f;
		flocks[fl_index].flockTargetPos[1] = cos(random_angle(Rng)) * 0.85f;
	}
}

void cohesion(Boid &boid, Flock flock, float coh_vec[])
{
	float vector_x = 0.0, vector_y = 0.0;

	for (int boid_id = 0; boid_id < flock.boids.size(); boid_id++)
	{
		if (boid.boid_sn != flock.boids[boid_id].boid_sn)
		{
			vector_x += flock.boids[boid_id].position[0];
			vector_y += flock.boids[boid_id].position[1];
		}
	}

	vector_x /= (flock.boids.size() - 1);
	vector_y /= (flock.boids.size() - 1);

	vector_x = (vector_x - boid.position[0]) / 100;
	vector_y = (vector_y - boid.position[1]) / 100;

	coh_vec[0] = vector_x;
	coh_vec[1] = vector_y;

}

void separation(Boid &boid, Flock flock, float spe_vec[])
{
	float vector_x = 0.0, vector_y = 0.0;

	for (int boid_id = 0; boid_id < flock.boids.size(); boid_id++)
	{
		if (boid.boid_sn != flock.boids[boid_id].boid_sn)
		{
			if (calDistance(boid.position, flock.boids[boid_id].position) < 0.05) // previous value: 0.025
			{
				vector_x = vector_x - (flock.boids[boid_id].position[0] - boid.position[0]);
				vector_y = vector_y - (flock.boids[boid_id].position[1] - boid.position[1]);
			}
		}
	}

	spe_vec[0] = vector_x;
	spe_vec[1] = vector_y;

}

void alignment(Boid &boid, Flock flock, float ali_vec[])
{
	float vector_x = 0.0, vector_y = 0.0;

	for (int boid_id = 0; boid_id < flock.boids.size(); boid_id++)
	{
		if (boid.boid_sn != flock.boids[boid_id].boid_sn)
		{
			vector_x += flock.boids[boid_id].velocity[0];
			vector_y += flock.boids[boid_id].velocity[1];
		}
	}

	vector_x /= (flock.boids.size() - 1);
	vector_y /= (flock.boids.size() - 1);

	vector_x = (vector_x - boid.velocity[0]) / 8;
	vector_y = (vector_y - boid.velocity[1]) / 8;

	ali_vec[0] = vector_x;
	ali_vec[1] = vector_y;

}


void goal(Boid &boid, Flock &flock, float goal_vec[])
{
	int arriveCtr = 0;

	for (int i = 0; i < flock.boids.size(); i++)
	{
		if (flock.boids[i].position[0] <= (0.15f + flock.flockTargetPos[0]) && flock.boids[i].position[0] >= (-0.15f + flock.flockTargetPos[0]))
		{
			if (flock.boids[i].position[1] <= (0.15f + flock.flockTargetPos[1]) && flock.boids[i].position[1] >= (-0.15f + flock.flockTargetPos[1]))
			{
				arriveCtr++;
			}
		}
	}

	if (arriveCtr == flock.boids.size())
	{
		//std::cout << "flock " << flock.flock_sn << " reached the target position!" << std::endl;
		// randomly generate the direction for a boid
		std::mt19937 Rng;
		Rng.seed(std::random_device()());
		std::uniform_int_distribution<std::mt19937::result_type> random_angle(0, 360);

		flock.flockTargetPos[0] = sin(random_angle(Rng)) * 0.85f;
		flock.flockTargetPos[1] = cos(random_angle(Rng)) * 0.85f;
	}
	else
	{
		for (int k = 0; k < 2; k++)
		{
			goal_vec[k] = (flock.flockTargetPos[k] - boid.position[k]) / 100;
		}
	}
}


void borderTest(Boid &boid, float rebound_v[])
{
	if (boid.position[0] < x_min)
		rebound_v[0] = Boid::speed; // I increase the value 2 times greater than before
	else if (boid.position[0] > x_max)
		rebound_v[0] = -1 * Boid::speed;

	if (boid.position[1] < y_min)
		rebound_v[1] = Boid::speed;
	else if (boid.position[1] > y_max)
		rebound_v[1] = -1 * Boid::speed;
}

void speedLimitTest(Boid &boid)
{
	float magnitude = pow((pow(boid.velocity[0], 2) + pow(boid.velocity[1], 2)), 0.5);

	if (magnitude > Boid::speed)
	{
		boid.velocity[0] = boid.velocity[0] / magnitude * Boid::speed;
		boid.velocity[1] = boid.velocity[1] / magnitude * Boid::speed;
	}
}

// Extra: Flock collision detection, on flock level
void flockCollisionDetectFL(Flock &flock, std::vector<Flock> flocks)
{
	int neighborCount = 0;
	float vector_x = 0.0, vector_y = 0.0;

	// Calculate the center of each flock
	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		flocks[fl_index].calFlockCenter();
	}

	// Check it all the boids in the flock has been in the range of flock center
	for (int bo_index = 0; bo_index < flock.boids.size(); bo_index++)
	{
		if (!flock.boidIsAroundFlockCenter(bo_index))
			return;
	}

	// Basically the separation rule applied to flock
	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		if (flock.flock_sn != flocks[fl_index].flock_sn)
		{
			if (calDistance(flock.flockCenter, flocks[fl_index].flockCenter) < flockRadius)
			{
				vector_x += flock.flockCenter[0] - flocks[fl_index].flockCenter[0];
				vector_y += flock.flockCenter[1] - flocks[fl_index].flockCenter[1];
				neighborCount++;
			}
		}
	}

	if (neighborCount != 0)
	{
		vector_x = vector_x / neighborCount;
		vector_y = vector_y / neighborCount;

		flock.flockCollisionPrevent[0] = vector_x / 100;
		flock.flockCollisionPrevent[1] = vector_y / 100;

		flock.collionDetected = true;
	}
	else
		flock.collionDetected = false;
}

// Extra: flock collision detection, on boid level
void flockCollisionDetectBL(Boid &boid, Flock &flock, std::vector<Flock>& flocks, float fcp_vec[])
{
	float vector_x = 0.0, vector_y = 0.0;

	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		if (flock.flock_sn != flocks[fl_index].flock_sn)
		{
			for (int bo_index = 0; bo_index < flocks[fl_index].boids.size(); bo_index++)
			{
				if (calDistance(boid.position, flocks[fl_index].boids[bo_index].position) < 0.06)
				{
					vector_x += (boid.position[0] - flocks[fl_index].boids[bo_index].position[0]);
					vector_y += (boid.position[1] - flocks[fl_index].boids[bo_index].position[1]);

					boid.collision = true;
				}
			}
		}
	}

	fcp_vec[0] = vector_x;
	fcp_vec[1] = vector_y;
}

void FlockCollisionPrevention(Boid &boid, Flock &flock, float vec_fcp[], float vec_sep[], float vec_ali[], float vec_coh[], float vec_goal[], float vec_border[])
{
	if (boid.collision)
	{
		boid.velocity[0] += vec_fcp[0];
		boid.velocity[1] += vec_fcp[1];
		boid.collision = false;
	}
	if (flock.collionDetected)
	{
		flock.flockGoalWeight = 0.0f;
		boid.velocity[0] += vec_sep[0] * sep_weight + vec_ali[0] * ali_weight + vec_coh[0] * coh_weight + vec_border[0] + vec_goal[0] * flock.flockGoalWeight;
		boid.velocity[1] += vec_sep[1] * sep_weight + vec_ali[1] * ali_weight + vec_coh[1] * coh_weight + vec_border[1] + vec_goal[1] * flock.flockGoalWeight;

		boid.velocity[0] += flock.flockCollisionPrevent[0];
		boid.velocity[1] += flock.flockCollisionPrevent[1];

		flock.collionDetected = false;
	}
	else
	{
		// Update the velocity of a boid
		boid.velocity[0] += vec_sep[0] * sep_weight + vec_ali[0] * ali_weight + vec_coh[0] * coh_weight + vec_border[0] + vec_goal[0] * flock.flockGoalWeight;
		boid.velocity[1] += vec_sep[1] * sep_weight + vec_ali[1] * ali_weight + vec_coh[1] * coh_weight + vec_border[1] + vec_goal[1] * flock.flockGoalWeight;
		flock.flockGoalWeight = 1.0f;
	}
}


// The boid thread (Solution one: using mutex and conditional variable)
void boidMove(Boid &boid, Flock &flock, int &counter)
{
	std::unique_lock<std::mutex> lock(mutex_flockMove);

	float vec_sep[2] = { 0.0f, 0.0f };
	float vec_ali[2] = { 0.0f, 0.0f };
	float vec_coh[2] = { 0.0f, 0.0f };
	float vec_border[2] = { 0.0f, 0.0f };
	float vec_goal[2] = { 0.0f, 0.0f };
	float vec_fcp[2] = { 0.0f, 0.0f };

	while (true)
	{
		mutex_boidMove.lock();										// mutex ensure synchronization

		separation(boid, flock, vec_sep);
		alignment(boid, flock, vec_ali);
		cohesion(boid, flock, vec_coh);
		borderTest(boid, vec_border);
		goal(boid, flock, vec_goal);

		flockCollisionDetectBL(boid, flock, flocks, vec_fcp);		// detect flock collision on flock level
		flockCollisionDetectFL(flock, flocks);						// detect flock collision on boid level

		mutex_boidMove.unlock();

		
		FlockCollisionPrevention(boid, flock, vec_fcp, vec_sep, vec_ali, vec_coh, vec_goal, vec_border);
		// Update the velocity of a boid
		//boid.velocity[0] += vec_sep[0] * sep_weight + vec_ali[0] * ali_weight + vec_coh[0] * coh_weight + vec_border[0] + vec_goal[0];
		//boid.velocity[1] += vec_sep[1] * sep_weight + vec_ali[1] * ali_weight + vec_coh[1] * coh_weight + vec_border[1] + vec_goal[1];


		// If the velocity is too fast then normalize it
		speedLimitTest(boid);

		// Update the coordinations of three vertices
		for (int i = 0; i < 2; i++)
		{
			boid.tri_v0[i] += boid.velocity[i];
			boid.tri_v1[i] += boid.velocity[i];
			boid.tri_v2[i] += boid.velocity[i];
		}

		// Update the centroid position
		for (int i = 0; i < 2; i++)
		{
			boid.position[i] += boid.velocity[i];
		}

		calculateRoatateAngle(boid, boid.rotate_angle);

		mutex_boidMove.lock();
		counter++;
		mutex_boidMove.unlock();

		cond.wait(lock);
	}
}

// The flock thread (Solution one: using mutex and conditional variable)
void flockMove(Flock &flock)
{
	// spawn boid threads
	for (int i = 0; i < flock.boids.size(); i++)
	{
		new std::thread(boidMove, std::ref(flock.boids[i]), std::ref(flock), std::ref(Flock::flock_counter));
	}

	while (true)
	{
		if (Flock::flock_counter == Flock::num_of_flocks * flock.boids.size())
		{
			while (!Flock::flocksTimeToUpdate)
			{
				std::this_thread::yield();
			}

			std::unique_lock<std::mutex> lock(mutex_flockMove);

			// reset the counter as zero
			Flock::flock_counter = 0;
			// reset the flocksTimeToUpdate as false
			Flock::flocksTimeToUpdate = false;

			cond.notify_all();
		}
		else
			std::this_thread::yield();
	}
}

// The function to spawn flock threads (Solution one: using mutex and conditional variable)
void flockRun()
{
	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		new std::thread(flockMove, std::ref(flocks[fl_index]));
	}

}


// New direction computation of a boid (this part will be done sequentially)
void calNewDirection(Boid &boid, Flock &flock)
{
	float vec_sep[3] = { 0.0f, 0.0f };
	float vec_ali[3] = { 0.0f, 0.0f };
	float vec_coh[3] = { 0.0f, 0.0f };
	float vec_border[3] = { 0.0f, 0.0f };
	float vec_goal[3] = { 0.0f, 0.0f };
	float vec_fcp[2] = { 0.0f, 0.0f };

	separation(boid, flock, vec_sep);
	alignment(boid, flock, vec_ali);
	cohesion(boid, flock, vec_coh);
	borderTest(boid, vec_border);
	goal(boid, flock, vec_goal);

	flockCollisionDetectBL(boid, flock, flocks, vec_fcp);	// detect flock collision on flock level
	flockCollisionDetectFL(flock, flocks);					// detect flock collision on boid level

	// Update the direction / velocity
	if (boid.collision)
	{
		boid.velocity[0] += vec_fcp[0];
		boid.velocity[1] += vec_fcp[1];
		boid.collision = false;
	}

	if (flock.collionDetected)
	{
		boid.velocity[0] += flock.flockCollisionPrevent[0];
		boid.velocity[1] += flock.flockCollisionPrevent[1];

		flock.dirAjustmentCtr--;

		if (flock.dirAjustmentCtr == 0)
		{
			flock.dirAjustmentCtr = 2;
			flock.collionDetected = false;
		}
			
		flock.flockGoalWeight = 0.0f;
	}
	else
	{
		// Update the velocity of a boid
		boid.velocity[0] += vec_sep[0] * sep_weight + vec_ali[0] * ali_weight + vec_coh[0] * coh_weight + vec_border[0] + vec_goal[0] * flock.flockGoalWeight;
		boid.velocity[1] += vec_sep[1] * sep_weight + vec_ali[1] * ali_weight + vec_coh[1] * coh_weight + vec_border[1] + vec_goal[1] * flock.flockGoalWeight;
		
		flock.flockGoalWeight = 1.0f;
	}

}

// The boid thread (Solution Two: using while loop and boolean variables)
void boidMoveX(Boid &boid)
{
	while (true)
	{
		if (boid.posUpdated)
			std::this_thread::yield();

		// if speed is too fast then normalize the velocity 
		speedLimitTest(boid);

		// update the coordinatino of three vertices
		for (int i = 0; i < 2; i++)
		{
			boid.tri_v0[i] += boid.velocity[i];
			boid.tri_v1[i] += boid.velocity[i];
			boid.tri_v2[i] += boid.velocity[i];
		}

		// update the centroid position
		for (int i = 0; i < 2; i++)
		{
			boid.position[i] += boid.velocity[i];
		}

		calculateRoatateAngle(boid, boid.rotate_angle);

		boid.posUpdated = true;
	}
}

// The flock thread (Solution Two: using while loop and boolean variables)
void flockMoveX(Flock &flock)
{
	// spawn boid threads
	for (int i = 0; i < flock.boids.size(); i++)
	{
		new std::thread(boidMoveX, std::ref(flock.boids[i]));
	}

	while (true)
	{
		while (!flock.allBoidsPosUpdated)
		{
			for (int boid_id = 0; boid_id < flock.boids.size(); boid_id++)
			{
				if (!flock.boids[boid_id].posUpdated)
					std::this_thread::yield();
			}

			flock.allBoidsPosUpdated = true;
		}

		if (Flock::flocksTimeToUpdate)
		{
			for (int bo_index = 0; bo_index < flock.boids.size(); bo_index++)
			{
				calNewDirection(flock.boids[bo_index], flock);
			}

			for (int boid_id = 0; boid_id < flock.boids.size(); boid_id++)
				flock.boids[boid_id].posUpdated = false;

			flock.allBoidsPosUpdated = false;
			Flock::flocksTimeToUpdate = false;
		}
		else
			std::this_thread::yield();
	}
}

// The function to spawn flock threads (Solution two:  using while loop and boolean variables)
void flockRunX()
{
	for (int fl_index = 0; fl_index < flocks.size(); fl_index++)
	{
		new std::thread(flockMoveX, std::ref(flocks[fl_index]));
	}

}