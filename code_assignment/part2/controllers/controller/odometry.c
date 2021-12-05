#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "odometry.h"

//-----------------------------------------------------------------------------------//

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC false     	// Print odometry values computed with wheel encoders
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t  _odo_pose_enc;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry pose to update 
 * @param[in]  Aleft_enc   The delta of the left encoder
 * @param[in]  Aright_enc  The delta of the right encoder
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	// Copy the current odometry 
	memcpy(&_odo_pose_enc, odo, sizeof(pose_t));

	///////////////////////////////////////////////////////////////
	// Update the variable _odo_pose_enc that stores the current
	// odometry estimate:

	// TODO

	///////////////////////////////////////////////////////////////

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

	if(VERBOSE_ODO_ENC)
    	printf("ODO with wheel encoders : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
}

/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{
	memset(&_odo_pose_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
}
