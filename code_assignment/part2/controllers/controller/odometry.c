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

	// Update the variable _odo_pose_enc that stores the current
	// odometry estimate:
	///////////////////////////////////////////////////////////////

	//  Rad to meter : Convert the wheel encoders units into meters

	Aleft_enc  = Aleft_enc * WHEEL_RADIUS;

	Aright_enc = Aright_enc * WHEEL_RADIUS;


	// Comupute speeds : Compute the forward and the rotational speed

	double omega = (Aright_enc - Aleft_enc) / (WHEEL_AXIS * _T);

	double speed = (Aright_enc + Aleft_enc) / (2.0 * _T);


	//  Compute the speed into the inertial frame (A)

	double speed_wx = speed * cos(_odo_pose_enc.heading);

	double speed_wy = speed * sin(_odo_pose_enc.heading);

	double omega_w  = omega;

	// Integration : Euler method

	_odo_pose_enc.x = _odo_pose_enc.x + speed_wx * _T ;

	 _odo_pose_enc.y = _odo_pose_enc.y + speed_wy * _T ;

	_odo_pose_enc.heading = _odo_pose_enc.heading + omega_w * _T;

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
