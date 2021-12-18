#include <stdio.h>
#include <string.h>

// base webots libraries
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/keyboard.h>
#include <webots/motor.h>

// webots sensor libraries
#include <webots/accelerometer.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

// include odometry file
#include "kalman.h"

//-----------------------------------------------------------------------------------//
// You can play with these values!

/*CONTROL MOTION*/
#define MOVE        true   // Set to false to prevent the robot from moving
#define PHASE_SHIFT 0.1234 // Change this to generate new trajectories

/*VERBOSE_FLAGS*/
#define VERBOSE_COMPASS false   // Print compass values
#define VERBOSE_ENC     false   // Print encoder values
//-----------------------------------------------------------------------------------//

/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  controller_error(X, Y, __LINE__, __FILE__)

/*ROBOT DEFINITION*/
#define ROBOT_DEF "ROBOT1"

/*DISTANCE SENSORS*/
#define NB_SENSORS 8
#define RANGE (1024 / 2)
#define RANGE_THRES 500         // Trigger avoidance if reading exceeds this value
#define INVALID 1000            // Arbitrary value to mark data as invalid

/*CONSTANTES*/
#define MAX_SPEED 1000          // Maximum speed
#define INC_SPEED 5             // Increment not expressed in webots
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define COMPASS_FREQ 1.0        // Compass measures at 1hz
#define COMPASS_TIME_DIV floor(1.0/COMPASS_FREQ/((1e-3)*_robot.time_step)) // Update compass every COMPASS_TIME_DIV steps

//-----------------------------------------------------------------------------------//
/*DEFINITIONS*/

typedef struct
{
  int time_step;
  WbDeviceTag compass;
  WbDeviceTag ps[NB_SENSORS];
  WbDeviceTag left_encoder;
  WbDeviceTag right_encoder;
  WbDeviceTag left_motor;
  WbDeviceTag right_motor;
} simulation_t;

typedef struct
{
  int max_ds;
  double ds_values[NB_SENSORS];
  double ds_range[NB_SENSORS];
  double compass;
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;

//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static simulation_t   _robot;
static measurement_t  _meas;
static pose_t         _ground_truth, _odo_enc, _kalman, _wall_detection;
static char updated_axis = 'o'; // can be either {o,x,y}

static FILE *fp;
//-----------------------------------------------------------------------------------//
/*FUNCTIONS DECLARATION*/

static void constraint_heading(double* heading);
static double IR_to_distance(double proximity);
static void compute_position_from_wall_detection(pose_t robot_pose);

static bool controller_init();
static bool controller_init_time_step();
static bool controller_init_distance_sensors();
static bool controller_init_compass();
static bool controller_init_encoder();
static bool controller_init_motors();
static bool controller_init_log(const char* filename);

static void controller_get_distance();
static void controller_get_compass();
static void controller_get_ground_truth(bool init);
static void controller_get_encoder();

static void controller_set_speed();

static void controller_print_log(double time);

static bool controller_error(bool test, const char * message, int line, const char * fileName);

//-----------------------------------------------------------------------------------//

int main()
{

  // initialize the webots controller library
  wb_robot_init();

  if(CATCH_ERR(controller_init(), "Controller fails to init \n"))
    return 1;

  // initialize the kalman filter
  kal_reset(_robot.time_step);

  while (wb_robot_step(_robot.time_step) != -1)
  {
    // Get ground truth and encoders
    controller_get_ground_truth(false);
    controller_get_encoder();

    // Odometry
    odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
    constraint_heading(&_odo_enc.heading);

    // Measurements
    controller_get_distance();
    controller_get_compass();
    compute_position_from_wall_detection(_ground_truth); // use either {_ground_truth, _odo_enc, _kalman}

    // Kalman filter prediction
    kal_compute_input_u(_meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
    kal_predict(&_kalman);

    // Kalman filter updates
    if(updated_axis == 'x'){
      kal_update_x(&_kalman,_wall_detection.x);
    }
    if(updated_axis == 'y'){
      kal_update_y(&_kalman,_wall_detection.y);
    }
    if(_meas.compass != INVALID){
      kal_update_heading(&_kalman,_meas.compass);
    }
    constraint_heading(&_kalman.heading);

		// Set the wheels' speed
		controller_set_speed();

	  // Log the data
	  controller_print_log(wb_robot_get_time());
  }

  // Close the log file
  if(fp != NULL)
    fclose(fp);

  // End of the simulation
  wb_robot_cleanup();

  return 0;
}

//-----------------------------------------------------------------------------------//

/**
 * @brief   Get the ground truth. The data retrieved here is comparable to what an MCS could provide.
 *          The data returned is expressed in the frame A.
 * @param init (bool) whether to set the initial pose with the acquired measurement.
 */
void controller_get_ground_truth(bool init){

  // Set the reference to the robot
  WbNodeRef ref = wb_supervisor_node_get_from_def(ROBOT_DEF);

  // Get the robot orientation in the World frame
  const double* o = wb_supervisor_node_get_orientation(ref);
  if(o != NULL){

    // Assume pure rotation around world y-axis (NUE).
    // Use the x-projection of the 3D rotation matrix (1st column)
    // heading = atan2(cos,-sin)
    // Result in range ]-pi,pi]
    _ground_truth.heading = atan2(o[0],o[6]);
  }

  // Get the robot position in the world frame
  const double* p = wb_supervisor_node_get_position(ref);
  if(p != NULL){
    _ground_truth.x =  p[0];  // =  X in world frame
    _ground_truth.y = -p[2];  // = -Z in world frame
  }

  constraint_heading(&_ground_truth.heading);
}

/**
 * @brief   Constrain an angle within [-pi,pi]
 *
 * @param heading value to constrain
 */
void constraint_heading(double* heading){
  while(*heading > M_PI){
    *heading -= 2.0*M_PI;
  }
  while(*heading < -M_PI){
    *heading += 2.0*M_PI;
  }
}

/**
 * @brief     Get the distance measurements.
 *            Return -1 if out of reach
 */
void controller_get_distance(){
  // read sensor values
  _meas.max_ds = 0;
  for (int i = 0; i < NB_SENSORS; i++){

    double value = wb_distance_sensor_get_value(_robot.ps[i]); // range: 0 (far) to 3800 (close)
    _meas.ds_values[i] = value;

    // Identify sensor with highest response (shortest range)
    if(_meas.ds_range[_meas.max_ds] < value) _meas.max_ds = i;

    // Convert into a range
    _meas.ds_range[i] = IR_to_distance(value);
  }
}

/**
 * @brief   Compute the position of the robot taking into
 *          account its current pose 'robot_pose' and the
 *          current wall detection.
 *
 * @param robot_pose (pose_t) pose to use as reference for the location of the robot
 */
void compute_position_from_wall_detection(pose_t robot_pose){

  // Update these values according to your detection
  _wall_detection.x = INVALID;
  _wall_detection.y = INVALID;
  _wall_detection.heading = INVALID;
  updated_axis = 'o';

  /**
   * TODO: using the current robot pose: {robot_pose} and measurements: {_meas.ds_range, _meas.ds_values},
   * update the fields of the structure _wall_detection and set updated_axis to either 'x' or 'y',
   * depending on the axis being updated.
   *
   * _wall_detection contains the assumed position of the robot in the arena where the wall detection occured.
   * It should therefore be set based on the current position of the robot in the arena (given by robot_pose),
   * and the acquired wall detection.
   *
   * If you decide not to include the current detection, leave the fields of _wall_detection to
   * their default values.
   */

    double dist_wall = _meas.ds_range[_meas.max_ds];
    
    printf("Wall dist  %f, %f, %f, %f\n", _meas.ds_range[0], _meas.ds_range[1], _meas.ds_range[6],_meas.ds_range[7]);
    
    if (dist_wall > 0){

      double dsx = abs(0.5-robot_pose.x);
      double dsy = abs(0.5-robot_pose.y);
      if(dsx < dsy){
        updated_axis = 'x';
        if(robot_pose.x > 0){
          _wall_detection.x = 0.47 - dist_wall;
        } else {
          _wall_detection.x = -0.47 - dist_wall;
        }
      } else {
        updated_axis = 'y';
        if(robot_pose.y > 0){
           _wall_detection.y = 0.47 - dist_wall;
        } else {
          _wall_detection.y = -0.47 - dist_wall;
        }
      }
    }

  return;
}

/**
 * @brief   Get the compass measurments for heading of the robot.
 *          Thie methods returns INVALID if no data is available.
 */
void controller_get_compass(){

  static int counter = -1;
  if(counter == -1) counter = COMPASS_TIME_DIV;

  counter++;

  if(counter >= COMPASS_TIME_DIV){

    counter = 0;

    const double *c = wb_compass_get_values(_robot.compass);
    double x_align = c[0]; // x-axis alignment with North (x-axis in world) [-1,1]
    double z_align = c[2]; // z-axis alignment with North (x-axis in world) [-1,1]

    // Make sure data is within [-1,1]
    if(x_align > 1.0) x_align = 1.0;
    else if(x_align < -1.0) x_align = -1.0;
    if(z_align > 1.0) z_align = 1.0;
    else if(z_align < -1.0) z_align = -1.0;

    _meas.compass = ( (x_align>=0 && z_align>=0) || (x_align<0 && z_align>0) ) ? acos(x_align) : -acos(x_align);

    constraint_heading(&_meas.compass);
  }
  else{
     _meas.compass = INVALID; // no new data available
  }

  if(VERBOSE_COMPASS)
    printf("ROBOT compass is at angle (deg): %.3lf\n",RAD2DEG(_meas.compass));
}

/**
 * @brief      Read the encoders values from the sensors
 */
void controller_get_encoder()
{
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(_robot.left_encoder);

  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;

  _meas.right_enc = wb_position_sensor_get_value(_robot.right_encoder);

  if(VERBOSE_ENC)
    printf("ROBOT enc : %g %g\n", _meas.left_enc, _meas.right_enc);
}

/**
 * @brief      Set the speed to the motors according to the user commands
 */
void controller_set_speed()
{
  // Get the simulation current time
  double time = wb_robot_get_time();

  // Init speed commands
  double speed_left  = 0.0;
  double speed_right = 0.0;

  if(MOVE){
    if(_meas.ds_values[0] > RANGE_THRES || _meas.ds_values[1] > RANGE_THRES || _meas.ds_values[2] > RANGE_THRES){
      // Obstacle on the right, turn left
      speed_left  = -MAX_SPEED_WEB/4.0;
      speed_right =  MAX_SPEED_WEB/4.0;
    }
    else if(_meas.ds_values[5] > RANGE_THRES || _meas.ds_values[6] > RANGE_THRES || _meas.ds_values[7] > RANGE_THRES){
      // Obstacle on the left, turn right
      speed_left  =  MAX_SPEED_WEB/4.0;
      speed_right = -MAX_SPEED_WEB/4.0;
    }
    else{
      // Random walk
      speed_left  = MAX_SPEED_WEB/2.0 + MAX_SPEED_WEB/6.0*sin(0.6*(time-PHASE_SHIFT));
      speed_right = MAX_SPEED_WEB/2.0 + MAX_SPEED_WEB/6.0*cos(0.5*time);
    }
  }

  // Update the speed on Webots
  wb_motor_set_velocity(_robot.left_motor, speed_left);
  wb_motor_set_velocity(_robot.right_motor, speed_right);
}


/**
 * @brief      Log the usefull informations about the simulation
 *
 * @param[in]  time  The time
 */
void controller_print_log(double time)
{

  if( fp != NULL)
  {
    // fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
    //         time, _pose.x, _pose.y , _pose.heading, _meas.gps[0], _meas.gps[1],
    //   _meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc,
    //   _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading);

      // _meas, _ground_truth, _odo_enc

      fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            time,                                                     // 1
            _ground_truth.x, _ground_truth.y, _ground_truth.heading,  // 3
            _odo_enc.x, _odo_enc.y, _odo_enc.heading,                 // 3
            _kalman.x, _kalman.y, _kalman.heading,                    // 3
            _meas.right_enc, _meas.left_enc,                          // 2
            _meas.compass,                                            // 1
            (double)_meas.max_ds, _meas.ds_range[_meas.max_ds],       // 2
            _wall_detection.x, _wall_detection.y);                    // 2
  }

}

//-----------------------------------------------------------------------------------//
/*INIT*/

/**
 * @brief      Run the initialization. Set the variables and structure to 0. Try to initialize the Webots components.
 *
 * @return     Return true if it rise and error
 */
bool controller_init()
{

  bool err = false;

  memset(&_robot, 0 , sizeof(simulation_t));

  memset(&_meas, 0 , sizeof(measurement_t));

  memset(&_ground_truth, 0 , sizeof(pose_t));

  memset(&_odo_enc, 0 , sizeof(pose_t));

  memset(&_kalman, 0 , sizeof(pose_t));

  memset(&_wall_detection, INVALID, sizeof(pose_t));

  CATCH(err,controller_init_time_step());

  controller_get_ground_truth(true);

  CATCH(err,controller_init_distance_sensors());

  CATCH(err,controller_init_compass());

  CATCH(err,controller_init_encoder());

  CATCH(err,controller_init_motors());

  CATCH(err, controller_init_log("data.csv"));

  wb_keyboard_enable(_robot.time_step);

  odo_reset(_robot.time_step);

  return err;
}

/**
 * @brief      Initialize the distance sensors from Webots
 *
 * @return     return true if it fails
 */
bool controller_init_distance_sensors(){

  bool err = false;
  // get and enable each distance sensor
  char name[] = "ps0";
  for(int i = 0; i < NB_SENSORS; i++) {

    _robot.ps[i] = wb_robot_get_device(name);

    err = CATCH_ERR(_robot.ps[i] == 0, "No distance sensor node found in the current robot file\n");

    if( !err ){
      // TODO: enable the distance sensors (use _robot.ps[i] and _robot.time_step)
      wb_distance_sensor_enable(_robot.ps[i],_robot.time_step);
    }
    else
      return err;

    name[2]++; // increase the device name to "ps1", "ps2", etc.
  }
  return err;
}

/**
 * @brief      Initialize the compass sensor from Webots
 *
 * @return     return true if it fails
 */
bool controller_init_compass(){
  _robot.compass = wb_robot_get_device("compass");

  bool err = CATCH_ERR(_robot.compass == 0, "No compass node found in the current robot file\n");

  if( !err )
  {
    // TODO: Enable the sensor on Webots (use _robot.compass and _robot.time_step)
    wb_compass_enable(_robot.compass, _robot.time_step);
  }

  return err;
}

/**
 * @brief      Initiliaze the the wheel encoders (position sensors) from Webots
 *
 * @return      return true if it fails
 */
bool controller_init_encoder()
{
  // Get the device from Webots
  _robot.left_encoder = wb_robot_get_device("left wheel sensor");
  // Write an error message if the initialization fails
  bool err = CATCH_ERR(_robot.left_encoder == 0, "No left wheel sensor node found in the current robot file\n");

  _robot.right_encoder = wb_robot_get_device("right wheel sensor");

  CATCH(err,CATCH_ERR(_robot.right_encoder == 0, "No right wheel sensor node found in the current robot file\n"));

  if( !err ) // if no error initialize the sensors
  {
    // TODO: enable both left and right encoders (use _robot.left_encoder, _robot.right_encoder and _robot.time_step)
    wb_position_sensor_enable(_robot.left_encoder,  _robot.time_step);
    wb_position_sensor_enable(_robot.right_encoder, _robot.time_step);
  }

  return err;
}


/**
 * @brief      Initiliaze the the wheel motors from Webots
 *
 * @return     return true if it fails
 */
bool controller_init_motors()
{
  // To Do : Get the left motors device from webots. Note : the name of the gps is "left wheel motor"
  _robot.left_motor = wb_robot_get_device("left wheel motor");

  bool err = CATCH_ERR(_robot.left_motor == 0, "No left wheel motor node found in the current robot file\n");

  // To Do : Get the right motors device from webots. Note : the name of the gps is "right wheel motor"
  _robot.right_motor = wb_robot_get_device("right wheel motor");

  CATCH(err,CATCH_ERR(_robot.left_motor == 0, "No right wheel motor node found in the current robot file\n"));

  if( !err )
  {
    wb_motor_set_position(_robot.left_motor, INFINITY);   // To Do : Set the left motor position to INFINITY.    Note : use _robot.left_motor
    wb_motor_set_position(_robot.right_motor, INFINITY);  // To Do : Set the right motor position to INFINITY.   Note : use _robot.right_motor
    wb_motor_set_velocity(_robot.left_motor, 0.0);        // To Do : Set the left motor speed to 0.               Note : use _robot.left_motor
    wb_motor_set_velocity(_robot.right_motor, 0.0);       // To Do : Set the right motor speed to 0.              Note : use _robot.right_motor
  }

  return err;
}


/**
 * @brief      Initialize the simulation time step on Webots
 *
 * @return     return true if it fails
 */
bool controller_init_time_step()
{
  _robot.time_step =  wb_robot_get_basic_time_step();

return CATCH_ERR(_robot.time_step == 0,"time step is not set\n");
}


/**
 * @brief      Initialize the logging of the file
 *
 * @param[in]  filename  The filename to write
 *
 * @return     return true if it fails
 */
bool controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");

  bool err = CATCH_ERR(fp == NULL, "Fails to create a log file\n");

  if( !err )
  {
    fprintf(fp, "time; gt_x; gt_y; gt_heading; odo_enc_x; odo_enc_y; odo_enc_heading; kalman_x; kalman_y; kalman_heading; right_enc; left_enc; compass; ds_id; ds_range; wall_x; wall_y\n");
  }

  return err;
}


/**
 * @brief      Do an error test if the result is true write the message in the stderr.
 *
 * @param[in]  test     The error test to run
 * @param[in]  message  The error message
 *
 * @return     true if there is an error
 */
bool controller_error(bool test, const char * message, int line, const char * fileName)
{
  if (test)
  {
    char buffer[256];

    sprintf(buffer, "file : %s, line : %d,  error : %s", fileName, line, message);

    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}


/*--------------------------------UTILS---------------------------------------*/

/**
 * @brief   Convert the distance sensor value to a range
 *
 * @param   proximity (double) proximity value
 *
 * @return (double) corresponding range (-1 if out of range)
 */
double IR_to_distance(double proximity){
  /* Uses values defined in webots proto:
          0 4095 0.005
          0.005 3474 0.037
          0.01 2211 0.071
          0.02 676 0.105
          0.03 306 0.125
          0.04 164 0.206
          0.05 90 0.269
          0.06 56 0.438
          0.07 34 0.704
	*/
    double prox_values [9] = {4095, 3474, 2211, 676, 306, 164, 90, 56, 34};
    double dist_values [9] = {0, 0.005, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07};

    if (proximity > 4094) {
        return 0.0;
    }

    int a_index, b_index;
    double x_a , x_b ;
    double y_a , y_b ;
    for(int i=0; i<9-1; i++){
        if (proximity >= prox_values[i + 1]) {
            // linear interpolations
            a_index = i;
            b_index = i+1;
            x_a = prox_values[a_index];
            x_b = prox_values[b_index];
            y_a = dist_values[a_index];
            y_b = dist_values[b_index];
            return y_a + (proximity - x_a)*(y_b-y_a)/(x_b-x_a);
        }
    }
    return -1.0;
}
