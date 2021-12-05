#include <stdio.h>
#include <string.h>

// include webots libraries
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>


//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  controller_error(X, Y, __LINE__, __FILE__)

/*CONSTANTES*/
#define MAX_SPEED 1000          // Maximum speed 
#define INC_SPEED 5             // Increment not expressed in webots 
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define TIME_INIT_ACC 5 	      // Time in second
#define CAMERA_HEIGHT 39
#define CAMERA_WIDTH 52

/*VERBOSE_FLAGS*/
#define VERBOSE_GPS false       // Print GPS values
#define VERBOSE_ACC false       // Print accelerometer values
#define VERBOSE_ACC_MEAN false  // Print accelerometer mean values
#define VERBOSE_POSE false      // Print pose values
#define VERBOSE_ENC false       // Print encoder values
#define VERBOSE_KEY true       // Print keyboard values

//-----------------------------------------------------------------------------------//
/*DEFINITIONS*/
typedef struct
{
  int time_step;
  WbDeviceTag gps;
  WbDeviceTag acc;
  WbDeviceTag left_encoder;
  WbDeviceTag right_encoder;
  WbDeviceTag left_motor; 
  WbDeviceTag right_motor; 
} simulation_t;

typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;

typedef struct
{
  bool isRunning;
  bool sign_left;
  bool sign_right;
  int  speed;
} motor_t;


bool take_picture = false;


//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static simulation_t   _robot;
static motor_t        _motor = {false, true, true, MAX_SPEED / 4.0};

static FILE *fp;

WbDeviceTag camera; // handler for the camera device
//-----------------------------------------------------------------------------------//
/*FUNCTIONS*/
void takeAndSaveImage();
static bool controller_init();
static bool controller_init_time_step();
static bool controller_init_motors();

static void controller_get_keyboard();

static void controller_set_speed();

static bool controller_error(bool test, const char * message, int line, const char * fileName);

//-----------------------------------------------------------------------------------//

int main() 
{
  //initialize the webots controller library
  wb_robot_init();
  
  controller_init();

  while (wb_robot_step(_robot.time_step) != -1) 
  {
   
		
		// 1. Cognition / Action
		controller_get_keyboard();
		
		// 2. Motor Controls
		controller_set_speed();

    // 3. take picture (save) if requested
    if(take_picture) {
      take_picture = false;
      takeAndSaveImage();
    }

  
  }

  // Close the log file
  if(fp != NULL)
    fclose(fp);

  // End of the simulation
  wb_robot_cleanup();

  return 0;
}

//-----------------------------------------------------------------------------------//



void takeAndSaveImage() {
   /* captures an image and stores it in 'image.jpg'*/
  wb_camera_save_image(camera, "image.jpg",100);
}


/**
 * @brief      Change the speed of the motors according to the keys pressed by the user
 */
void controller_get_keyboard()
{
  // Get the ascii code of the key pressed by the user 
  int key = wb_keyboard_get_key();

  char msg[128] = "";

 // move the robot with the keys
  switch (WB_KEYBOARD_KEY & key) 
  {
    case WB_KEYBOARD_UP: 
      sprintf(msg, "Move forward\n");
      _motor.sign_left  = true;
      _motor.sign_right = true;
      break;
  
    case WB_KEYBOARD_DOWN: 
     sprintf(msg, "Move backward\n");
      _motor.sign_left  = false;
      _motor.sign_right = false;
     break;
  
    case WB_KEYBOARD_LEFT: // turn left
     sprintf(msg, "Turn left\n" );
      _motor.sign_left  = false;
      _motor.sign_right = true;
      break;
  
    case WB_KEYBOARD_RIGHT: 
     sprintf(msg, "Turn right\n");
      _motor.sign_left  = true;
      _motor.sign_right = false;
      break;
  
    case 'S':
       if(_motor.isRunning) sprintf(msg, "STOP\n");
      _motor.isRunning = false;
    break;
  
    case 'R':
       if(!_motor.isRunning) sprintf(msg, "RUN\n");
      _motor.isRunning = true;
    break;
  
    case 'U':
       sprintf(msg, "Speed UP\n");
      _motor.speed += INC_SPEED;
    break;
  
    case 'D':
       sprintf(msg, "Speed Down\n");
      _motor.speed -= INC_SPEED;
    break;	

    case 'P':
       sprintf(msg, "Picture time!\n");
       take_picture = true;
    break;

    default:
      break;
  }

  if(VERBOSE_KEY)
    printf(msg);
}



/**
 * @brief      Set the speed to the motors according to the user commands
 */
void controller_set_speed()
{

  // Check speed bounds
  _motor.speed = (_motor.speed > MAX_SPEED ? MAX_SPEED : _motor.speed);
  _motor.speed = (_motor.speed < 0 ? 0 : _motor.speed);

  double msl_w = 0;
  double msr_w = 0;

  if(_motor.isRunning)
  {
    // Convert speed for Webots
    msl_w = _motor.speed * MAX_SPEED_WEB / MAX_SPEED;
    msr_w = _motor.speed * MAX_SPEED_WEB / MAX_SPEED;

    // Set direction of the motors
    msl_w = (_motor.sign_left  ? msl_w : -msl_w);
    msr_w = (_motor.sign_right ? msr_w : -msr_w);
  }

  // Update the speed on Webots
  wb_motor_set_velocity(_robot.left_motor, msl_w);
  wb_motor_set_velocity(_robot.right_motor, msr_w);
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


  CATCH(err,controller_init_time_step());
  CATCH(err,controller_init_motors());
  
  wb_keyboard_enable(_robot.time_step);


    // Camera - this function is defined above. Please have a look at it.
  camera = wb_robot_get_device("camera"); // identifier of the camera for enabling, sending commands to or reading data from this device
  wb_camera_enable(camera, _robot.time_step); // enables the camera with as sampling period the specified time step
  int camera_width = wb_camera_get_width(camera);
  int camera_height = wb_camera_get_height(camera);
  printf("Camera width  = %d, height = %d \n", camera_width, camera_height);
  if (camera_width != CAMERA_WIDTH || camera_height != CAMERA_HEIGHT) {
    printf("Camera width/height is not defined correctly with #define. Correct values are printed above. Please set them correctly \n");
    return (-1);
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

return CATCH_ERR(_robot.time_step == 0,"time step is not is not set\n");
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

    fprintf(stderr,buffer);

    return(true);
  }

  return false;
}