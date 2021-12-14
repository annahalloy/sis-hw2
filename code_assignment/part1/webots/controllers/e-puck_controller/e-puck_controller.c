// #### Header Files
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include "kiss_fft.h"

// DO NOT MODIFY !
#define TIME_STEP 64
#define NB_SENSORS 8
#define MAX_SPEED_WB      6.28    // Maximum speed webots
#define CAMERA_HEIGHT 39
#define CAMERA_WIDTH 52
#define SIGNAL_LENGTH 64
// -- END DO NOT MODIFY

// CUSTOM defines Modify here: add your thresholds, etc. here


// -- END CUSTOM defines

// ######################################################################
// #### Global variables

WbDeviceTag left_motor; // handler for left wheel of the robot
WbDeviceTag right_motor; // handler for the right wheel of the robot
WbDeviceTag ps[NB_SENSORS]; // handlers for distance sensors (this is an array)
WbDeviceTag camera; // handler for the camera device


// enum used to transmit the decision of the image analysis
enum analysisResult {left, back, right, unclear};
// enum and global variable used to store the next behavior
enum basicBehaviors {goForward, turnLeft, turnRight, turnAround, goBackwards, takePicture, analyzePicture} nextBehavior;

// variable used to store the most recent image
int signal_data[CAMERA_HEIGHT][CAMERA_WIDTH];
// store distance sensor values to use them more easily in different functions
double distance_sensors_values[NB_SENSORS];

// ######################################################################
// #### Functions

void init() {
  // Robot
  wb_robot_init();
  
  // Motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  
  // Camera - this function is defined above. Please have a look at it.
  camera = wb_robot_get_device("camera"); // identifier of the camera for enabling, sending commands to or reading data from this device
  wb_camera_enable(camera, TIME_STEP); // enables the camera with as sampling period the specified time step
  int camera_width = wb_camera_get_width(camera);
  int camera_height = wb_camera_get_height(camera);
  printf("Camera width  = %d, height = %d \n", camera_width, camera_height);
  if (camera_width != CAMERA_WIDTH || camera_height != CAMERA_HEIGHT) {
    printf("Camera width/height is not defined correctly with #define. Correct values are printed above. Please set them correctly \n");
    return (-1);
  }
  
  // Distance sensor
  char textPS[] = "ps0";
  for (int it=0; it<NB_SENSORS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
    wb_distance_sensor_enable(ps[it],TIME_STEP);
  wb_robot_step(2*TIME_STEP);
  }
}


void print_camera_gray_image() {
  printf("Top 10x10 part of camera image grayscale data ----------------------\n");
  for (int h=0; h < 10; h++){
    for (int w=0; w < 10; w++){
      printf("%03d   ", signal_data[h][w]);
    }
    printf("\n");
  }
}


/* read the distance sensors */
void get_sensor_input() {
  //int i;

  /* Modify here: get distance sensor values and store them in the distance_sensors_values array */
  double distance_sensors_value[NB_SENSORS];
  
  for (int i=0; i<NB_SENSORS; i++) {
  distance_sensors_values[i] = ps[i];
  }
  
  /* Optional (if needed for your algorithm): add filtering to avoid noise */
  
}

void takeImageBehavior() {
  printf("Running behavior: takeImageBehavior\n");
   /* captures an image and stores it in 'signal_data'*/
  const unsigned char* im = wb_camera_get_image(camera);
  for (int m=0;m<CAMERA_WIDTH;m++) {
    for (int n=0;n<CAMERA_HEIGHT;n++) {
      signal_data[n][m]= wb_camera_image_get_grey(im,CAMERA_WIDTH,m,n); // convert from color to grey scale
    }
  }
}


enum analysisResult analyzePictureBehavior(){
  printf("Running behavior: analyzePictureBehavior\n");

     /* fft variables */  
    kiss_fft_cfg cfg = kiss_fft_alloc( SIGNAL_LENGTH ,0 ,NULL, NULL );  
    kiss_fft_cpx cx_in_example[SIGNAL_LENGTH], cx_out_example[SIGNAL_LENGTH];
    kiss_fft_cpx cx_in_col[SIGNAL_LENGTH], cx_out_col[SIGNAL_LENGTH];
    kiss_fft_cpx cx_in_row[SIGNAL_LENGTH], cx_out_row[SIGNAL_LENGTH];
    
    /* loop variables */
    int n, m;


    /* fft usage example */
    // prepare input
    for (m=0;m<SIGNAL_LENGTH;m++) {
      cx_in_example[m].r = 0.;
      cx_in_example[m].i = 0.;
    }
    // add signal in the 'in' structure of the kiss_fft
    for (m=0;m<CAMERA_WIDTH;m++) {
      cx_in_example[m].r = signal_data[n][m];
    }
    // run fft
    kiss_fft( cfg , cx_in_example , cx_out_example ); 
    
    

    // Example on how to write the result to a file
    FILE *fp;
    fp =fopen("fft_wb.txt","w+");
    for(m = 0; m < SIGNAL_LENGTH ; m++) { 
        fprintf(fp,"%d,%f,%f \n",m,cx_out_example[m].r);
    }  
    fclose(fp);

    // free fft memory
    free(cfg);
  
    /* Modify here: calculate average of signal (image) */
    long int sum=0;
    double mean=0;
    
    


    /* prepare the signal variables */
    for (m=0;m<SIGNAL_LENGTH;m++) {
      cx_in_col[m].r = 0.;
      cx_in_col[m].i = 0.;
      cx_in_row[m].r = 0.;
      cx_in_row[m].i = 0.;
    }


    /* Modify here: do some fft preparation magic here */
   
   for (n = 0; n=1; n++) {
    for (m=0;m<CAMERA_WIDTH;m++) {
      cx_in_col[m].r = signal_data[n][m];
      cx_in_row[m].r = signal_data[n][m];
    }
    }

    /* Modify here: perform the fft using kiss_fft() and free() as in example above*/

    kiss_fft( cfg , cx_in_col , cx_out_col ); 
    double F_mag_col[SIGNAL_LENGTH] ;
    for (m=0;m<CAMERA_WIDTH;m++) {
    F_mag_col[m] = cx_out_col[m].r*cx_out_col[m].r + cx_out_col[m].i*cx_out_col[m].i;
    }
    
    free(cfg);
    
    kiss_fft( cfg , cx_in_row , cx_out_row ); 
    double F_mag_row[SIGNAL_LENGTH] ;
    for (m=0;m<CAMERA_WIDTH;m++) {
    F_mag_row[m] = cx_out_row[m].r*cx_out_row[m].r + cx_out_row[m].i*cx_out_row[m].i;
    }
    
    free(cfg);
    
    /* Modify here: decide on your strategy and the direction to turn*/
    

    
    decision = left;
    decision = right;
    decision = back;
    decision = unclear;
    /* Modify here: decide on your strategy and the direction to turn*/
    enum analysisResult decision;
    


    return decision;
} 
 
void goForwardBehavior(){
  printf("Running behavior: goForwardBehavior\n");
  /* Modify here: go forward */
  while(wb_robot_step(TIME_STEP) != -1){
    // Based on above computation, compute the wheel speeds and make the robot move.
    
    double left_speed = MAX_SPEED_WB/10;
    double right_speed = MAX_SPEED_WB/10;
    
    
    // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    /* Modify here: decide when to stop going forward and return to main loop*/
    
    //////Noticing an obstacle -> robot needs to take a picture and analyse it
    
    bool wall_detected = false;
	//double distance = 0;
	//int i,j;
	
	/// TODO: Implement detection of front wall
	if(ps[0] > 1000 || ps[7] > 1000 ){
		wall_detected = true;}
	
	if (!wall_detected){ // if no wall is detected we cannot update our position
		return;}
	
	if(wall_detected) {
	takeImageBehavior() ;}

  }
} 

void goBackwardsBehavior(){
  printf("Running behavior: goBackwardsBehavior\n");
  /* Modify here: go backwards */
  while(wb_robot_step(TIME_STEP) != -1){
     // Based on above computation, compute the wheel speeds and make the robot move.
    double left_speed = - MAX_SPEED_WB/10;
    double right_speed = - MAX_SPEED_WB/10;
     // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);

  /* Modify here: decide when to stop going backwards and return to main loop */
  
  
  

  }
}

void turnLeftBehavior(){
  printf("Running behavior: turnLeftBehavior\n");
  /* Modify here: turn left */
  while(wb_robot_step(TIME_STEP) != -1){
     // Based on above computation, compute the wheel speeds and make the robot move.
     double left_speed = 0;
     double right_speed = 0;
     // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);

  /* Modify here: decide when to stop turning and return to main loop */
     
  }
}

void turnRightBehavior(){
  printf("Running behavior: turnRightBehavior\n");
  /* Modify here: turn right */
  while(wb_robot_step(TIME_STEP) != -1){
     // Based on above computation, compute the wheel speeds and make the robot move.
     double left_speed = 0;
     double right_speed = 0;
     // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);

  /* Modify here: decide when to stop turning and return to main loop */

  }
}

        
  
    

// ######################################################################
// #### Main function
int main(int argc, char **argv){

  // ### Initialization of robot, other devices etc.
  init();
  
  // set initial behavior
  nextBehavior = goForward;

  // While loop
   while (wb_robot_step(TIME_STEP)!=-1){

    switch(nextBehavior){
      case goForward:
        goForwardBehavior();
        nextBehavior = takePicture ;
        break;

      case goBackwards:
        goBackwardsBehavior();
        nextBehavior = turnRight ;
        break;

      case turnLeft:
        turnLeftBehavior();
        nextBehavior = goForward ;
        break;

      case turnRight:
        turnRightBehavior();
        nextBehavior = goForward ;
        break;

      case takePicture:
        takeImageBehavior();
        nextBehavior = analyzePicture; ///
        break;

      case analyzePicture: ; // ';' is necessary in a switch case if you follow case by a declaration
        enum analysisResult res;
        res = analyzePictureBehavior();
        switch(res){
          case left:
            nextBehavior = turnLeft;
            break;
          case right:
            nextBehavior = turnRight;
            break;
          case back:
            nextBehavior = goBackwards;
            break;
          case unclear:
            nextBehavior = takePicture;
            break;
        }
        break;

      case turnAround:
        turnLeftBehavior();
        turnLeftBehavior();
        nextBehavior = goForward ; 
        break;

      default:
        printf("This behavior is not implemented.\n");
        break;
    }
        
  }
      
  wb_robot_cleanup();
  return 0;
}
