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
enum analysisResult decision;
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
    //return (-1);
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
  for (int i=0; i<NB_SENSORS; i++) {
  distance_sensors_values[i] = wb_distance_sensor_get_value(ps[i]);
  }
  /* Modify here: get distance sensor values and store them in the distance_sensors_values array */
  /* Optional (if needed for your algorithm): add filtering to avoid noise */
}




enum analysisResult analyzePictureBehavior(){
  printf("Runninsignal_data[n][m] = g behavior: analyzePictureBehavior\n");
     /* fft variables */
    kiss_fft_cpx cx_in_col[CAMERA_HEIGHT], cx_out_col[CAMERA_HEIGHT];
    kiss_fft_cpx cx_in_row[CAMERA_WIDTH], cx_out_row[CAMERA_WIDTH];
    print_camera_gray_image();
    
    for (int m=0;m<CAMERA_WIDTH;m++) {
        cx_in_row[m].r = 1.*signal_data[8][m];
        cx_in_row[m].i = 0.;
    }
    for (int n=0; n<CAMERA_HEIGHT; n++) {
        cx_in_col[n].r = 1.*signal_data[n][8];
        cx_in_col[n].i = 0.;
    }
    /* Modify here: perform the fft using kiss_fft() and free() as in example above*/
          
    kiss_fft_cfg cfg2 = kiss_fft_alloc( CAMERA_HEIGHT ,0 ,NULL, NULL );
    kiss_fft( cfg2 , cx_in_col , cx_out_col );
    double F_mag_col[CAMERA_HEIGHT] ;

    for (int n=0;n<CAMERA_HEIGHT;n++) {
    F_mag_col[n] = cx_out_col[n].r*cx_out_col[n].r + cx_out_col[n].i*cx_out_col[n].i;
    printf("f mag col %f\n", F_mag_col[n]);
    }
    
    free(cfg2);
    
    kiss_fft_cfg cfg3 = kiss_fft_alloc( CAMERA_WIDTH ,0 ,NULL, NULL );
    kiss_fft( cfg3 , cx_in_row , cx_out_row );
    double F_mag_row[CAMERA_WIDTH] ;
    
    for (int m=0;m<CAMERA_WIDTH;m++) {
        F_mag_row[m] = cx_out_row[m].r*cx_out_row[m].r + cx_out_row[m].i*cx_out_row[m].i;
        printf("f mag row %f\n", F_mag_row[m]);
    }
    
    free(cfg3);
    
    /* Modify here: decide on your strategy and the direction to turn*/
    
    bool peakStateCol = false;
        int peakCountCol = 0;
        
        for (int m=0;m<CAMERA_WIDTH;m++) {
          if ((F_mag_col[m]>10000)&&(peakStateCol == false)){
            peakCountCol +=1;
            peakStateCol = true;
            } else {
            peakStateCol = false;
          }
        }
    printf("peaks col  = %i \n",peakCountCol);
    
    bool peakStateRow = false;
    int peakCountRow = 0;
    
    for (int m=0;m<CAMERA_HEIGHT;m++) {
      if ((F_mag_row[m]>10000)&&(peakStateRow == false)){
        peakCountRow +=1;
        peakStateRow = true;
        } else {
        peakStateRow = false;
      }
    }
    printf("peaks row  = %i \n",peakCountRow);
    /* Modify here: decide on your strategy and the direction to turn*/
    if(peakCountCol>peakCountRow) {
        printf("right\n");
        return right ;
    }
    else if(peakCountCol<peakCountRow) {
        return left ;
        printf("left\n");
    }
    else if(peakCountCol==peakCountRow) {
        decision = back ;
        printf("back\n");
    }
    else {
        decision = unclear ;
    }
    return decision;
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
  
 
void goForwardBehavior(){
  printf("Running behavior: goForwardBehavior\n");
  /* Modify here: go forward */
  while(wb_robot_step(TIME_STEP) != -1){
    // Based on above computation, compute the wheel speeds and make the robot move.
    double left_speed = MAX_SPEED_WB/5;
    double right_speed = MAX_SPEED_WB/5;
    // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    /* Modify here: decide when to stop going forward and return to main loop*/
    get_sensor_input();
    bool wall_detected = false;
	//detection of front wall
    if(distance_sensors_values[0] + distance_sensors_values[7] > 200 ){
	wall_detected = true;
    }	
    if (!wall_detected){
          return;
    }
    if(wall_detected) {
        nextBehavior = takePicture ;
        return ;
    }
  }
} 



void goBackwardsBehavior(){
  printf("Running behavior: goBackwardsBehavior\n");
  /* Modify here: go backwards */
    double count = 0 ;
    
  while(wb_robot_step(TIME_STEP) != -1){
     // Based on above computation, compute the wheel speeds and make the robot move.
    double left_speed = - MAX_SPEED_WB/5;
    double right_speed = - MAX_SPEED_WB/5;
     // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);

  /* Modify here: decide when to stop going backwards and return to main loop */
     
      if (count == 10) {
          break;
      }
      count ++;
      
  }
}

void turnLeftBehavior(){
  printf("Running behavior: turnLeftBehavior\n");
  /* Modify here: turn left */
    double count = 0;
  while(wb_robot_step(TIME_STEP) != -1){
     // Based on above computation, compute the wheel speeds and make the robot move.
      double left_speed = - MAX_SPEED_WB/10;
      double right_speed = MAX_SPEED_WB/10;
     // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);

  /* Modify here: decide when to stop turning and return to main loop */
      if (count == 54) {
          break;
      }
      count ++;
  }
}

void turnRightBehavior(){
  printf("Running behavior: turnRightBehavior\n");
    
    double count = 0;

  while(wb_robot_step(TIME_STEP) != -1){
     // Based on above computation, compute the wheel speeds and make the robot move.
      double left_speed = MAX_SPEED_WB/10;
      double right_speed = - MAX_SPEED_WB/10;
     // Tip: You need to make sure that the wheel speeds do not exceed MAX_SPEED_WB
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);

  /*  when to stop turning and return to main loop */
      if (count == 54) {
          break;
      }
      count ++;
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
   printf("next behavior %d\n", nextBehavior);
    switch(nextBehavior){
      case goForward:
        nextBehavior = goForward ;
        goForwardBehavior();
        break;

      case goBackwards:
        goBackwardsBehavior();
        nextBehavior = turnAround ;
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
        nextBehavior = analyzePicture;
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
        turnRightBehavior();
        turnRightBehavior();
        nextBehavior = goForward;
        break;

      default:
        printf("This behavior is not implemented.\n");
        break;
    }
        
  }
        
  
      
  wb_robot_cleanup();
  return 0;
}
