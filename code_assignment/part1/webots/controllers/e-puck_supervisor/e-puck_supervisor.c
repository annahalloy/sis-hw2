#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#include <math.h>
#define TIME_STEP 64

int main(int argc, const char* argv[])
{
  wb_robot_init();

  // Get light nodes
  WbNodeRef robot;
  robot = wb_supervisor_node_get_from_def("e-puck");
  WbFieldRef epuck_translation;
  WbFieldRef epuck_rotation;
  epuck_translation =  wb_supervisor_node_get_field(robot, "translation");
  epuck_rotation = wb_supervisor_node_get_field(robot, "rotation");


  // Main loop
  FILE* logfile= fopen("supervisor_logfile.txt", "w");
  float elapsed = 0.f;
  while (wb_robot_step(TIME_STEP) != -1) {
    double x,z,th;
    x = (double)wb_supervisor_field_get_sf_vec3f(epuck_translation)[2];  
    z = (double)wb_supervisor_field_get_sf_vec3f(epuck_translation)[0];  
    th = (double)wb_supervisor_field_get_sf_rotation(epuck_rotation)[3]; // heading

    //printf("Supervisor knows that at timestep %f the epuck is at: %f, %f with orientation %f\n",elapsed,x,z,th);

    
    // write to file
    fprintf(logfile, "%f %f %f %f\n", elapsed, x, z, th);
    fflush(logfile);

  
    elapsed += TIME_STEP / 1000.f;
  }

  fclose(logfile);
  wb_robot_cleanup();
  return 0;
}
