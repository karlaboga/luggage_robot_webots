#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>

#include <stdio.h>

#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };

int main() {
  wb_robot_init();
  
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2 * TIME_STEP);
  wb_camera_recognition_enable(camera, 2 * TIME_STEP);

  int i = 0;
  int state = WAITING;
  const double target_positions[] = {-1.3, -3, -2.38, -2};
  const double target_positions1[] = {-2.3, 2.8, -2.31, -1.51};
  const double target_positions2[] = {-2, -1.6, -2.5, -1.51};
  double r = 0.00, g = 0.00, b = 0.00;
  double speed = 1;

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  
  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);
  
  // Set the initial position of the robot (in radians)
  for (int i = 0; i < 4; ++i) {
    wb_motor_set_position(ur_motors[i], 0.0);
  }
  for (i = 0; i < 3; ++i)
      wb_motor_set_position(hand_motors[i], 0.05);
      
  while (wb_robot_step(TIME_STEP) != -1) {

      printf("start---------------------\n");

      int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
      printf("\nNumber of luggages: %d.\n", number_of_objects);

      const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
      
      for (i = 0; i < number_of_objects; ++i) {
        for (int j = 0; j < objects[i].number_of_colors; ++j) {
          r = objects[i].colors[3 * j];
          g = objects[i].colors[3 * j + 1];
          b = objects[i].colors[3 * j + 2];
          if (r == 1.000000 && g == 0.000000 && b == 0.000000)
            printf("FLIGHT NUMBER: W61343\n");
          else if (r == 0.000000 && g == 1.000000 && b == 0.000000)
            printf("FLIGHT NUMBER: AM3645\n");
           else if (r == 0.000000 && g == 0.000000 && b == 1.000000)
            printf("FLIGHT NUMBER: F3954\n");
        }
      }

      switch (state) {
      
        case WAITING:
        for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.05); 
          printf("Distance: %lf\n", wb_distance_sensor_get_value(distance_sensor));
          fflush(stdout);
          if (wb_distance_sensor_get_value(distance_sensor) < 300 && wb_distance_sensor_get_value(distance_sensor) > 100) {
            state = GRASPING;
            printf("Grasping object\n");
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85); 
          }
          break;
          
        case GRASPING:
          if ( r == 1.000000 && g == 0.000000 && b == 0.000000)
            for(int i=0; i<4; ++i)
                wb_motor_set_position(ur_motors[i], target_positions2[i]);
            else if (r == 0.000000 && g == 1.000000 && b == 0.000000)
                  for(i=0; i<4; ++i)
                    wb_motor_set_position(ur_motors[i], target_positions1[i]);

            else if(r == 0.000000 && g == 0.000000 && b == 1.000000)
                for(i=0; i<4; i++)
                   wb_motor_set_position(ur_motors[i], target_positions[i]);
          printf("Rotating arm\n");
          state = ROTATING;
          break;
          
        case ROTATING:
          if (wb_position_sensor_get_value(position_sensor) < -2.3) {
            printf("Releasing object\n");
            state = RELEASING;
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));
          }
          break;
          
        case RELEASING:
          for (i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], 0.0);
          printf("Rotating arm back\n");
          state = ROTATING_BACK;
          break;
          
        case ROTATING_BACK:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
            printf("Waiting object\n");
          }
          break;
      }
    
    printf("-----------------------end\n");
  }

  wb_robot_cleanup();
  return 0;
}
