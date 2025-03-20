#include "main.h"

void UCFO::SetAnim_RightFrontWheel(){

    //Determine rotation speed of wheel

    VECTOR3 speed;
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    // Actualización de la rotación de la rueda
    right_front_wheel_rotation = fmod(right_front_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    SetAnimation(anim_right_front_wheel_rotation, right_front_wheel_rotation);

    SetAnimation(anim_right_front_wheel_steer, 0.5 + angle_right / (45*RAD));

    SetAnimation(anim_right_front_wheel_travel, 0.5 + front_right_displacement / (2.0*travel));

}

void UCFO::SetAnim_LeftFrontWheel(){

    VECTOR3 speed;
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    left_front_wheel_rotation = fmod(left_front_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    SetAnimation(anim_left_front_wheel_rotation, left_front_wheel_rotation);

    SetAnimation(anim_left_front_wheel_steer, 0.5 + angle_left / (45*RAD));


}

void UCFO::SetAnim_RightRearWheel(){


    VECTOR3 speed;
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    right_rear_wheel_rotation = fmod(right_rear_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    SetAnimation(anim_right_rear_wheel_rotation, right_rear_wheel_rotation);

    SetAnimation(anim_right_rear_wheel_travel, 0.5 + rear_right_displacement / (2.0 * travel));

}

void UCFO::SetAnim_LeftRearWheel(){

    VECTOR3 speed;
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    left_rear_wheel_rotation = fmod(left_rear_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    SetAnimation(anim_left_rear_wheel_rotation, left_rear_wheel_rotation);

    SetAnimation(anim_left_rear_wheel_travel, 0.5 + rear_left_displacement / (2.0*travel));
}