#include "main.hpp"

void UCFO::SetAnim_RightFrontWheel(){

    //Determine rotation speed of wheel

    VECTOR3 speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    // Actualización de la rotación de la rueda
    right_front_wheel_rotation = fmod(right_front_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    //SetAnimation(anim_right_front_wheel_travel, 0.5 + front_right_displacement / (2.0*travel));

    SetAnimation(anim_right_front_wheel_steer, 0.5 + angle_right / (45*RAD));

    SetAnimation(anim_right_front_wheel_rotation, right_front_wheel_rotation);


}


void UCFO::SetAnim_LeftFrontWheel(){

    //Determine rotation speed of wheel

    VECTOR3 speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    // Actualización de la rotación de la rueda
    left_front_wheel_rotation = fmod(left_front_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    //SetAnimation(anim_left_front_wheel_travel, 0.5 + front_left_displacement / (2.0*travel));

    SetAnimation(anim_left_front_wheel_steer, 0.5 + angle_left / (45*RAD));

    SetAnimation(anim_left_front_wheel_rotation, left_front_wheel_rotation);


}

void UCFO::SetAnim_LeftRearWheel(){

    //Determine rotation speed of wheel

    VECTOR3 speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    // Actualización de la rotación de la rueda
    left_rear_wheel_rotation = fmod(left_rear_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    //SetAnimation(anim_left_front_wheel_travel, 0.5 + front_left_displacement / (2.0*travel));

    SetAnimation(anim_left_rear_wheel_rotation, left_rear_wheel_rotation);


}

void UCFO::SetAnim_RightRearWheel(){

    //Determine rotation speed of wheel

    VECTOR3 speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, speed);

    double rotation_speed = speed.z / (2 * PI * wheel_radius);

    // Actualización de la rotación de la rueda
    right_rear_wheel_rotation = fmod(right_rear_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    //SetAnimation(anim_left_front_wheel_travel, 0.5 + front_left_displacement / (2.0*travel));

    SetAnimation(anim_right_rear_wheel_rotation, right_rear_wheel_rotation);


}