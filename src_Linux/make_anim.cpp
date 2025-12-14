#include "main.hpp"

void UCFO::MakeAnim_RightFrontWheel(){

    static MGROUP_ROTATE right_front_wheel_rotate(
        uimesh_UCFO,
        &front_right_wheel_id,
        1,
        front_right_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_right_front_wheel_rotation = CreateAnimation(0.0);

    AddAnimationComponent(anim_right_front_wheel_rotation, 0, 1, &right_front_wheel_rotate);
     
}

void UCFO::MakeAnim_RightRearWheel(){
    
    static MGROUP_ROTATE right_rear_wheel_rotate(
        uimesh_UCFO,
        &rear_right_wheel_id,
        1,
        rear_right_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_right_rear_wheel_rotation = CreateAnimation(0.0);

    AddAnimationComponent(anim_right_rear_wheel_rotation, 0, 1, &right_rear_wheel_rotate);
   
}

void UCFO::MakeAnim_LeftRearWheel() {
    
    static MGROUP_ROTATE left_rear_wheel_rotate(
        uimesh_UCFO,
        &rear_left_wheel_id,
        1,
        rear_left_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_left_rear_wheel_rotation = CreateAnimation(0.0);

    AddAnimationComponent(anim_left_rear_wheel_rotation, 0, 1, &left_rear_wheel_rotate);
   
}

void UCFO::MakeAnim_LeftFrontWheel() {

    static MGROUP_ROTATE left_front_wheel_rotate(
        uimesh_UCFO,
        &front_left_wheel_id,
        1,
        front_left_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_left_front_wheel_rotation = CreateAnimation(0.0);

    AddAnimationComponent(anim_left_front_wheel_rotation, 0, 1, &left_front_wheel_rotate);
    
}