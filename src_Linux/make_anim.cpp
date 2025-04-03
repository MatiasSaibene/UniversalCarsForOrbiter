#include "main.hpp"

void UCFO::MakeAnim_RightFrontWheel() {
    
    right_front_wheel_rotate = new MGROUP_ROTATE(
        uimesh_UCFO,
        (unsigned int *)front_right_wheel_id,
        1,
        front_right_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_right_front_wheel_rotation = CreateAnimation(0.0);
    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_right_front_wheel_rotation, 0, 1, right_front_wheel_rotate);

    right_front_wheel_travel = new MGROUP_TRANSLATE(
        uimesh_UCFO,
        (unsigned int *)front_right_wheel_id,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_right_front_wheel_travel = CreateAnimation(0.5);
     AddAnimationComponent(anim_right_front_wheel_travel, 0, 1, right_front_wheel_travel, parent);

    right_front_wheel_steer = new MGROUP_ROTATE(
        uimesh_UCFO,
        (unsigned int *)front_right_wheel_id,
        1,
        front_right_wheel_pos,
        _V(0, 1, 0),
        (float)(45 * RAD)
    );

    anim_right_front_wheel_steer = CreateAnimation(0.5);
    AddAnimationComponent(anim_right_front_wheel_steer, 0, 1, right_front_wheel_steer, parent);    
}

void UCFO::MakeAnim_LeftFrontWheel() {
    
    left_front_wheel_rotate = new MGROUP_ROTATE(
        uimesh_UCFO,
        (unsigned int *)front_left_wheel_id,
        1,
        front_left_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_left_front_wheel_rotation = CreateAnimation(0.0);
    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_left_front_wheel_rotation, 0, 1, left_front_wheel_rotate);

    left_front_wheel_travel = new MGROUP_TRANSLATE(
        uimesh_UCFO,
        (unsigned int *)front_left_wheel_id,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_left_front_wheel_travel = CreateAnimation(0.5);
     AddAnimationComponent(anim_left_front_wheel_travel, 0, 1, left_front_wheel_travel, parent);

    left_front_wheel_steer = new MGROUP_ROTATE(
        uimesh_UCFO,
        (unsigned int *)front_left_wheel_id,
        1,
        front_left_wheel_pos,
        _V(0, 1, 0),
        (float)(45 * RAD)
    );

    anim_left_front_wheel_steer = CreateAnimation(0.5);
    AddAnimationComponent(anim_left_front_wheel_steer, 0, 1, left_front_wheel_steer, parent);    
}

void UCFO::MakeAnim_RightRearWheel() {
    
    right_rear_wheel_rotate = new MGROUP_ROTATE(
        uimesh_UCFO,
        (unsigned int *)rear_right_wheel_id,
        1,
        rear_right_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_right_rear_wheel_rotation = CreateAnimation(0.0);
    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_right_rear_wheel_rotation, 0, 1, right_rear_wheel_rotate);

    right_rear_wheel_travel = new MGROUP_TRANSLATE(
        uimesh_UCFO,
        (unsigned int *)rear_right_wheel_id,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_right_rear_wheel_travel = CreateAnimation(0.5);
     AddAnimationComponent(anim_right_rear_wheel_travel, 0, 1, right_rear_wheel_travel, parent);
   
}

void UCFO::MakeAnim_LeftRearWheel() {
    
    left_rear_wheel_rotate = new MGROUP_ROTATE(
        uimesh_UCFO,
        (unsigned int *)rear_left_wheel_id,
        1,
        rear_left_wheel_pos,
        _V(1, 0, 0),
        (float)(360 * RAD)
    );

    anim_left_rear_wheel_rotation = CreateAnimation(0.0);
    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_left_rear_wheel_rotation, 0, 1, left_rear_wheel_rotate);

    left_rear_wheel_travel = new MGROUP_TRANSLATE(
        uimesh_UCFO,
        (unsigned int *)rear_left_wheel_id,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_left_rear_wheel_travel = CreateAnimation(0.5);
     AddAnimationComponent(anim_left_rear_wheel_travel, 0, 1, left_rear_wheel_travel, parent);
   
}