#include "main.h"

void UCFO::MakeAnim_LeftFrontWheel(){

    unsigned int FrontLeftWheelGrp = front_left_wheel_id;

    static MGROUP_ROTATE left_front_wheel_steer(
        uimesh_UCFO,
        &FrontLeftWheelGrp,
        1,
        front_left_wheel_axle,
        _V(0, 1, 0),
        (float)(45*RAD)
    );

    anim_left_front_wheel_steer = CreateAnimation(0.5);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_left_front_wheel_steer, 0, 1, &left_front_wheel_steer);

    auto left_front_wheel_travel = new MGROUP_TRANSLATE (
        uimesh_UCFO,
        &FrontLeftWheelGrp,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_left_front_wheel_travel = CreateAnimation(0.5);

    AddAnimationComponent(anim_left_front_wheel_travel, 0, 1, left_front_wheel_travel, parent);
    

    auto left_front_wheel_rotate = new MGROUP_ROTATE (
        uimesh_UCFO,
        &FrontLeftWheelGrp,
        1,
        front_left_wheel_axle,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_left_front_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_left_front_wheel_rotation, 0, 1, left_front_wheel_rotate, parent);

}

void UCFO::MakeAnim_RightFrontWheel(){

    unsigned int FrontRightWheelGrp = front_right_wheel_id;

    static MGROUP_ROTATE right_front_wheel_steer(
        uimesh_UCFO,
        &FrontRightWheelGrp,
        1,
        front_right_wheel_axle,
        _V(0, 1, 0),
        (float)(45*RAD)
    );

    anim_right_front_wheel_steer = CreateAnimation(0.5);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_right_front_wheel_steer, 0, 1, &right_front_wheel_steer);

    auto right_front_wheel_travel = new MGROUP_TRANSLATE (
        uimesh_UCFO,
        &FrontRightWheelGrp,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_right_front_wheel_travel = CreateAnimation(0.5);

    AddAnimationComponent(anim_right_front_wheel_travel, 0, 1, right_front_wheel_travel, parent);
    

    auto right_front_wheel_rotate = new MGROUP_ROTATE (
        uimesh_UCFO,
        &FrontRightWheelGrp,
        1,
        front_right_wheel_axle,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_right_front_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_right_front_wheel_rotation, 0, 1, right_front_wheel_rotate, parent);
    
}

void UCFO::MakeAnim_LeftRearWheel(){

    unsigned int RearLeftWheelGrp = rear_left_wheel_id;

    static MGROUP_TRANSLATE left_rear_wheel_travel(
        uimesh_UCFO,
        &RearLeftWheelGrp,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_left_rear_wheel_travel = CreateAnimation(0.5);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_left_rear_wheel_travel, 0, 1, &left_rear_wheel_travel);
    

    auto left_rear_wheel_rotate = new MGROUP_ROTATE (
        uimesh_UCFO,
        &RearLeftWheelGrp,
        1,
        rear_left_wheel_axle,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_left_rear_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_left_rear_wheel_rotation, 0, 1, left_rear_wheel_rotate, parent);

}

void UCFO::MakeAnim_RightRearWheel(){

    unsigned int RearRightWheelGrp = rear_right_wheel_id;

    static MGROUP_TRANSLATE right_rear_wheel_travel(
        uimesh_UCFO,
        &RearRightWheelGrp,
        1,
        _V(0, 2.0 * travel, 0)
    );

    anim_right_rear_wheel_travel = CreateAnimation(0.5);

    ANIMATIONCOMPONENT_HANDLE parent = AddAnimationComponent(anim_right_rear_wheel_travel, 0, 1, &right_rear_wheel_travel);
    

    auto right_rear_wheel_rotate = new MGROUP_ROTATE (
        uimesh_UCFO,
        &RearRightWheelGrp,
        1,
        rear_right_wheel_axle,
        _V(1, 0, 0),
        (float)(360*RAD)
    );

    anim_right_rear_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_right_rear_wheel_rotation, 0, 1, right_rear_wheel_rotate, parent);

}

