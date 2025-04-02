#include "main.hpp"
#include <cmath>

void UCFO::SetContact_TouchdownPoints(){

    front_stiffness = 0.5 * max_weight_front / travel;
    double front_damping = sqrt(4.0 * front_stiffness * empty_mass);

    oapiWriteLogV("front_stiffness %.2f", front_stiffness);
    oapiWriteLogV("front_damping %.2f", front_damping);

    rear_stiffness = 0.5 * max_weight_rear / travel;
    double rear_damping = sqrt(4.0 * rear_stiffness * empty_mass);

    oapiWriteLogV("rear_stiffness %.2f", rear_stiffness);
    oapiWriteLogV("rear_damping %.2f", rear_damping);

    double body_stiffness = max_weight / travel;
    double body_damping = sqrt(4.0 * body_stiffness * empty_mass);

    td_points[0] = {front_right_wheel_contact, front_stiffness, front_damping, 0.0, 0.0};
    td_points[1] = {front_left_wheel_contact, front_stiffness, front_damping, 0.0, 0.0};
    td_points[2] = {rear_right_wheel_contact, rear_stiffness, rear_damping, 0.0, 0.0};
    td_points[3] = {rear_left_wheel_contact, rear_stiffness, rear_damping, 0.0, 0.0};

    oapiWriteLogV("front_right_wheel_contact %f, %.2f, %f", front_right_wheel_contact.x, front_right_wheel_contact.y, front_right_wheel_contact.z);
    oapiWriteLogV("front_left_wheel_contact %f, %.2f, %f", front_left_wheel_contact.x, front_left_wheel_contact.y, front_left_wheel_contact.z);
    oapiWriteLogV("rear_right_wheel_contact %f, %.2f, %f", rear_right_wheel_contact.x, rear_right_wheel_contact.y, rear_right_wheel_contact.z);
    oapiWriteLogV("rear_left_wheel_contact %f, %f, %f", rear_left_wheel_contact.x, rear_left_wheel_contact.y, rear_left_wheel_contact.z);

    td_points[4] = {TDP1, body_stiffness, body_damping, 1.0, 1.0};
    td_points[5] = {TDP2, body_stiffness, body_damping, 1.0, 1.0};
    td_points[6] = {TDP3, body_stiffness, body_damping, 1.0, 1.0};
    td_points[7] = {TDP4, body_stiffness, body_damping, 1.0, 1.0};
    td_points[8] = {TDP5, body_stiffness, body_damping, 1.0, 1.0};
    td_points[9] = {TDP6, body_stiffness, body_damping, 1.0, 1.0};
    td_points[10] = {TDP7, body_stiffness, body_damping, 1.0, 1.0};
    td_points[11] = {TDP8, body_stiffness, body_damping, 1.0, 1.0};

    oapiWriteLogV("Before assigning: front_right_wheel_contact = %f, %f, %f",
        front_right_wheel_contact.x,
        front_right_wheel_contact.y,
        front_right_wheel_contact.z);

    SetTouchdownPoints(td_points, ntdvtx_td_points);

    oapiWriteLogV("After SetTouchdownPoints: td_points[0] = %f, %f, %f",
        td_points[0].pos.x,
        td_points[0].pos.y,
        td_points[0].pos.z);

}