#include "main.hpp"
#include <cmath>

void UCFO::SetContact_TouchdownPoints(){

    front_stiffness = 0.5 * max_weight_front / travel;
    double front_damping = sqrt(4.0 * front_stiffness * empty_mass);

    rear_stiffness = 0.5 * max_weight_rear / travel;
    double rear_damping = sqrt(4.0 * rear_stiffness * empty_mass);

    double body_stiffness = max_weight / travel;
    double body_damping = sqrt(4.0 * body_stiffness * empty_mass);

    td_points[0] = {front_right_wheel_contact, front_stiffness, front_damping, 0.0, 0.0};
    td_points[1] = {front_left_wheel_contact, front_stiffness, front_damping, 0.0, 0.0};
    td_points[2] = {rear_right_wheel_contact, rear_stiffness, rear_damping, 0.0, 0.0};
    td_points[3] = {rear_left_wheel_contact, rear_stiffness, rear_damping, 0.0, 0.0};

    td_points[4] = {TDP1, body_stiffness, body_damping, 1.0, 1.0};
    td_points[5] = {TDP2, body_stiffness, body_damping, 1.0, 1.0};
    td_points[6] = {TDP3, body_stiffness, body_damping, 1.0, 1.0};
    td_points[7] = {TDP4, body_stiffness, body_damping, 1.0, 1.0};
    td_points[8] = {TDP5, body_stiffness, body_damping, 1.0, 1.0};
    td_points[9] = {TDP6, body_stiffness, body_damping, 1.0, 1.0};
    td_points[10] = {TDP7, body_stiffness, body_damping, 1.0, 1.0};
    td_points[11] = {TDP8, body_stiffness, body_damping, 1.0, 1.0};

    SetTouchdownPoints(td_points, ntdvtx_td_points);

}