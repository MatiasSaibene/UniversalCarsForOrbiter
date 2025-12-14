#include "main.hpp"

void UCFO::MakeContact_TouchdownPoints(){

    front_right_wheel_contact.y = -wheel_radius - travel;
    front_left_wheel_contact.y = -wheel_radius - travel;
    rear_right_wheel_contact.y = -wheel_radius - travel;
    rear_left_wheel_contact.y = -wheel_radius - travel;
   
}