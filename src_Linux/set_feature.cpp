#include "main.hpp"
#include <cmath>

double UCFO::UpdateLvlWheelsTrails(){

    double speed = 0.0;
    speed = GetGroundspeed();


    if((speed > 10.0) && (GroundContact())){

        return 1.0;

    } else {

        return 0.0;

    }

}

void UCFO::SetFeature_Caster(){
    
    //Slowly returns steering angle to zero when no steering inputs applied
    if(steering_angle > 0){

        steering_angle = steering_angle - 1*RAD;

    } else if(steering_angle < 0){
        
        steering_angle = steering_angle + 1*RAD;
        
    }

}

void UCFO::SetFeature_Ackermann(){

    double R = (wheel_base / sin(steering_angle));

    if(steering_angle > 0){

        //Turning right
        angle_right = steering_angle;
        angle_left = atan(wheel_base / (R + wheel_track));
        turn_radius = R + 0.5 * wheel_track;

    } else if(steering_angle < 0){

        //Turning left
        angle_right = -atan(wheel_base / (-R + wheel_track));
        angle_left = steering_angle;
        turn_radius = R - 0.5 * wheel_track;

    } else {
         
         angle_right = 0;
         angle_left = 0;
         turn_radius = INFINITY;
    }
}

void UCFO::SetFeature_Brakes() {
    if (brake_status == 'N') {
        if (length(vel) > 0) {

            AddForce(_V(0, 0, (-vel.z/length(vel) * mu_dyn * max_weight)), _V(0, -wheel_radius, 0));
            
        }
        SetThrusterGroupLevel(THGROUP_MAIN, 0);
    }
}