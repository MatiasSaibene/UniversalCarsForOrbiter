#include "main.hpp"

void UCFO::SetLightHeadlights(){

    if(headlight_status == 'N'){

        left_headlight->Activate(true);

        right_headlight->Activate(true);


        left_headlight_beacon_spec.active = true;

        right_headlight_beacon_spec.active = true;

    } else {

        left_headlight->Activate(false);

        right_headlight->Activate(false);


        left_headlight_beacon_spec.active = false;

        right_headlight_beacon_spec.active = false;

    }

}

void UCFO::SetLightBrakelights(){

    if(brake_status == 'N'){

        left_tail_light_point->Activate(true);

        right_tail_light_point->Activate(true);


        left_tail_light_spec.active = true;

        right_tail_light_spec.active = true;

    } else {

        left_tail_light_point->Activate(false);

        right_tail_light_point->Activate(false);


        left_tail_light_spec.active = false;

        right_tail_light_spec.active = false;
        
    }

}

void UCFO::SetLightBackuplights(){

    if(drive_status == 'R'){

        left_backup_light_point->Activate(true);

        right_backup_light_point->Activate(true);


        left_backup_light_spec.active = true;

        right_backup_light_spec.active = true;

    } else {

        left_backup_light_point->Activate(false);

        right_backup_light_point->Activate(false);


        left_backup_light_spec.active = false;

        right_backup_light_spec.active = false;
        
    }

}