#include "main.h"

void UCFO::MakeLightsHeadlights(){

    headlight_status = 'F';

    left_headlight_beacon_spec.shape = BEACONSHAPE_COMPACT;
    left_headlight_beacon_spec.pos = &left_headlight_pos;
    left_headlight_beacon_spec.col = &col_white; //RGB
    left_headlight_beacon_spec.size = 0.18;
    left_headlight_beacon_spec.falloff = 0.6;
    left_headlight_beacon_spec.period = 0; //continuously on
    left_headlight_beacon_spec.duration = 0.05;
    left_headlight_beacon_spec.tofs = 0.6;
    left_headlight_beacon_spec.active = false;


    right_headlight_beacon_spec.shape = BEACONSHAPE_COMPACT;
    right_headlight_beacon_spec.pos = &right_headlight_pos;
    right_headlight_beacon_spec.col = &col_white; //RGB
    right_headlight_beacon_spec.size = 0.18;
    right_headlight_beacon_spec.falloff = 0.6;
    right_headlight_beacon_spec.period = 0; //continuously on
    right_headlight_beacon_spec.duration = 0.05;
    right_headlight_beacon_spec.tofs = 0.6;
    right_headlight_beacon_spec.active = false;


    left_headlight = AddSpotLight(left_headlight_pos, FORWARD_DIRECTION, 300, 1e-3, 0, 2e-3, 25*RAD, 45*RAD, col_white_d, col_white_s, col_white_a);

    AddBeacon(&left_headlight_beacon_spec);


    right_headlight = AddSpotLight(right_headlight_pos, FORWARD_DIRECTION, 300, 1e-3, 0, 2e-3, 25*RAD, 45*RAD, col_white_d, col_white_s, col_white_a);

    AddBeacon(&right_headlight_beacon_spec);

}

void UCFO::MakeLightsTaillights(){

    brake_status = 'F';

    left_tail_light_spec.shape = BEACONSHAPE_COMPACT;
    left_tail_light_spec.pos = &left_tail_light_pos;
    left_tail_light_spec.col = &col_red;
    left_tail_light_spec.size = 0.18;
    left_tail_light_spec.falloff = 0.6;
    left_tail_light_spec.period = 0;
    left_tail_light_spec.duration = 0.05;
    left_tail_light_spec.tofs = 0.6;
    left_tail_light_spec.active = false;

    right_tail_light_spec.shape = BEACONSHAPE_COMPACT;
    right_tail_light_spec.pos = &left_tail_light_pos;
    right_tail_light_spec.col = &col_red;
    right_tail_light_spec.size = 0.18;
    right_tail_light_spec.falloff = 0.6;
    right_tail_light_spec.period = 0;
    right_tail_light_spec.duration = 0.05;
    right_tail_light_spec.tofs = 0.6;
    right_tail_light_spec.active = false;


    left_tail_light_point = AddSpotLight(left_tail_light_pos, BACKWARD_DIRECTION, 0.5, 1e-1, 0, 2e-1, 25*RAD, 45*RAD, col_red_d, col_red_s, col_red_a);

    AddBeacon(&left_tail_light_spec);

    right_tail_light_point = AddSpotLight(right_tail_light_pos, BACKWARD_DIRECTION, 0.5, 1e-1, 0, 2e-1, 25*RAD, 45*RAD, col_red_d, col_red_s, col_red_a);

    AddBeacon(&right_tail_light_spec);

}

void UCFO::MakeLightsBackuplights(){

    left_backup_light_spec.shape = BEACONSHAPE_COMPACT;
    left_backup_light_spec.pos = &left_backup_pos;
    left_backup_light_spec.col = &col_white; //RGB
    left_backup_light_spec.size = 0.18;
    left_backup_light_spec.period = 0; //continuously on
    left_backup_light_spec.duration = 0.05;
    left_backup_light_spec.tofs = 0.6;
    left_backup_light_spec.active = false;

    right_backup_light_spec.shape = BEACONSHAPE_COMPACT;
    right_backup_light_spec.pos = &left_backup_pos;
    right_backup_light_spec.col = &col_white; //RGB
    right_backup_light_spec.size = 0.18;
    right_backup_light_spec.period = 0; //continuously on
    right_backup_light_spec.duration = 0.05;
    right_backup_light_spec.tofs = 0.6;
    right_backup_light_spec.active = false;

    AddBeacon(&left_backup_light_spec);

    AddBeacon(&right_backup_light_spec);

}
