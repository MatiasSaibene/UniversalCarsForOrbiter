/////////////////////////////////////////////////////
//          Copyright (c)2024 Matías Saibene       //
//ORBITER MODULE: UNIVERSAL CARS FOR ORBITER (UCFO)//
//          Licenced under the MIT Licence         //
//              main.h  Main header file           //
/////////////////////////////////////////////////////
#pragma once

#ifndef __MAIN_H
#define __MAIN_H

#define STRICT 1

#include <cmath>
#include <cstddef>
#include <exception>
#include <sys/types.h>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <limits>

#include "HEADERS//Orbitersdk.h"
#include "HEADERS//OrbiterAPI.h"
#include "HEADERS//ModuleAPI.h"
#include "HEADERS//VesselAPI.h"
#include "HEADERS//XRSound.h"
#include "HEADERS//GraphicsAPI.h"

const double UCFO_ISP = 2e4;

const double UCFO_FUEL_MASS = 50; //Fuel mass in kg.

const VECTOR3 ENGINE_LOCATION = {0, 0, 0};

const VECTOR3 FORWARD_DIRECTION = {0, 0, 1};

const VECTOR3 BACKWARD_DIRECTION = {0, 0, -1};

static const int ntdvtx = 12;

class UCFO : public VESSEL4{

    public:

        UCFO(OBJHANDLE hVessel, int flightmodel);

        virtual ~UCFO();

        void TerminateAtError(const char *error, const char * className, const char *type);

        void DefineAnimations();
        void clbkSetClassCaps(FILEHANDLE cfg) override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs) override;
        void clbkPostCreation() override;
        void clbkPreStep(double simt, double simdt, double mjd) override;
        void clbkPostStep(double simt, double simdt, double mjd) override;
        int clbkConsumeBufferedKey(int key, bool down, char *kstate) override;
        int clbkConsumeDirectKey(char *kstate) override;

        VECTOR3 RotatePitch(VECTOR3, double);
        VECTOR3 RotateYaw(VECTOR3, double);
        VECTOR3 RotateBank(VECTOR3, double);
        VECTOR3 Rotate(VECTOR3, double, double, double);
        void NormalForce(double pitch, double yaw, double bank);
        void WheelAxis();
        void WheelVelocity(VECTOR3, VECTOR3);
        void DynamicFriction();
        void StaticFriction();
        void StickOrSkid();
        void Caster();
        void Ackermann();
        void EnginePower();
        void Brakes();
        void SetContactTouchdownPoints();
        
        
        void SetAnnotationMessages();
        void MakeAnnotationFormat();

        void AnimRightFrontWheel();
        void AnimLeftFrontWheel();
        void AnimRightRearWheel();
        void AnimLeftRearWheel();

        double UpdateLvlWheelsTrails();

        VISHANDLE vhUCFO;
        MESHHANDLE mhUCFO, mhcockpitUCFO;
        unsigned int uimesh_UCFO;
        unsigned int uimesh_Cockpit = 1;

    private:

        char MeshName[256];
        double size;
        double empty_mass;

        int front_left_wheel_id;
        int front_right_wheel_id;
        int rear_right_wheel_id;
        int rear_left_wheel_id;
        int steering_wheel_id;
        VECTOR3 front_left_wheel_pos;
        VECTOR3 front_right_wheel_pos;
        VECTOR3 rear_left_wheel_pos;
        VECTOR3 rear_right_wheel_pos;
        VECTOR3 left_headlight_pos;
        VECTOR3 right_headlight_pos;
        VECTOR3 left_tail_light_pos;
        VECTOR3 right_tail_light_pos;
        VECTOR3 left_backup_pos;
        VECTOR3 right_backup_pos;
        VECTOR3 camera_pos;
        VECTOR3 TDP1;
        VECTOR3 TDP2;
        VECTOR3 TDP3;
        VECTOR3 TDP4;
        VECTOR3 TDP5;
        VECTOR3 TDP6;
        VECTOR3 TDP7;
        VECTOR3 TDP8;

        NOTEHANDLE msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9, msg10, msg11;
        
        const char *cockpit_meshname = NULL;

        double lvlwheeltrails;

        double anim_delay = 0.5;

        unsigned int anim_steering_wheel;
        

        unsigned int anim_custom_1;
        unsigned int anim_custom_2;
        unsigned int anim_custom_3;

        unsigned int anim_right_front_wheel_rotation, anim_left_front_wheel_rotation, anim_right_rear_wheel_rotation, anim_left_rear_wheel_rotation;

        double right_rear_wheel_rotation, right_front_wheel_rotation, left_rear_wheel_rotation, left_front_wheel_rotation;

        unsigned int anim_right_front_wheel_steer, anim_left_front_wheel_steer;
        unsigned int anim_right_front_wheel_travel, anim_left_front_wheel_travel, anim_right_rear_wheel_travel,
        anim_left_rear_wheel_travel;

        THRUSTER_HANDLE th_dummy;
        THGROUP_HANDLE thg_dummy;
        BEACONLIGHTSPEC left_headlight_beacon_spec, right_headlight_beacon_spec, right_tail_light_spec, left_tail_light_spec, left_turning_wheel, right_turning_wheel, beacon, stop_light, left_backup_light_spec, right_backup_light_spec;
        SURFHANDLE skin[1];
        char skinpath[256];
        LightEmitter *left_headlight, *right_headlight, *left_tail_light_point, *right_tail_light_point;
        COLOUR4 col_white_d = {0.9,0.8,1,0};
	    COLOUR4 col_white_s = {1.9,0.8,1,0};
	    COLOUR4 col_white_a = {0,0,0,0};
        COLOUR4 col_red_d = {1, 0, 0, 1};
        COLOUR4 col_red_s = {1, 0, 0, 1};
        COLOUR4 col_red_a = {1, 0, 0, 1};
        VECTOR3 col_white = {1, 1, 1};
        VECTOR3 col_red = {1.0, 0.0, 0.0};

        double steering_angle;
        double R;
        double wheel_base;
        double angle_right;
        double angle_left;
        double wheel_track;
        double turn_radius;
        char brake_status = 'F';
        VECTOR3 vel;
        double mu_dyn = 0.7;
        double mu_sta = 1.0; //static friction coefficient (rubber on asphalt)
        double max_weight;
        double wheel_radius = 0.322;
        double main_fuel_tank_max;
        char drive_status;
        double pitch, yaw, bank;
        VECTOR3 omega;
        double x, z;
        VECTOR3 y;
        VECTOR3 angvel;
        int mass;
        PROPELLANT_HANDLE Fuel;
        double max_weight_rear, max_weight_front;
        VECTOR3 rear_right_wheel_contact, rear_left_wheel_contact, front_right_wheel_contact, front_left_wheel_contact;
        VECTOR3 max_weight_vector;
        double front_stiffness, rear_stiffness, body_stiffness;
        double front_damping, rear_damping, body_damping;
        double travel = 0.06;
        double front_left_tilt_angle, front_right_tilt_angle, rear_right_tilt_angle, rear_left_tilt_angle;
        double front_right_displacement, front_left_displacement, rear_right_displacement, rear_left_displacement;
        double front_right_wheel_force, front_left_wheel_force, rear_right_wheel_force, rear_left_wheel_force;
        VECTOR3 front_right_axle_axis, front_left_axle_axis, rear_right_axle_axis, rear_left_axle_axis;
        VECTOR3 velFR, velFL, velRR, velRL;
        VECTOR3 front_right_skid_force, front_left_skid_force, rear_right_skid_force, rear_left_skid_force;
        VECTOR3 front_right_skid_force_axis, front_left_skid_force_axis, rear_right_skid_force_axis, rear_left_skid_force_axis;
        VECTOR3 front_right_turning_force, front_left_turning_force, rear_right_turning_force, rear_left_turning_force;
        VECTOR3 front_right_turning_force_axis, front_left_turning_force_axis, rear_right_turning_force_axis, rear_left_turning_force_axis;
        VECTOR3 front_right_wheel_contact_local, front_left_wheel_contact_local, rear_right_wheel_contact_local, rear_left_wheel_contact_local;
        char FR_status, FL_status, RR_status, RL_status;
        char headlight_status;
        VECTOR3 front_right_wheel_axle, front_left_wheel_axle, rear_right_wheel_axle, rear_left_wheel_axle;

        //Displacements for lateral static force model

        double dxFR;
        double dxFL;
        double dxRR;
        double dxRL;

        double dzFR;
        double dzFL;
        double dzRR;
        double dzRL;

};

#endif //!__MAIN_H
