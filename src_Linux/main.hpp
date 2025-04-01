#pragma once

#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <cmath>
#include <sys/types.h>
#include <cstring>
#include <cstdio>

#include "HEADERS//Orbitersdk.h"

class UCFO : public VESSEL4{

    public:
        UCFO(OBJHANDLE hVessel, int flightmodel);
        
        virtual ~UCFO();

        void MakeContact_TouchdownPoints();
        void SetContact_TouchdownPoints();

        void SetFeature_Caster();
        void SetFeature_Ackermann();
        void SetFeature_Brakes();
        void SetEngine_Power();

        VECTOR3 GetHelp_RotatePitch(VECTOR3, double);
        VECTOR3 GetHelp_RotateYaw(VECTOR3, double);
        VECTOR3 GetHelp_RotateBank(VECTOR3, double);
        VECTOR3 GetHelp_Rotate(VECTOR3, double, double, double);
        void GetHelp_NormalForce();
        void GetHelp_WheelAxis();
        void GetHelp_WheelVelocity(VECTOR3, VECTOR3);
        void GetHelp_DynamicFriction();
        void GetHelp_StaticFriction();
        void GetHelp_StickOrSkid();

        void MakeAnnotation_Format();
        void SetAnnotation_Messages();

        void clbkSetClassCaps(FILEHANDLE cfg) override;
        void clbkPostCreation() override;
        void clbkPreStep(double, double, double) override;
        int clbkConsumeBufferedKey(int, bool, char *) override;
        int clbkConsumeDirectKey(char *) override;
    
    private:
        VISHANDLE vhUCFO;
        MESHHANDLE mhUCFO, mhcockpitUCFO;
        DEVMESHHANDLE dmhUCFO;
        unsigned int uimesh_UCFO;
        unsigned int uimesh_Cockpit;
        THRUSTER_HANDLE th_dummy;
        THGROUP_HANDLE thg_dummy;
        PROPELLANT_HANDLE main_fuel_tank;
        static const int ntdvtx_td_points = 12;
        TOUCHDOWNVTX td_points[ntdvtx_td_points];

        NOTEHANDLE msg1_annotation, msg2_annotation, msg3_annotation, msg4_annotation,  msg5_annotation, msg6_annotation, msg7_annotation, msg8_annotation, msg9_annotation, msg10_annotation, msg11_annotation;

        double mass;

        double empty_mass = 910.0 + 100.0;
        double main_fuel_tank_max = 25.0;
        double travel = 0.06;
        double mu_dyn = 0.7;
        double mu_sta = 1.0;
        double wheel_radius = 0.322;

        VECTOR3 front_right_wheel_contact;
        VECTOR3 front_left_wheel_contact;
        VECTOR3 rear_right_wheel_contact;
        VECTOR3 rear_left_wheel_contact;


        double wheel_base;

        double wheel_track;

        VECTOR3 front_right_wheel_axle = _V(0.586, 0.000, 1.392);
        VECTOR3 front_left_wheel_axle = _V(-0.586, 0.000, 1.392);
        VECTOR3 rear_right_wheel_axle = _V(0.586, 0.000, -1.084);
        VECTOR3 rear_left_wheel_axle = _V(-0.586, 0.000, -1.084);

        

        double dxFR;
        double dxFL;
        double dxRR;
        double dxRL;

        double dzFR;
        double dzFL;
        double dzRR;
        double dzRL;

        double steering_angle;

        char drive_status;
        char brake_status;

        double max_weight;
        double max_weight_front;
        double max_weight_rear;

        VECTOR3 TDP1;
        VECTOR3 TDP2;
        VECTOR3 TDP3;
        VECTOR3 TDP4;
        VECTOR3 TDP5;
        VECTOR3 TDP6;
        VECTOR3 TDP7;
        VECTOR3 TDP8;

        double angle_right;
        double angle_left;
        double turn_radius;

        VECTOR3 vel;

        double force;

        double pitch;
        double yaw;
        double bank;
        VECTOR3 omega;

        double front_stiffness;
        double rear_stiffness;

        VECTOR3 front_right_axle_axis;
        VECTOR3 front_left_axle_axis;
        VECTOR3 rear_right_axle_axis;
        VECTOR3 rear_left_axle_axis;

        VECTOR3 velFR;
        VECTOR3 velFL;
        VECTOR3 velRR;
        VECTOR3 velRL;

        VECTOR3 front_right_skid_force;
        double front_right_wheel_force;
        VECTOR3 front_left_skid_force;
        double front_left_wheel_force;

        VECTOR3 rear_right_skid_force;
        double rear_right_wheel_force;
        VECTOR3 rear_left_skid_force;
        double rear_left_wheel_force;

        VECTOR3 front_right_skid_force_axis;
        VECTOR3 front_left_skid_force_axis;
        VECTOR3 rear_right_skid_force_axis;
        VECTOR3 rear_left_skid_force_axis;


        VECTOR3 front_right_turning_force;
        VECTOR3 front_left_turning_force;
        VECTOR3 rear_right_turning_force;
        VECTOR3 rear_left_turning_force;

        VECTOR3 front_right_turning_force_axis;
        VECTOR3 front_left_turning_force_axis;
        VECTOR3 rear_right_turning_force_axis;
        VECTOR3 rear_left_turning_force_axis;

        VECTOR3 front_right_wheel_contact_local;
        VECTOR3 front_left_wheel_contact_local;
        VECTOR3 rear_right_wheel_contact_local;
        VECTOR3 rear_left_wheel_contact_local;

        char FR_status;
        char FL_status;
        char RR_status;
        char RL_status;

        VECTOR3 wv;

        double displacement;
        double n_rev;
        double r; //Compression ratio
        double idle_rpm;
        double max_rpm;
        
        double k; //polytropic specific heat ratio
        double cp_air; //specific heat of air J/kg K

        double HV; //lower heating value of gasoline J/kg
        double AF; //stochiometric fuel air ratio for gasoline

        //Ambient air properties
        double air_density;
        double air_temp;

        VECTOR3 gsv;



        //Determine lateral wheel forces needed to turn vehicle with no skid at low speeds

        double dt;

        //Determine angle between rotated and local contact points

        double front_right_tilt_angle;
        double front_left_tilt_angle;
        double rear_right_tilt_angle;
        double rear_left_tilt_angle;

        //Determine strut displacement from tilt angle using right triangle

        double front_right_displacement;
        double front_left_displacement;
        double rear_right_displacement;
        double rear_left_displacement;
};

#endif //!__MAIN_HPP