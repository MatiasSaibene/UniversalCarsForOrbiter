#include "main.hpp"
#include <algorithm>
#include <cmath>

void UCFO::SetEngine_Power(){

    //VW Thing engine specs

    
    //Ambient air properties
    air_density = GetAtmDensity();
    air_temp = GetAtmTemperature();
    k= 1.113; //polytropic specific heat ratio
    cp_air = 1005; //specific heat of air J/kg K

    if (air_density <= 0 || air_temp <= 0) {
        oapiWriteLog("Error: air_density or air_temp are invalid.");
        return; // Evita cálculos con datos erróneos
    }

    //Fuel properties

   

    double throttle = GetThrusterLevel(th_dummy);

    double rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
    double omega = rpm * (2 * PI) / 60;

    double mass_flow_air = air_density * displacement * (rpm/n_rev) * (1.0/60); //air flow in kg/s
    double mass_flow_gas = mass_flow_air / AF; //fuel flow in kg/s

    double QH = mass_flow_gas * HV;

    double T1 = air_temp;
    double T2 = T1 * (pow((r), (k-1.0)));
    double T3 = T2 + (QH) / (mass_flow_air * cp_air);
    double T4 = T3 * (pow((1.0/r), (k-1.0)));

    double QL = mass_flow_air * cp_air * (T4-T1);

    double power = QH - QL;
    double torque = power / omega;

    double ghp = 0.26 * mass_flow_gas * 3600;

    
    GetGroundspeedVector(FRAME_REFLOCAL, gsv);
    double omega_wheel = gsv.z / wheel_radius;

    if(omega_wheel <= 0){

        force = throttle * mu_dyn * max_weight_rear;

    } else if(omega_wheel > 0){

        force = std::min((torque / wheel_radius) * (omega / omega_wheel), throttle * mu_dyn * max_weight_rear);

    }

    if(GroundContact()){

        if(drive_status == 'F'){

            AddForce(_V(0, 0, 0.5 * force), rear_right_wheel_contact);

            AddForce(_V(0, 0, 0.5 * force), rear_left_wheel_contact);

        } else if(drive_status == 'R'){

            AddForce(_V(0, 0, -0.5 * force), rear_right_wheel_contact);
            
            AddForce(_V(0, 0, -0.5 * force), rear_left_wheel_contact);
        }
    }

    SetPropellantMass(main_fuel_tank, GetPropellantMass(main_fuel_tank) - mass_flow_gas * oapiGetSimStep());

}