#include "main.h"

void UCFO::SetEngine_Power(){

    //VW Thing engine specs

    double displacement = 1.584e-3; //Engine displacement per cycle in cubic meters
    int n_rev = 2; //Shaft revolutions per cycle (1 for 2-stroke, 2 for 4-stroke)
    double r = 7.5; //Compression ratio
    int idle_rpm = 700;
    int max_rpm = 5000;

    //Ambient air properties

    double air_density = GetAtmDensity();
    double air_temp = GetAtmTemperature();
    double k = 1.113; //Polytropic specific heat ratio
    int cp_air = 1005; //Specific heat of air J/kg K

    //Fuel properties
    double HV = 45e+6; //Lower heating value of gasoline J/kg
    double AF = 14.7; //Stochiometric fuel air ratio for gasoline

    double throttle = GetThrusterLevel(th_dummy);

    double rpm = idle_rpm + throttle * (max_rpm - idle_rpm);

    double omega = rpm * (2 * PI) / 60;

    double mass_flow_air = air_density * displacement * (rpm / n_rev) * (1.0 / 60); //Air flow in kg/s
    double mass_flow_gas= mass_flow_air / AF;

    double QH = mass_flow_gas * HV;

    double T1 = air_temp;
    double T2 = T1 * (pow((r),(k-1)));
    double T3 = T2 + ((QH) / (mass_flow_air * cp_air));
    double T4 = T3 * (pow((1/r), (k-1)));

    double QL = mass_flow_air * cp_air * (T4-T1);

    double power = QH - QL;
    double torque = power / omega;

    double gph = 0.26 * mass_flow_gas * 3600;

    VECTOR3 omega_wheel_aux;
    GetGroundspeedVector(FRAME_LOCAL, omega_wheel_aux);

    double omega_wheel = omega_wheel_aux.z / wheel_radius;

    double force;

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

    SetPropellantMass(Fuel, GetPropellantMass(Fuel) - mass_flow_gas * oapiGetSimStep());

}