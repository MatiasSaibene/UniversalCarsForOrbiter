#include "main.hpp"

VECTOR3 UCFO::GetHelp_RotatePitch(VECTOR3 point, double pitch){

    /* --function to rotate point vector around X-axis in vessel coordinates into relative coordinates
    --IN LEFT HAND COORDINATE SYSTEM!!! 

    --populate rotation matrix */

    double m11 = 1.0;
    double m12 = 0.0;
    double m13 = 0.0;

    double m21 = 0.0;
    double m22 = cos(pitch);
    double m23 = sin(pitch);

    double m31 = 0.0;
    double m32 = -sin(pitch);
    double m33 = cos(pitch);

    MATRIX3 rot_matrix = {
            m11, m12, m13,
            m21, m22, m23,
            m31, m32, m33,
    };

    VECTOR3 rotated_point = mul(rot_matrix, point);

    return rotated_point;
}

VECTOR3 UCFO::GetHelp_RotateYaw(VECTOR3 point, double yaw){

    /* --function to rotate point vector around Y-axis in vessel coordinates into relative coordinates
    --IN LEFT HAND COORDINATE SYSTEM!!! 

    --populate rotation matrix */

    double m11 = cos(yaw);
    double m12 = 0.0;
    double m13 = -sin(yaw);

    double m21 = 0.0;
    double m22 = 1.0;
    double m23 = 0.0;

    double m31 = sin(yaw);
    double m32 = 0.0;
    double m33 = cos(yaw);

    MATRIX3 rot_matrix = {
            m11, m12, m13,
            m21, m22, m23,
            m31, m32, m33
    };

    VECTOR3 rotated_point = mul(rot_matrix, point);

    return rotated_point;

}

VECTOR3 UCFO::GetHelp_RotateBank(VECTOR3 point, double bank){

    /* --function to rotate point vector around Y-axis in vessel coordinates into relative coordinates 
    --IN LEFT HAND COORDINATE SYSTEM!!!

    --populate rotation matrix */

    double m11 = cos(bank);
    double m12 = sin(bank);
    double m13 = 0.0;

    double m21 = -sin(bank);
    double m22 = cos(bank);
    double m23 = 0.0;

    double m31 = 0.0;
    double m32 = 0.0;
    double m33 = 1.0;

    MATRIX3 rot_matrix = {
            m11, m12, m13,
            m21, m22, m23,
            m31, m32, m33};

    VECTOR3 rotated_point = mul(rot_matrix, point);

    return rotated_point;

}

VECTOR3 UCFO::GetHelp_Rotate(VECTOR3 point, double pitch, double yaw, double bank){
    

    VECTOR3 rotated_point = GetHelp_RotateYaw(point, yaw);

    rotated_point = GetHelp_RotatePitch(rotated_point, pitch);

    rotated_point = GetHelp_RotateBank(rotated_point, bank);
    

    return rotated_point;

}

void UCFO::GetHelp_NormalForce(){

    //Determine location of vehicle contact points in rotated vessel frame

    front_right_wheel_contact_local = GetHelp_Rotate(front_right_wheel_contact, pitch, yaw, bank);
    front_left_wheel_contact_local = GetHelp_Rotate(front_left_wheel_contact, pitch, yaw, bank);

    rear_right_wheel_contact_local = GetHelp_Rotate(rear_right_wheel_contact, pitch, yaw, bank);
    rear_left_wheel_contact_local = GetHelp_Rotate(rear_left_wheel_contact, pitch, yaw, bank);

    //Determine angle between rotated and local contact points

    front_right_tilt_angle = (front_right_wheel_contact_local.y / abs(front_right_wheel_contact_local.y)) * acos(dotp(front_right_wheel_contact, front_right_wheel_contact_local) / length(front_right_wheel_contact) * length(front_right_wheel_contact_local));


    front_left_tilt_angle = (front_left_wheel_contact_local.y / abs(front_left_wheel_contact_local.y)) * acos(dotp(front_left_wheel_contact, front_left_wheel_contact_local) / (length(front_left_wheel_contact) * length(front_left_wheel_contact_local)));


    rear_right_tilt_angle = (rear_right_wheel_contact_local.y / abs(rear_right_wheel_contact_local.y)) * acos(dotp(rear_right_wheel_contact, rear_right_wheel_contact_local) / (length(rear_right_wheel_contact) * length(rear_right_wheel_contact_local)));


    rear_left_tilt_angle = (rear_left_wheel_contact_local.y / abs(rear_left_wheel_contact_local.y)) * acos(dotp(rear_left_wheel_contact, rear_left_wheel_contact_local) / (length(rear_left_wheel_contact) * length(rear_left_wheel_contact_local)));


    //Determine strut displacement from tilt angle using right triangle

    front_right_displacement = length(front_right_wheel_contact) * sin(front_right_tilt_angle);

    front_left_displacement = length(front_left_wheel_contact) * sin(front_left_tilt_angle);

    rear_right_displacement = length(rear_right_wheel_contact) * sin(rear_right_tilt_angle);

    rear_left_displacement = length(rear_left_wheel_contact) * sin(rear_left_tilt_angle);


    //Determine normal force on each wheel based on displacement and touchdown point stiffnesses

    front_right_wheel_force = max((-front_right_displacement + travel) * front_stiffness, 0.0);

    front_left_wheel_force  = max((-front_left_displacement + travel) * front_stiffness, 0.0);

    rear_right_wheel_force  = max((-rear_right_displacement + travel) * rear_stiffness, 0.0);

    rear_left_wheel_force   = max((-rear_left_displacement + travel) * rear_stiffness, 0.0);

}

void UCFO::GetHelp_WheelAxis(){

    //Determine direction of each wheel axis
    front_right_axle_axis = _V(cos(angle_right), 0, -sin(angle_right));

    front_left_axle_axis = _V(cos(angle_left), 0, -sin(angle_left));

    rear_right_axle_axis = _V(1, 0, 0);

    rear_left_axle_axis = _V(1, 0, 0);

}

void UCFO::GetHelp_WheelVelocity(VECTOR3 vel, VECTOR3 omega){

    //Determine relative speed of contact points to ground accounting for angular rotation speed

    velFR = operator+(vel, crossp(front_right_wheel_contact, omega));

    velFL = operator+(vel, crossp(front_left_wheel_contact, omega));

    velRR = operator+(vel, crossp(rear_right_wheel_contact, omega));

    velRL = operator+(vel, crossp(rear_left_wheel_contact, omega));

}

void UCFO::GetHelp_DynamicFriction(){

    //Determine dynamic friction force vector on all wheels in vessel coordinates

    /* front_right_skid_force = operator*(velFR, -mu_dyn * front_right_wheel_force / length(velFR)); */

    double len_velFR = length(velFR);
    if (len_velFR > 1e-6) {  // Evita divisiones por valores pequeños
        front_right_skid_force = operator*(velFR, -mu_dyn * front_right_wheel_force / len_velFR);
    } else {
        front_right_skid_force = _V(0, 0, 0);
    }

    /* front_left_skid_force = operator*(velFL, -mu_dyn * front_left_wheel_force / length(velFL)); */

    double len_velFL = length(velFL);
    if (len_velFL > 1e-6) {  // Evita divisiones por valores pequeños
        front_left_skid_force = operator*(velFL, -mu_dyn * front_left_wheel_force / len_velFL);
    } else {
        front_left_skid_force = _V(0, 0, 0);
    }

    /* rear_right_skid_force = operator*(velRR, -mu_dyn * rear_right_wheel_force / length(velRR)); */

    double len_velRR = length(velRR);
    if (len_velRR > 1e-6) {  // Evita divisiones por valores pequeños
        rear_right_skid_force = operator*(velRR, -mu_dyn * rear_right_wheel_force / len_velRR);
    } else {
        rear_right_skid_force = _V(0, 0, 0);
    }

    /* rear_left_skid_force = operator*(velRL, -mu_dyn * rear_left_wheel_force / length(velRL)); */

    double len_velRL = length(velRL);
    if (len_velRL > 1e-6) {  // Evita divisiones por valores pequeños
        rear_left_skid_force = operator*(velRL, -mu_dyn * rear_left_wheel_force / len_velRL);
    } else {
        rear_left_skid_force = _V(0, 0, 0);
    }

    //Get lateral component of dynamic friction force vector in wheel coordinates

    front_right_skid_force_axis = front_right_axle_axis * dotp(front_right_skid_force, front_right_axle_axis);

    front_left_skid_force_axis = front_left_axle_axis * dotp(front_left_skid_force, front_left_axle_axis);

    rear_right_skid_force_axis = rear_right_axle_axis * dotp(rear_right_skid_force, rear_right_axle_axis);

    rear_left_skid_force_axis = rear_left_axle_axis * dotp(rear_left_skid_force, rear_left_axle_axis);

}

void UCFO::GetHelp_StaticFriction(){

    //Determine lateral wheel forces needed to turn vehicle with no skid at low speeds

    dt = oapiGetSimStep();

    double shear_modulus = 1e5; //shear modulus of rubber (Pa)
    double contact_area = 0.02;  //Square meters
    double tread_depth = 0.01; //1cm tread depth

    double friction_stiffness = shear_modulus * contact_area / tread_depth;

    double friction_damping = sqrt(mass * friction_stiffness); //~ Critically damped spring

    double urr = 0.5;

    dxFR = urr*dxFR + velFR.x*dt;
    dxFL = urr*dxFL + velFL.x*dt;
    dxRR = urr*dxRR + velRR.x*dt;
    dxRL = urr*dxRL + velRL.x*dt;

    dzFR = urr*dzFR + velFR.z*dt;
    dzFL = urr*dzFL + velFL.z*dt;
    dzRR = urr*dzRR + velRR.z*dt;
    dzRL = urr*dzRL + velRL.z*dt;


    if(front_right_wheel_force > 0){

        front_right_turning_force = _V(-friction_stiffness * dxFR - friction_damping * velFR.x, 0, -friction_stiffness * dzFR -  friction_damping * velFR.z);

    } else {

        front_right_turning_force = _V(0, 0, 0);

    }


    if(front_left_wheel_force > 0){

        front_left_turning_force = _V(-friction_stiffness * dxFL - friction_damping * velFL.x, 0, -friction_stiffness * dzFL - friction_damping * velFL.z);

    } else {

        front_left_turning_force = _V(0, 0, 0);

    }

    if(rear_right_wheel_force > 0){

        rear_right_turning_force = _V(-friction_stiffness * dxRR - friction_damping * velRR.x, 0, -friction_stiffness * dzRR - friction_damping * velRR.z);

    } else {

        rear_right_turning_force = _V(0, 0, 0);

    }

    if(rear_left_wheel_force > 0){

        rear_left_turning_force = _V(-friction_stiffness * dxRL - friction_damping * velRL.x, 0, -friction_stiffness * dzRL -  friction_damping * velRL.z);

    } else {

        rear_left_turning_force = _V(0, 0, 0);

    }

    //Get lateral component of static friction force vector in wheel coordinates

    front_right_turning_force_axis = front_right_axle_axis * dotp(front_right_turning_force, front_right_axle_axis);

    front_left_turning_force_axis = front_left_axle_axis * dotp(front_left_turning_force, front_left_axle_axis);

    rear_right_turning_force_axis = rear_right_axle_axis * dotp(rear_right_turning_force, rear_right_axle_axis);

    rear_left_turning_force_axis = rear_left_axle_axis * dotp(rear_left_turning_force, rear_left_axle_axis);

}

void UCFO::GetHelp_StickOrSkid(){

    if(length(front_right_turning_force_axis) > mu_sta * front_right_wheel_force){

        AddForce(front_right_skid_force_axis, front_right_wheel_contact_local);
        FR_status = 'S';

    } else {

        AddForce(front_right_turning_force_axis, front_right_wheel_contact_local);
        FR_status = 'T';

    }

    if(length(front_left_turning_force_axis) > mu_sta * front_left_wheel_force){

        AddForce(front_left_skid_force_axis, front_left_wheel_contact_local);
        FL_status = 'S';

    } else {
        
        AddForce(front_left_turning_force_axis, front_left_wheel_contact_local);
        FL_status = 'T';

    }

    if(length(rear_right_turning_force_axis) > mu_sta * rear_right_wheel_force){

        AddForce(rear_right_skid_force_axis, rear_right_wheel_contact_local);
        RR_status = 'S';

    } else {

        AddForce(rear_right_turning_force_axis, rear_right_wheel_contact_local);
        RR_status = 'T';

    }

    if(length(rear_left_turning_force_axis) > mu_sta * rear_left_wheel_force){

        AddForce(rear_left_skid_force_axis, rear_left_wheel_contact_local);
        RL_status = 'S';

    } else {

        AddForce(rear_left_turning_force_axis, rear_left_wheel_contact_local);
        RL_status = 'T';

    }


}
