///////////////////////////////////////////////////////
//Copyright (c)2024 Thunder Chicken & Matías Saibene //
//ORBITER MODULE: UNIVERSAL CARS FOR ORBITER (UCFO)  //
//      Licenced under the MIT Licence               //
// main.cpp  Main implementation file                //
///////////////////////////////////////////////////////
#define NOMINMAX
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#define ORBITER_MODULE
#include "main.h"

//Constructor
UCFO::UCFO(OBJHANDLE hVessel, int flightmodel) : 
VESSEL4(hVessel, flightmodel){

	mhUCFO = NULL;

    vhUCFO = NULL;

    mhcockpitUCFO = NULL;

    front_left_wheel_id = 0;

    front_right_wheel_id = 0;

    rear_right_wheel_id = 0;

    rear_left_wheel_id = 0;

    steering_wheel_id = 0;

    msg1 = NULL;

    msg2 = NULL;
    
    msg3 = NULL;
    
    msg4 = NULL;
    
    msg5 = NULL;
    
    msg6 = NULL;
    
    msg7 = NULL;
    
    msg8 = NULL;
    
    msg9 = NULL;
    
    msg10 = NULL;

    msg11 = NULL;

    lvlwheeltrails = 0.0;

    anim_steering_wheel = 0.0;

    anim_custom_1 = 0.0;

    anim_custom_2 = 0.0;

    anim_custom_3 = 0.0;

    anim_right_front_wheel_rotation = 0;

    anim_left_front_wheel_rotation = 0;

    anim_right_rear_wheel_rotation = 0.0;

    anim_left_rear_wheel_rotation = 0.0;

    anim_right_front_wheel_steer = 0.0;

    anim_left_front_wheel_steer = 0.0;

    anim_right_front_wheel_travel = 0.0;

    anim_left_front_wheel_travel = 0.0;

    anim_right_rear_wheel_travel = 0.0;

    anim_left_rear_wheel_travel = 0.0;

    th_dummy = NULL;

    thg_dummy = NULL;

    R = 0.0;

    main_fuel_tank_max = 0.0;

    x = 0.0;

    z = 0.0;

    FR_status = '\0';

    FL_status = '\0';

    RR_status = '\0';

    RL_status = '\0';

    dxFR = 0.0;

    dxFL = 0.0;

    dxRR = 0.0;

    dxRL = 0.0;

    dzFR = 0.0;

    dzFL = 0.0;

    dzRR = 0.0;

    dzRL = 0.0;

    uimesh_UCFO = 0;

	wheel_base = 0.0;

	wheel_track = 0.0;

    rear_right_wheel_contact = _V(0, 0, 0);
    
    rear_left_wheel_contact = _V(0, 0, 0);
    
    front_right_wheel_contact = _V(0, 0, 0);
    
    front_left_wheel_contact = _V(0, 0, 0);
    
	wheel_base = front_right_wheel_contact.z - rear_right_wheel_contact.x;

    wheel_track = front_right_wheel_contact.x - front_left_wheel_contact.x;

    front_stiffness = 0.0;

	front_damping = 0.0;

	rear_stiffness = 0.0;

	rear_damping = 0.0;

	body_stiffness = 0.0;

	body_damping = 0.0;

	drive_status = 'F';

	empty_mass = 0.0;

	max_weight_front = 0.0;

	max_weight = 0.0;

	max_weight_vector = _V(0, 0, 0);

	max_weight_rear = 0.0;

	MeshName[0] = '\0';

	size = 0.0;
	
    steering_angle = 0.0;

    turn_radius = 0.0;

    angle_left = 0.0;

    angle_right = 0.0;

    right_rear_wheel_rotation = 0.0; 
    
    right_front_wheel_rotation = 0.0;
    
    left_rear_wheel_rotation = 0.0; 
    
    left_front_wheel_rotation = 0.0;

    front_right_skid_force_axis = _V(0, 0, 0);
    
    front_left_skid_force_axis = _V(0, 0, 0); 
    
    rear_right_skid_force_axis = _V(0, 0, 0);
    
    rear_left_skid_force_axis = _V(0, 0, 0);

    front_right_wheel_force = 0.0; 
    
    front_left_wheel_force = 0.0; 
    
    rear_right_wheel_force = 0.0;
    
    rear_left_wheel_force = 0.0;

    front_right_skid_force = _V(0, 0, 0);
    
    front_left_skid_force = _V(0, 0, 0);
    
    rear_right_skid_force = _V(0, 0, 0);
    
    rear_left_skid_force = _V(0, 0, 0);

    velFR = _V(0, 0, 0);
    
    velFL = _V(0, 0, 0);
    
    velRR = _V(0, 0, 0);
    
    velRL = _V(0, 0, 0);

    pitch = 0.0;
    
    yaw = 0.0;
    
    bank = 0.0;

    front_right_turning_force_axis = _V(0, 0, 0);
    
    front_left_turning_force_axis = _V(0, 0, 0);
    
    rear_right_turning_force_axis = _V(0, 0, 0);
    
    rear_left_turning_force_axis = _V(0, 0, 0);

    front_right_wheel_contact_local = _V(0, 0, 0);
    
    front_left_wheel_contact_local = _V(0, 0, 0);
    
    rear_right_wheel_contact_local = _V(0, 0, 0);
    
    rear_left_wheel_contact_local = _V(0, 0, 0);

    mass = 0.0;

    omega = _V(0, 0, 0);

    front_left_tilt_angle = 0.0;

    front_right_tilt_angle = 0.0;
    
    rear_right_tilt_angle = 0.0; 
    
    rear_left_tilt_angle = 0.0;

    front_right_displacement = 0.0;
    
    front_left_displacement = 0.0;
    
    rear_right_displacement = 0.0; 
    
    rear_left_displacement = 0.0;

    front_right_axle_axis = _V(0, 0, 0);

    front_left_axle_axis = _V(0, 0, 0);

    rear_left_axle_axis = _V(0, 0, 0);

    rear_right_axle_axis = _V(0, 0, 0);

    left_headlight_beacon_spec = {
        0,
        nullptr,
        nullptr,
        0,
        0,
        0,
        0,
        false
    };

    right_headlight_beacon_spec = {
        0,
        nullptr,
        nullptr,
        0,
        0,
        0,
        0,
        false
    };

    left_tail_light_spec = {
        0,
        nullptr,
        nullptr,
        0,
        0,
        0,
        0,
        false
    };

    right_tail_light_spec = {
        0,
        nullptr,
        nullptr,
        0,
        0,
        0,
        0,
        false
    };

    left_backup_light_spec = {
        0,
        nullptr,
        nullptr,
        0,
        0,
        0,
        0,
        false
    };

    right_backup_light_spec = {
        0,
        nullptr,
        nullptr,
        0,
        0,
        0,
        0,
        false
    };

    left_headlight = nullptr;

    right_headlight = nullptr;

    left_tail_light_point = nullptr;

    right_tail_light_point = nullptr;

    left_backup_light_point = nullptr;

    right_backup_light_point = nullptr;

    headlight_status = '\0';

    Fuel = nullptr;

    flw_parent1 = nullptr;
    
    flw_parent2 = nullptr;
    
    frw_parent1 = nullptr;
    
    frw_parent2 = nullptr;
    
    rrw_parent1 = nullptr;
    
    rlw_parent1 = nullptr;

    left_front_wheel_steer = nullptr;

    left_front_wheel_rotate = nullptr;
    
    right_front_wheel_steer = nullptr;
    
    right_front_wheel_rotate = nullptr;
    
    right_rear_wheel_rotate = nullptr; 
    
    left_rear_wheel_rotate = nullptr;

    left_front_wheel_travel = nullptr;
    
    right_front_wheel_travel = nullptr;
    
    right_rear_wheel_travel = nullptr;
    
    left_rear_wheel_travel = nullptr;

}

//Destructor
UCFO::~UCFO(){

}

void UCFO::DefineAnimations() {

    unsigned int mshc = GetMeshCount();

    //Wheels groups
    unsigned int FrontLeftWheelGrp[1] = {static_cast<unsigned int>(front_left_wheel_id)};
    unsigned int FrontRightWheelGrp[1] = {static_cast<unsigned int>(front_right_wheel_id)};
    unsigned int RearRightWheelGrp[1] = {static_cast<unsigned int>(rear_right_wheel_id)};
    unsigned int RearLeftWheelGrp[1] = {static_cast<unsigned int>(rear_left_wheel_id)};

    // Animación: Rueda delantera izquierda
    left_front_wheel_steer = new MGROUP_ROTATE(
        mshc,
        FrontLeftWheelGrp,
        1,
        front_left_wheel_axle,
        _V(0, 1, 0), 
        (float)(45 * RAD));
        
    anim_left_front_wheel_steer = CreateAnimation(0.5);

    flw_parent1 = AddAnimationComponent(anim_left_front_wheel_steer, 0, 1, left_front_wheel_steer);

    left_front_wheel_travel = new MGROUP_TRANSLATE(
        mshc,
        FrontLeftWheelGrp,
        1,
        _V(0, 2.0 * travel, 0));


    
    anim_left_front_wheel_travel = CreateAnimation(0.5);

    flw_parent2 = AddAnimationComponent(anim_left_front_wheel_travel, 0, 1, left_front_wheel_travel, flw_parent1);

    left_front_wheel_rotate = new MGROUP_ROTATE(
        mshc,
        FrontLeftWheelGrp,
        1,
        front_left_wheel_axle,
        _V(1, 0, 0),
        (float)(360 * RAD));
        
    anim_left_front_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_left_front_wheel_rotation, 0, 1, left_front_wheel_rotate, flw_parent2);

    // Animación: Rueda delantera derecha
    right_front_wheel_steer = new MGROUP_ROTATE(
        mshc,
        FrontRightWheelGrp,
        1,
        front_right_wheel_axle,
        _V(0, 1, 0), (float)(45 * RAD));

    anim_right_front_wheel_steer = CreateAnimation(0.5);

    frw_parent1 = AddAnimationComponent(anim_right_front_wheel_steer, 0, 1, right_front_wheel_steer);

    right_front_wheel_travel = new MGROUP_TRANSLATE(
        mshc,
        FrontRightWheelGrp,
        1,
        _V(0, 2.0 * travel, 0));

    anim_right_front_wheel_travel = CreateAnimation(0.5);

    frw_parent2 = AddAnimationComponent(anim_right_front_wheel_travel, 0, 1, right_front_wheel_travel, frw_parent1);

    right_front_wheel_rotate = new MGROUP_ROTATE(
        mshc,
        FrontRightWheelGrp,
        1,
        front_right_wheel_axle,
        _V(1, 0, 0), (float)(360 * RAD));

    anim_right_front_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_right_front_wheel_rotation, 0, 1, right_front_wheel_rotate, frw_parent2);

    // Animación: Rueda trasera derecha
    right_rear_wheel_travel = new MGROUP_TRANSLATE(
        mshc,
        RearRightWheelGrp,
        1,
        _V(0, 2.0 * travel, 0));

    anim_right_rear_wheel_travel = CreateAnimation(0.5);

    rrw_parent1 = AddAnimationComponent(anim_right_rear_wheel_travel, 0, 1, right_rear_wheel_travel);

    right_rear_wheel_rotate = new MGROUP_ROTATE(
        mshc,
        RearRightWheelGrp,
        1,
        rear_right_wheel_axle,
        _V(1, 0, 0), (float)(360 * RAD));

    anim_right_rear_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_right_rear_wheel_rotation, 0, 1, right_rear_wheel_rotate, rrw_parent1);

    // Animación: Rueda trasera izquierda
    left_rear_wheel_travel = new MGROUP_TRANSLATE(
        mshc,
        RearLeftWheelGrp,
        1,
        _V(0, 2.0 * travel, 0));

    anim_left_rear_wheel_travel = CreateAnimation(0.5);

    rlw_parent1 = AddAnimationComponent(anim_left_rear_wheel_travel, 0, 1, left_rear_wheel_travel);

    left_rear_wheel_rotate = new MGROUP_ROTATE(
        mshc,
        RearLeftWheelGrp,
        1,
        rear_left_wheel_axle,
        _V(1, 0, 0), (float)(360 * RAD));

    anim_left_rear_wheel_rotation = CreateAnimation(0.5);

    AddAnimationComponent(anim_left_rear_wheel_rotation, 0, 1, left_rear_wheel_rotate, rlw_parent1);

}


void UCFO::clbkSetClassCaps(FILEHANDLE cfg){


	//Get vessel parameters from configuration file,
    //and set the physical vessel parameters.

	if(!oapiReadItem_string(cfg, "Mesh", MeshName)){
		TerminateAtError("Mesh", GetName(), "car");
	}

	SetMeshVisibilityMode (AddMesh (mhUCFO = oapiLoadMeshGlobal (MeshName)), MESHVIS_ALWAYS);


	if(!oapiReadItem_float(cfg, "Size", size)){
		TerminateAtError("Size", GetName(), "car");
	}

	SetSize(size);


	if(!oapiReadItem_float(cfg, "Mass", empty_mass)){
		TerminateAtError("Mass", GetName(), "car");
	}

	SetEmptyMass(empty_mass);




	if(!oapiReadItem_int(cfg, "FrontLeftWheelID", front_left_wheel_id)){
		TerminateAtError("FrontLeftWheelID", GetName(), "car");
	}

	oapiWriteLogV("%s: FrontLeftWheelID: %d", GetName(), front_left_wheel_id);


	if(!oapiReadItem_int(cfg, "FrontRightWheelID", front_right_wheel_id)){
		TerminateAtError("FrontRightWheelID", GetName(), "car");
	}

	oapiWriteLogV("%s: FrontRightWheelID: %d", GetName(), front_right_wheel_id);


	if(!oapiReadItem_int(cfg, "RearRightWheelID", rear_right_wheel_id)){
		TerminateAtError("RearRightWheelID", GetName(), "car");
	}

	oapiWriteLogV("%s: RearRightWheelID: %d", GetName(), rear_right_wheel_id);


	if(!oapiReadItem_int(cfg, "RearLeftWheelID", rear_left_wheel_id)){
		TerminateAtError("RearLeftWheelID", GetName(), "car");
	}

	oapiWriteLogV("%s: RearLeftWheelID: %d", GetName(), rear_left_wheel_id);


	if(!oapiReadItem_vec(cfg, "FrontLeftWheelPosition", front_left_wheel_pos)){
		TerminateAtError("FrontLeftWheelPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "FrontRightWheelPosition", front_right_wheel_pos)){
		TerminateAtError("FrontRightWheelPosition", GetName(), "car");
	}
	
	if(!oapiReadItem_vec(cfg, "RearLeftWheelPosition", rear_left_wheel_pos)){
		TerminateAtError("RearLeftWheelPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RearRightWheelPosition", rear_right_wheel_pos)){
		TerminateAtError("RearRightWheelPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "LeftHeadlightPosition", left_headlight_pos)){
		TerminateAtError("LeftHeadlightPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RightHeadlightPosition", right_headlight_pos)){
		TerminateAtError("RightHeadlightPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "LeftTailLightPosition", left_tail_light_pos)){
		TerminateAtError("LeftTailLightPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RightTailLightPosition", right_tail_light_pos)){
		TerminateAtError("RightTailLightPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "LeftBackupLightPosition", left_backup_pos)){
		TerminateAtError("LeftBackupLightPosition", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RightBackupLightPosition", right_backup_pos)){
		TerminateAtError("RightBackupLightPosition", GetName(), "car");
	}



	if(!oapiReadItem_vec(cfg, "CameraPosition", camera_pos)){
		TerminateAtError("CameraPosition", GetName(), "car");
	}

	SetCameraOffset(camera_pos);



	if(!oapiReadItem_vec(cfg, "FrontRightWheelAxle", front_right_wheel_axle)){
		TerminateAtError("FrontRightWheelAxle", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "FrontLeftWheelAxle", front_left_wheel_axle)){
		TerminateAtError("FrontLeftWheelAxle", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RearRightWheelAxle", rear_right_wheel_axle)){
		TerminateAtError("RearRightWheelAxle", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RearLeftWheelAxle", rear_left_wheel_axle)){
		TerminateAtError("RearLeftWheelAxle", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "FrontRightWheelContact", front_right_wheel_contact)){
		TerminateAtError("FrontRightWheelContact", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "FrontLeftWheelContact", front_left_wheel_contact)){
		TerminateAtError("FrontLeftWheelContact", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RearRightWheelContact", rear_right_wheel_contact)){
		TerminateAtError("RearRightWheelContact", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "RearLeftWheelContact", rear_left_wheel_contact)){
		TerminateAtError("RearLeftWheelContact", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint1", TDP1)){
		TerminateAtError("TouchdownPoint1", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint2", TDP2)){
		TerminateAtError("TouchdownPoint2", GetName(), "car");
	}
	
	if(!oapiReadItem_vec(cfg, "TouchdownPoint3", TDP3)){
		TerminateAtError("TouchdownPoint3", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint4", TDP4)){
		TerminateAtError("TouchdownPoint4", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint5", TDP5)){
		TerminateAtError("TouchdownPoint5", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint6", TDP6)){
		TerminateAtError("TouchdownPoint6", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint7", TDP7)){
		TerminateAtError("TouchdownPoint7", GetName(), "car");
	}

	if(!oapiReadItem_vec(cfg, "TouchdownPoint8", TDP8)){
		TerminateAtError("TouchdownPoint8", GetName(), "car");
	}



	Fuel = CreatePropellantResource(UCFO_FUEL_MASS);



	//Dummy thruster to provide throttle input to engine model. 1 N thrust is to allow thruster status
    //to indicate throttle position

	th_dummy = CreateThruster(ENGINE_LOCATION, FORWARD_DIRECTION, 1, Fuel, UCFO_ISP);

	thg_dummy = CreateThrusterGroup(&th_dummy, 1, THGROUP_MAIN);

	SetCW(0.5, 0.5, 0.3, 0.3);



	static PARTICLESTREAMSPEC wheel_trails = {
        0, 0.5, 5, 10, 0.03, 5, 1, 3.0, 
        PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PLIN, -1.0, 25.0,
		PARTICLESTREAMSPEC::ATM_PLIN, 
    };

	AddParticleStream(&wheel_trails, front_left_wheel_pos, BACKWARD_DIRECTION, &lvlwheeltrails);

    AddParticleStream(&wheel_trails, front_right_wheel_pos, BACKWARD_DIRECTION, &lvlwheeltrails);

    AddParticleStream(&wheel_trails, rear_left_wheel_pos, BACKWARD_DIRECTION, &lvlwheeltrails);

    AddParticleStream(&wheel_trails, rear_right_wheel_pos, BACKWARD_DIRECTION, &lvlwheeltrails);

	//DefineAnimations();

    //Light locations and specifications
    MakeLightHeadlights();
    MakeLightBackuplights();
    MakeLightTaillights();

    //Screen message formatting
    MakeAnnotationFormat();

}

void UCFO::clbkPostCreation(){

	//Following is needed to allow the suspension to be set and tuned to the correct height independent of the planetary body it is on.

	GetWeightVector(max_weight_vector);
    max_weight = length(max_weight_vector);

	max_weight_front = (max_weight / 2);

	max_weight_rear = (max_weight / 2);

    /* max_weight_front = max_weight * (-rear_right_wheel_contact.z / wheel_base); //Weight supported by two front wheells.
    max_weight_rear = (max_weight - max_weight_front); //Weight supported by two rear wheels. */

    SetContactTouchdownPoints();

}

void UCFO::clbkPreStep(double simt, double simdt, double mjd){

	//Determine strut travel for animations and to calculate normal contact forces on all wheels
	pitch = GetPitch();
	yaw = 0;
	bank = GetBank();
	GetGroundspeedVector(FRAME_LOCAL, vel);
	GetAngularVel(angvel);
    omega = _V(0, angvel.y, 0);
	mass = empty_mass + GetPropellantMass(Fuel);

    Caster();
    Ackermann();
	EnginePower();
	Brakes();

    //Get normal force on wheels for friction calculations

    NormalForce(pitch, yaw, bank);

    //Get direction of wheel axes for steering force directions

    WheelAxis();

    //Get wheel velocity relative to ground for friction vector

    WheelVelocity(vel, omega);

    DynamicFriction();

    StaticFriction();

    //Compare and apply lesser of dynamic and static friction forces on wheels

    StickOrSkid();

    //Update animations and lights

    AnimRightFrontWheel();
    AnimLeftFrontWheel();
    AnimRightRearWheel();
    AnimLeftRearWheel();

    SetLightHeadlights();
    SetLightBrakelights();
    SetLightBackuplights();

    SetAnnotationMessages();

}

VECTOR3 UCFO::RotatePitch(VECTOR3 point, double pitch) {
    // Construcción de la matriz de rotación para el pitch
    MATRIX3 rot_matrix = {
        1.0, 0.0, 0.0,
        0.0, std::cos(pitch), std::sin(pitch),
        0.0, -std::sin(pitch), std::cos(pitch)
    };

    // Multiplicación de la matriz de rotación por el punto
    return mul(rot_matrix, point);
}

VECTOR3 UCFO::RotateYaw(VECTOR3 point, double yaw) {
    // Construcción de la matriz de rotación para el yaw
    MATRIX3 rot_matrix = {
        std::cos(yaw), 0.0, std::sin(yaw),
        0.0, 1.0, 0.0,
        -std::sin(yaw), 0.0, std::cos(yaw)
    };

    // Multiplicación de la matriz de rotación por el punto
    return mul(rot_matrix, point);
}

VECTOR3 UCFO::RotateBank(VECTOR3 point, double bank) {
    // Construcción de la matriz de rotación para el bank
    MATRIX3 rot_matrix = {
        std::cos(bank), std::sin(bank), 0.0,
        -std::sin(bank), std::cos(bank), 0.0,
        0.0, 0.0, 1.0
    };

    // Multiplicación de la matriz de rotación por el punto
    return mul(rot_matrix, point);
}


VECTOR3 UCFO::Rotate(VECTOR3 point, double pitch, double yaw, double bank) {

    VECTOR3 rotated_point = RotateYaw(point, yaw);

    rotated_point = RotatePitch(rotated_point, pitch);

    rotated_point = RotateBank(rotated_point, bank);

    return rotated_point;
}


void UCFO::DynamicFriction(){

    //Determine dynamic friction force vector on all wheels in vessel coordinates

    front_right_skid_force = operator*(velFR, -mu_dyn * front_right_wheel_force / length(velFR));

    front_left_skid_force = operator*(velFL, -mu_dyn * front_left_wheel_force / length(velFL));

    rear_right_skid_force = operator*(velRR, -mu_dyn * rear_right_wheel_force / length(velRR));

    rear_left_skid_force = operator*(velRL, -mu_dyn * rear_left_wheel_force / length(velRL));


    //Get lateral component of dynamic friction force vector in wheel coordinates

    front_right_skid_force_axis = front_right_axle_axis * dotp(front_right_skid_force, front_right_axle_axis);

    front_left_skid_force_axis = front_left_axle_axis * dotp(front_left_skid_force, front_left_axle_axis);

    rear_right_skid_force_axis = rear_right_axle_axis * dotp(rear_right_skid_force,rear_right_axle_axis);

    rear_left_skid_force_axis = rear_left_axle_axis * dotp(rear_left_skid_force,rear_left_axle_axis);

}

void UCFO::StaticFriction(){

    //Determine lateral wheel forces needed to turn vehicle with no skid at low speeds

    double dt = oapiGetSimStep();

    double shear_modulus = 1e5; //shear modulus of rubber (Pa)
    double contact_area = 0.02;  //Square meters
    double tread_depth = 0.01; //1cm tread depth

    double friction_stiffness = shear_modulus * contact_area / tread_depth;

    double friction_damping = std::sqrt(mass * friction_stiffness); //~ Critically damped spring

    float urr = 0.5;

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

void UCFO::StickOrSkid(){

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

void UCFO::AnimRightFrontWheel(){

    //Determine rotation speed of wheel

    VECTOR3 l_speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, l_speed);

    double rotation_speed = 0.0;
    rotation_speed = l_speed.z / (2*PI*wheel_radius);

    // Actualización de la rotación de la rueda
    right_front_wheel_rotation = fmod(right_front_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    SetAnimation(anim_right_front_wheel_rotation, right_front_wheel_rotation);

    SetAnimation(anim_right_front_wheel_steer, 0.5 + angle_right / (45*RAD));

    SetAnimation(anim_right_front_wheel_travel, 0.5 + front_right_displacement / (2.0*travel));

}

void UCFO::AnimLeftFrontWheel(){

    VECTOR3 l_speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, l_speed);

    double rotation_speed = 0.0;
    rotation_speed = l_speed.z / (2 * PI * wheel_radius);

    left_front_wheel_rotation = fmod(left_front_wheel_rotation + oapiGetSimStep() * rotation_speed, 1.0);

    //Set the rotation animation, steering animation linked to rudder

    SetAnimation(anim_left_front_wheel_rotation, left_front_wheel_rotation);

    SetAnimation(anim_left_front_wheel_steer, 0.5 + angle_left / (45*RAD));


}

void UCFO::AnimRightRearWheel(){


    VECTOR3 l_speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, l_speed);

    double l_rotation_speed = 0.0;
    l_rotation_speed = l_speed.z / (2 * PI * wheel_radius);

    right_rear_wheel_rotation = fmod(right_rear_wheel_rotation + oapiGetSimStep() * l_rotation_speed, 1.0);

    SetAnimation(anim_right_rear_wheel_rotation, right_rear_wheel_rotation);

    SetAnimation(anim_right_rear_wheel_travel, 0.5 + rear_right_displacement / (2.0 * travel));

}

void UCFO::AnimLeftRearWheel(){

    VECTOR3 l_speed = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, l_speed);

    double l_rotation_speed = 0.0; 
    l_rotation_speed = l_speed.z / (2 * PI * wheel_radius);

    left_rear_wheel_rotation = fmod(left_rear_wheel_rotation + oapiGetSimStep() * l_rotation_speed, 1.0);

    SetAnimation(anim_right_rear_wheel_rotation, left_rear_wheel_rotation);

    SetAnimation(anim_left_rear_wheel_travel, 0.5 + rear_left_displacement / (2.0*travel));
}

void UCFO::WheelVelocity(VECTOR3 vel, VECTOR3 omega){

    //Determine relative speed of contact points to ground accounting for angular rotation speed

    velFR = operator+(vel, crossp(front_right_wheel_contact, omega));

    velFL = operator+(vel, crossp(front_left_wheel_contact, omega));

    velRR = operator+(vel, crossp(rear_right_wheel_contact, omega));

    velRL = operator+(vel, crossp(rear_left_wheel_contact, omega));

}

void UCFO::WheelAxis(){

    //Determine direction of each wheel axis
    front_right_axle_axis = _V(std::cos(angle_right), 0, -std::sin(angle_right));

    front_left_axle_axis = _V(std::cos(angle_left), 0, -std::sin(angle_left));

    rear_right_axle_axis = _V(1, 0, 0);

    rear_left_axle_axis = _V(1, 0, 0);

}

void UCFO::NormalForce(double pitch, double yaw, double bank){

    //Determine location of vehicle contact points in rotated vessel frame

    front_right_wheel_contact_local = Rotate(front_right_wheel_contact, pitch, yaw, bank);
    front_left_wheel_contact_local = Rotate(front_left_wheel_contact, pitch, yaw, bank);

    rear_right_wheel_contact_local = Rotate(rear_right_wheel_contact, pitch, yaw, bank);
    rear_left_wheel_contact_local = Rotate(rear_left_wheel_contact, pitch, yaw, bank);

    
    //Determine angle between rotated and local contact points
    front_right_tilt_angle = (front_right_wheel_contact_local.y / std::abs(front_right_wheel_contact_local.y)) * std::acos(dotp(front_right_wheel_contact, front_right_wheel_contact_local) / length(front_right_wheel_contact) * length(front_right_wheel_contact_local));

    front_left_tilt_angle = (front_left_wheel_contact_local.y / std::abs(front_left_wheel_contact_local.y)) * std::acos(dotp(front_left_wheel_contact, front_left_wheel_contact_local) / (length(front_left_wheel_contact) * length(front_left_wheel_contact_local)));

    rear_right_tilt_angle = (rear_right_wheel_contact_local.y / std::abs(rear_right_wheel_contact_local.y)) * std::acos(dotp(rear_right_wheel_contact, rear_right_wheel_contact_local) / (length(rear_right_wheel_contact) * length(rear_right_wheel_contact_local)));

    rear_left_tilt_angle = (rear_left_wheel_contact_local.y / std::abs(rear_left_wheel_contact_local.y)) * std::acos(dotp(rear_left_wheel_contact, rear_left_wheel_contact_local) / (length(rear_left_wheel_contact) * length(rear_left_wheel_contact_local)));


    //Determine strut displacement from tilt angle using right triangle

    front_right_displacement = length(front_right_wheel_contact) * std::sin(front_right_tilt_angle);

    front_left_displacement = length(front_left_wheel_contact) * std::sin(front_left_tilt_angle);

    rear_right_displacement = length(rear_right_wheel_contact) * std::sin(rear_right_tilt_angle);

    rear_left_displacement = length(rear_left_wheel_contact) * std::sin(rear_left_tilt_angle);


    //Determine normal force on each wheel based on displacement and touchdown point stiffnesses

    front_right_wheel_force = std::max((-front_right_displacement + travel) * front_stiffness, 0.0);

    front_left_wheel_force  = std::max((-front_left_displacement + travel) * front_stiffness, 0.0);

    rear_right_wheel_force  = std::max((-rear_right_displacement + travel) * rear_stiffness, 0.0);

    rear_left_wheel_force   = std::max((-rear_left_displacement + travel) * rear_stiffness, 0.0);

}

void UCFO::Caster(){
    
    //Slowly returns steering angle to zero when no steering inputs applied
    if(steering_angle > 0){

        steering_angle = steering_angle - 0.01*RAD;

    } else if(steering_angle < 0){
        
        steering_angle = steering_angle + 0.01*RAD;
        
    }

}

void UCFO::Ackermann(){

    R = wheel_base / std::sin(steering_angle);

    if(steering_angle > 0){

        //Turning right
        angle_right = steering_angle;
        angle_left = std::atan(wheel_base / (R + wheel_track));
        turn_radius = R + 0.5 * wheel_track;

    } else if(steering_angle < 0){

        //Turning left
        angle_right = -std::atan(wheel_base / (-R + wheel_track));
        angle_left = steering_angle;
        turn_radius = R - 0.5 * wheel_track;

    } else {
         
         angle_right = 0;
         angle_left = 0;
         turn_radius = std::numeric_limits<double>::infinity();

    }
}

void UCFO::EnginePower(){

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

    double l_throttle = 0.0;
    l_throttle = GetThrusterLevel(th_dummy);

    double l_rpm = idle_rpm + l_throttle * (max_rpm - idle_rpm);

    double l_omega = 0.0;
    l_omega = l_rpm * (2 * PI) / 60;

    double l_mass_flow_air = air_density * displacement * (l_rpm / n_rev) * (1.0 / 60); //Air flow in kg/s
    double l_mass_flow_gas= l_mass_flow_air / AF;

    double l_QH = l_mass_flow_gas * HV;

    double l_T1 = air_temp;
    double l_T2 = l_T1 * (pow((r),(k-1)));
    double l_T3 = l_T2 + ((l_QH) / (l_mass_flow_air * cp_air));
    double l_T4 = l_T3 * (pow((1/r), (k-1)));

    double l_QL = l_mass_flow_air * cp_air * (l_T4-l_T1);

    double l_power = l_QH - l_QL;
    double l_torque = l_power / l_omega;

    double gph = 0.26 * l_mass_flow_gas * 3600;

    VECTOR3 omega_wheel_aux = _V(0, 0, 0);
    GetGroundspeedVector(FRAME_LOCAL, omega_wheel_aux);

    double l_omega_wheel = 0.0;
    l_omega_wheel = omega_wheel_aux.z / wheel_radius;

    double force = 0.0;

    if(l_omega_wheel <= 0){
        
        force = l_throttle * mu_dyn * max_weight_rear;

    } else if(l_omega_wheel > 0){

        force = std::min((l_torque / wheel_radius) * (l_omega / l_omega_wheel), l_throttle * mu_dyn * max_weight_rear);

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

    SetPropellantMass(Fuel, GetPropellantMass(Fuel) - l_mass_flow_gas * oapiGetSimStep());

}

void UCFO::Brakes(){

	if(brake_status == 'N'){

        if(length(vel) > 0){
            AddForce((_V(0, 0, (-vel.z / length(vel)) * mu_dyn * max_weight)), _V(0, -wheel_radius, 0));
        }

        SetThrusterGroupLevel(THGROUP_MAIN, 0);

    }

}

void UCFO::SetAnnotationMessages(){

    // Velocidad en km/h
    double kph = 0.0;
    kph = 3.6 * vel.z;
    char buffer1[50];
    snprintf(buffer1, sizeof(buffer1), "%.1f km/h", kph);
    char* message1 = buffer1;

    // Angulo de giro en grados
    char buffer2[50];
    snprintf(buffer2, sizeof(buffer2), "%.1f deg turn", steering_angle * DEG);
    char* message2 = buffer2;

    char *message3 = "";
    char *message4 = "";
    char *message5 = "";
    char *message6 = "";
    char *message7 = "";
    char *message8 = "";
    char *message9 = "";
    char *message10 = "";
    char *message11 = "";
    

    if(drive_status == 'F'){
        
        message3 = "D";
        
        message4 = "";

        message5 = "";
        
    } else if(drive_status == 'N'){

        message3 = "";
            
        message4 = "N";
            
        message5 = "";

    } else if(drive_status == 'R'){

        message3 = "";

        message4 = "";

        message5 = "R";

    }

    message7 = "Steer with rudder controls";

    message8 = "Brake with spacebar";

    message9 = "Shift forward/reverse with comma (,) and period (.)";

    message10 = "Switch between forward view (F) and tire view (V)";

    message11 = "Toggle headlights with L";

    oapiAnnotationSetText(msg1, message1);
    oapiAnnotationSetText(msg2, message2);
    oapiAnnotationSetText(msg3, message3);
    oapiAnnotationSetText(msg4, message4);
    oapiAnnotationSetText(msg5, message5);
    oapiAnnotationSetText(msg6, "");
    oapiAnnotationSetText(msg7, message7);
    oapiAnnotationSetText(msg8, message8);
    oapiAnnotationSetText(msg9, message9);
    oapiAnnotationSetText(msg10, message10);
    oapiAnnotationSetText(msg11, message11);

    
}

void UCFO::MakeAnnotationFormat(){

    msg1 = oapiCreateAnnotation(true, 0.8, _V(0, 1, 0));
    oapiAnnotationSetPos(msg1, 0.01, 0.22, 0.5, 1);

    msg2 = oapiCreateAnnotation(true, 0.8, _V(0, 1, 0));
    oapiAnnotationSetPos(msg2, 0.01, 0.26, 0.5, 1);
    
    msg3 = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg3, 0.01, 0.30, 0.5, 1);

    msg4 = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg4, 0.01, 0.34, 0.5, 1);

    msg5 = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg5, 0.01, 0.38, 0.5, 1);

    msg6 = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg6, 0.01, 0.42, 0.5, 1);

    msg7 = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg7, 0.01, 0.76, 0.5, 1);

    msg8 = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg8, 0.01, 0.80, 0.5, 1);

    msg9 = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg9, 0.01, 0.84, 0.5, 1);

    msg10 = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg10, 0.01, 0.88, 0.5, 1);

    msg11 = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg11, 0.01, 0.92, 0.5, 1);

}

void UCFO::SetContactTouchdownPoints(){

	empty_mass = GetEmptyMass();

	front_stiffness = 0.5 * max_weight_front / travel;
	front_damping = std::sqrt(0.1 * front_stiffness * empty_mass);

    rear_stiffness = 0.5 * max_weight_rear / travel;
    rear_damping = std::sqrt(0.1 * rear_stiffness * empty_mass);

    body_stiffness = max_weight / travel;
    body_damping = std::sqrt(0.1 * body_stiffness * empty_mass);

	oapiWriteLogV("empty_mass: %.2f", empty_mass);

	oapiWriteLogV("max_weight_front: %.2f", max_weight_front);

	oapiWriteLogV("max_weight_rear: %.2f", max_weight_rear);

    const TOUCHDOWNVTX td_points[ntdvtx]{
        {(front_left_wheel_contact), front_stiffness, front_damping, 0.0, 0.0},
        {(front_right_wheel_contact), front_stiffness, front_damping, 0.0, 0.0},
        {(rear_left_wheel_contact), rear_stiffness, rear_damping, 0.0, 0.0},
        {(rear_right_wheel_contact), rear_stiffness, rear_damping, 0.0, 0.0},
        {(TDP1), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP2), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP3), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP4), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP5), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP6), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP7), body_stiffness, body_damping, 1.0, 1.0},
        {(TDP8), body_stiffness, body_damping, 1.0, 1.0}
    };

    SetTouchdownPoints(td_points, ntdvtx);

}

double UCFO::UpdateLvlWheelsTrails(){

    double speed = 0.0;
    speed = GetGroundspeed();


    if((speed > 10.0) && (GroundContact())){

        return 1.0;

    } else {

        return 0.0;

    }

}

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

void UCFO::MakeLightHeadlights(){

    headlight_status = 'F';

    left_headlight_beacon_spec.pos = &left_headlight_pos;
    left_headlight_beacon_spec.shape = BEACONSHAPE_STAR;
    left_headlight_beacon_spec.col = &col_white;
    left_headlight_beacon_spec.size = 0.18;
    left_headlight_beacon_spec.falloff = 0.6;
    left_headlight_beacon_spec.period = 0;
    left_headlight_beacon_spec.duration = 0.05;
    left_headlight_beacon_spec.tofs = 0.6;
    left_headlight_beacon_spec.active = false;

    right_headlight_beacon_spec.pos = &right_headlight_pos;
    right_headlight_beacon_spec.shape = BEACONSHAPE_STAR;
    right_headlight_beacon_spec.col = &col_white;
    right_headlight_beacon_spec.size = 0.18;
    right_headlight_beacon_spec.falloff = 0.6;
    right_headlight_beacon_spec.period = 0;
    right_headlight_beacon_spec.duration = 0.05;
    right_headlight_beacon_spec.tofs = 0.6;
    right_headlight_beacon_spec.active = false;

    left_headlight = AddSpotLight(left_headlight_pos, FORWARD_DIRECTION, 300, 1e-3, 0, 2e-3, 25*RAD, 45*RAD, col_white_d, col_white_s, col_white_a);

    right_headlight = AddSpotLight(right_headlight_pos, FORWARD_DIRECTION, 300, 1e-3, 0, 2e-3, 25*RAD, 45*RAD, col_white_d, col_white_s, col_white_a);

}

void UCFO::MakeLightTaillights(){

    brake_status = 'F';

    left_tail_light_spec.pos = &left_tail_light_pos;
    left_tail_light_spec.shape = BEACONSHAPE_COMPACT;
    left_tail_light_spec.col = &col_red;
    left_tail_light_spec.size = 0.18;
    left_tail_light_spec.falloff = 0.6;
    left_tail_light_spec.period = 0;
    left_tail_light_spec.duration = 0.05;
    left_tail_light_spec.tofs = 0.6;
    left_tail_light_spec.active = false;

    right_tail_light_spec.pos = &right_tail_light_pos;
    right_tail_light_spec.shape = BEACONSHAPE_COMPACT;
    right_tail_light_spec.col = &col_red;
    right_tail_light_spec.size = 0.18;
    right_tail_light_spec.falloff = 0.6;
    right_tail_light_spec.period = 0;
    right_tail_light_spec.duration = 0.05;
    right_tail_light_spec.tofs = 0.6;
    right_tail_light_spec.active = false;

    left_tail_light_point = AddSpotLight(left_tail_light_pos, BACKWARD_DIRECTION, 0.5, 1e-1, 0, 2e-1, 25*RAD, 45*RAD, col_red_d, col_red_s, col_red_a);

    right_tail_light_point = AddSpotLight(right_tail_light_pos, BACKWARD_DIRECTION, 0.5, 1e-1, 0, 2e-1, 25*RAD, 45*RAD, col_red_d, col_red_s, col_red_a);

}

void UCFO::MakeLightBackuplights(){

    left_backup_light_spec.pos = &left_tail_light_pos;
    left_backup_light_spec.shape = BEACONSHAPE_COMPACT;
    left_backup_light_spec.col = &col_white;
    left_backup_light_spec.size = 0.18;
    left_backup_light_spec.falloff = 0.6;
    left_backup_light_spec.period = 0;
    left_backup_light_spec.duration = 0.05;
    left_backup_light_spec.tofs = 0.6;
    left_backup_light_spec.active = false;

    right_backup_light_spec.pos = &right_tail_light_pos;
    right_backup_light_spec.shape = BEACONSHAPE_COMPACT;
    right_backup_light_spec.col = &col_white;
    right_backup_light_spec.size = 0.18;
    right_backup_light_spec.falloff = 0.6;
    right_backup_light_spec.period = 0;
    right_backup_light_spec.duration = 0.05;
    right_backup_light_spec.tofs = 0.6;
    right_backup_light_spec.active = false;

    left_backup_light_point = AddSpotLight(left_tail_light_pos, BACKWARD_DIRECTION, 0.5, 1e-1, 0, 2e-1, 25*RAD, 45*RAD, col_white_d, col_white_s, col_white_a);

    right_backup_light_point = AddSpotLight(right_tail_light_pos, BACKWARD_DIRECTION, 0.5, 1e-1, 0, 2e-1, 25*RAD, 45*RAD, col_white_d, col_white_s, col_white_a);

}

int UCFO::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate){

    if(key == OAPI_KEY_PERIOD && down){
        
        if(drive_status == 'F'){
            
            drive_status = 'N';

        } else if(drive_status == 'N'){

            drive_status = 'R';

        } else if(drive_status == 'R'){

            drive_status = 'R';
        }

    }

    if(key == OAPI_KEY_COMMA && down){

        if(drive_status == 'R'){

            drive_status = 'N';

        } else if(drive_status == 'N'){

            drive_status = 'F';

        } else if(drive_status == 'F'){

            drive_status = 'F';

        }

    }

    if(key == OAPI_KEY_L && down){

        if(headlight_status == 'N'){

            headlight_status = 'F';
        } else if (headlight_status == 'F'){

            headlight_status = 'N';
        }
        
    }

    if(key == OAPI_KEY_F && down){

        SetCameraOffset(camera_pos);

    }

    if(key == OAPI_KEY_V && down){

        VECTOR3 front_left_wheel_pos_aux = _V(front_left_wheel_pos.x-1.75, front_left_wheel_pos.y, front_left_wheel_pos.z-2.75);

        SetCameraOffset(front_left_wheel_pos_aux);

    }

    return 0;
}

int UCFO::clbkConsumeDirectKey(char *kstate){

    if(KEYDOWN(kstate, OAPI_KEY_SPACE)){
        
        brake_status = 'N';

    } else {
        brake_status = 'F';
    }

    if(KEYDOWN(kstate, OAPI_KEY_NUMPAD1)){

        steering_angle = std::max(steering_angle - 0.05 * RAD, -1.0);

    }

    if(KEYDOWN(kstate, OAPI_KEY_NUMPAD3)){
        
        steering_angle = std::min(steering_angle + 0.05 * RAD, 1.0);
        
    }

    return 0;
}

void UCFO::clbkLoadStateEx(FILEHANDLE scn, void *vs){

    char *line;

    while(oapiReadScenario_nextline(scn, line)){
        ParseScenarioLineEx(line, vs);
    }

}

void UCFO::clbkSaveState(FILEHANDLE scn){

    SaveDefaultState(scn);

}

void UCFO::clbkPostStep(double simt, double simdt, double mjd){

	lvlwheeltrails = UpdateLvlWheelsTrails();

}

void UCFO::TerminateAtError(const char *error, const char * className, const char *type){

	oapiWriteLogV("ERROR: UCFO cannot continue: The %s of %s %s is not specified", error, className, type);

    std::terminate();
}

DLLCLBK void InitModule(HINSTANCE hModule){

}

DLLCLBK void ExitModule(HINSTANCE *hModule){

}

///////////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    
	return new UCFO(hvessel, flightmodel);

}

/////////////Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    
	if(vessel) delete(UCFO*)vessel;
	
}