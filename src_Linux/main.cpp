#include "main.hpp"
#include <cmath>

//Constructor
UCFO::UCFO(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){

    vhUCFO = nullptr;

    mhUCFO = nullptr;

    mhcockpitUCFO = nullptr;

    dmhUCFO = nullptr;

    uimesh_UCFO = 0;

    uimesh_Cockpit = 0;

    th_dummy = nullptr;

    thg_dummy = nullptr;

    main_fuel_tank = nullptr;

    for(int i = 0; i < 12; i++){

        td_points[i] = {0};

    }

    mass = 0.0;

    brake_status = '\0';

    max_weight = 0.0;

    max_weight_front = 0.0;

    max_weight_rear = 0.0;

    angle_right = 0.0;

    angle_left = 0.0;

    turn_radius = 0.0;

    force = 0.0;

    pitch = 0.0;

    yaw = 0.0;

    bank = 0.0;

    front_stiffness = 0.0;

    rear_stiffness = 0.0;

    front_right_wheel_force = 0.0;

    front_left_wheel_force = 0.0;

    rear_right_wheel_force = 0.0;

    rear_left_wheel_force = 0.0;

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

    steering_angle = 0.0;

    drive_status = 'F';

    TDP1 = _V(0, 0, 0);
    TDP2 = _V(0, 0, 0);
    TDP3 = _V(0, 0, 0);
    TDP4 = _V(0, 0, 0);
    TDP5 = _V(0, 0, 0);
    TDP6 = _V(0, 0, 0);
    TDP7 = _V(0, 0, 0);
    TDP8 = _V(0, 0, 0);
    
    wv = _V(0, 0, 0);

    wheel_base = 0.0;

    wheel_track = 0.0;

    displacement = 0.0;;
    n_rev = 0.0;
    r = 0.0; //Compression ratio
    idle_rpm = 0.0;
    max_rpm = 0.0;

    k = 0.0; //polytropic specific heat ratio
    cp_air = 0.0; //specific heat of air J/kg K

    HV = 0.0; //lower heating value of gasoline J/kg
    AF = 0.0; //stochiometric fuel air ratio for gasoline

    air_density = 0.0;

    air_temp = 0.0;

    gsv = _V(0, 0, 0);

    dt = 0.0;

    front_right_wheel_contact = _V(0, 0, 0);
    front_left_wheel_contact = _V(0, 0, 0);
    rear_right_wheel_contact = _V(0, 0, 0);
    rear_left_wheel_contact = _V(0, 0, 0);

    vel = _V(0, 0, 0);

    omega = _V(0, 0, 0);

    front_right_axle_axis = _V(0, 0, 0);
    front_left_axle_axis = _V(0, 0, 0);
    rear_right_axle_axis = _V(0, 0, 0);
    rear_left_axle_axis = _V(0, 0, 0);

    velFR = _V(0, 0, 0);
    velFL = _V(0, 0, 0);
    velRR = _V(0, 0, 0);
    velRL = _V(0, 0, 0);

    front_right_skid_force = _V(0, 0, 0);
    front_left_skid_force = _V(0, 0, 0);
    rear_right_skid_force = _V(0, 0, 0);
    rear_left_skid_force = _V(0, 0, 0);

    front_right_skid_force_axis = _V(0, 0, 0);
    front_left_skid_force_axis = _V(0, 0, 0);
    rear_right_skid_force_axis = _V(0, 0, 0);
    rear_left_skid_force_axis = _V(0, 0, 0);

    front_right_turning_force = _V(0, 0, 0);
    front_left_turning_force = _V(0, 0, 0);
    rear_right_turning_force = _V(0, 0, 0);
    rear_left_turning_force = _V(0, 0, 0);

    front_right_turning_force_axis = _V(0, 0, 0);
    front_left_turning_force_axis = _V(0, 0, 0);
    rear_right_turning_force_axis = _V(0, 0, 0);
    rear_left_turning_force_axis = _V(0, 0, 0);

    front_right_wheel_contact_local = _V(0, 0, 0);
    front_left_wheel_contact_local = _V(0, 0, 0);
    rear_right_wheel_contact_local = _V(0, 0, 0);
    rear_left_wheel_contact_local = _V(0, 0, 0);

    front_right_tilt_angle = 0.0;
    front_left_tilt_angle = 0.0;
    rear_right_tilt_angle = 0.0;
    rear_left_tilt_angle = 0.0;

    //Determine strut displacement from tilt angle using right triangle

    front_right_displacement = 0.0;
    front_left_displacement = 0.0;
    rear_right_displacement = 0.0;
    rear_left_displacement = 0.0;

    msg1_annotation = nullptr;
    msg2_annotation = nullptr;
    msg3_annotation = nullptr;
    msg4_annotation = nullptr;
    msg5_annotation = nullptr;
    msg6_annotation = nullptr;
    msg7_annotation = nullptr;
    msg8_annotation = nullptr;
    msg9_annotation = nullptr;
    msg10_annotation = nullptr;
    msg11_annotation = nullptr;

}

//Destructor
UCFO::~UCFO(){

}


void UCFO::clbkSetClassCaps(FILEHANDLE cfg){

    SetEmptyMass(empty_mass);
    main_fuel_tank = CreatePropellantResource(main_fuel_tank_max);

    //Dummy thruster to provide throttle input to engine model. 1 N thrust is to allow thruster status
    //to indicate throttle position

    th_dummy = CreateThruster(_V(0, 0, 0), _V(0, 1, 0), 1, main_fuel_tank, INFINITY);
    thg_dummy = CreateThrusterGroup(&th_dummy, 1, THGROUP_MAIN);


    //Engine initialization
    //VW Thing engine specs
    displacement = 1.584e-3; //engine displacement per cycle in cubic meters
    n_rev = 2; //shaft revolutions per cycle (1 for 2-stroke, 2 for 4-stroke)
    r = 7.5; //Compression ratio
    idle_rpm = 700;
    max_rpm = 5000;

    //Fuel properties
    HV = 45e+6; //lower heating value of gasoline J/kg
    AF = 14.7; //stochiometric fuel air ratio for gasoline

    mhUCFO = oapiLoadMeshGlobal("VWThing/VWThing");
    uimesh_UCFO = AddMesh(mhUCFO);
    SetMeshVisibilityMode(uimesh_UCFO, MESHVIS_ALWAYS);

    SetCameraOffset(_V(-0.25, 1.0, 0.0));

    MakeContact_TouchdownPoints();
    oapiWriteLogV("front_right_wheel_contact: %f, %f, %f",
        front_right_wheel_contact.x,
        front_right_wheel_contact.y,
        front_right_wheel_contact.z);

    //Screen message formatting
    MakeAnnotation_Format();

}

void UCFO::clbkPostCreation(){

    //Following is needed to allow the suspension to be set and tuned to the correct height independent of the planetary body it is on.
    wheel_base = front_right_wheel_contact.z - rear_right_wheel_contact.z;

    wheel_track = front_right_wheel_contact.x - front_left_wheel_contact.x;

    GetWeightVector(wv);

    max_weight = length(wv);

    max_weight_front = max_weight * (-rear_right_wheel_contact.z / wheel_base); // weight supported by two front wheels.

    max_weight_rear = max_weight - max_weight_front; //weight supported by two rear wheels.

    SetContact_TouchdownPoints();

    
}

void UCFO::clbkPreStep(double simt, double simdt, double mjd){

    SetFeature_Caster();
    SetFeature_Ackermann();
    SetFeature_Brakes();

        

    SetEngine_Power();
    

    pitch = GetPitch();
    yaw = 0.0;
    bank = GetBank();
    GetGroundspeedVector(FRAME_LOCAL, vel);
    VECTOR3 avel = _V(0, 0, 0);
    GetAngularVel(avel);
    omega = {0, avel.y, 0};
    mass = empty_mass + GetPropellantMass(main_fuel_tank);

    //get normal force on wheels for friction calculations

    GetHelp_NormalForce();

    //get direction of wheel axes for steering force directions

    GetHelp_WheelAxis();

    //get wheel velocity relative to ground for friction vector

    GetHelp_WheelVelocity(vel, omega);

    //Calculate dynamic and static friction forces on wheels

    GetHelp_DynamicFriction();
    GetHelp_StaticFriction();

    //Compare and apply lesser of dynamic and static friction forces on wheels

    GetHelp_StickOrSkid();

    SetAnnotation_Messages();


}

int UCFO::clbkConsumeDirectKey(char *kstate){

    if(KEYDOWN(kstate, OAPI_KEY_SPACE)){
        
        brake_status = 'N';

    } else {
        brake_status = 'F';
    }

    if(KEYDOWN(kstate, OAPI_KEY_NUMPAD1)){

        steering_angle = std::max(steering_angle - 5 * RAD, -1.0);

    }

    if(KEYDOWN(kstate, OAPI_KEY_NUMPAD3)){
        
        steering_angle = std::min(steering_angle + 5 * RAD, 1.0);
        
    }

    return 0;
}

int UCFO::clbkConsumeBufferedKey(int key, bool down, char *kstate){

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

    return 0;
}

DLLCLBK void InitModule(MODULEHANDLE hModule){

}

DLLCLBK void ExitModule(MODULEHANDLE *hModule){

}

///////////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    
	return new UCFO(hvessel, flightmodel);

}

/////////////Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    
	if(vessel) delete(UCFO*)vessel;
	
}