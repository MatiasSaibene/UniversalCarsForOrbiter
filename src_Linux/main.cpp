/////////////////////////////////////////////////////
//Copyright (c)2025 Thunder Chicken & Matías Saibene//
//ORBITER MODULE: UNIVERSAL CARS FOR ORBITER (UCFO)//
//      Licenced under the MIT Licence             //
//        main.cpp  Main implementation file       //
/////////////////////////////////////////////////////
#include "main.hpp"
#include <cctype>
#include <cmath>
#include <cstddef>
#include <optional>
#include "format"

//Constructor
UCFO::UCFO(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel), uacs(this, &vslAstrInfo, nullptr){

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

    anim_right_front_wheel_rotation = 0;

    anim_left_front_wheel_rotation = 0;

    anim_right_rear_wheel_rotation = 0;

    anim_left_rear_wheel_rotation = 0;

    anim_right_front_wheel_steer = 0;

    anim_left_front_wheel_steer = 0;

    anim_right_front_wheel_travel = 0;

    anim_left_front_wheel_travel = 0;

    anim_right_rear_wheel_travel = 0;

    anim_left_rear_wheel_travel = 0;

    right_rear_wheel_rotation = 0.0;

    right_front_wheel_rotation = 0.0;

    left_rear_wheel_rotation = 0.0;

    left_front_wheel_rotation = 0.0;

    right_front_wheel_travel = nullptr;

    right_front_wheel_steer = nullptr;

    right_front_wheel_rotate = nullptr;

    MeshName[0] = '\0';

    size = 0.0;

    empty_mass = 0.0;

    main_fuel_tank_max = 0.0;

    front_left_wheel_id = nullptr;

    front_right_wheel_id = nullptr;

    rear_right_wheel_id = nullptr;

    rear_left_wheel_id = nullptr;

    travel = 0.0;

    wheel_radius = 0.0;

    left_front_wheel_rotate = nullptr;

    left_front_wheel_travel = nullptr;

    left_front_wheel_steer = nullptr;

    left_rear_wheel_travel = nullptr;

    left_rear_wheel_rotate = nullptr;

    right_rear_wheel_travel = nullptr;

    right_rear_wheel_rotate = nullptr;

    headlight_status = '\0';

    lvlwheeltrails = 0.0;

    left_headlight_beacon_spec = {0};

    right_headlight_beacon_spec = {0};

    right_tail_light_spec = {0};

    left_tail_light_spec = {0};

    left_backup_light_spec = {0};

    right_backup_light_spec = {0};

    left_headlight = nullptr;

    right_headlight = nullptr;

    left_tail_light_point = nullptr;

    right_tail_light_point = nullptr;

    left_backup_light_point = nullptr;

    right_backup_light_point = nullptr;

    pBrake = false;

    ExitPosition1 = _V(0, 0, 0);
}

//Destructor
UCFO::~UCFO(){

}


void UCFO::clbkSetClassCaps(FILEHANDLE cfg){

    //Get vessel parameters from configuration file,
    //and set the physical vessel parameters.

    if(!oapiReadItem_string(cfg, "Mesh", MeshName)){
        TerminateAtError("%s: Mesh: %s", GetName(), "car");
    }
    oapiWriteLogV("%s: Mesh: %s", GetName(), MeshName);

    mhUCFO = oapiLoadMeshGlobal(MeshName);
    uimesh_UCFO = AddMesh(mhUCFO);
    SetMeshVisibilityMode(uimesh_UCFO, MESHVIS_ALWAYS);

    if(!oapiReadItem_float(cfg, "Size", size)){
        TerminateAtError("Size", GetName(), "car");
    }
    SetSize(size);


    if(!oapiReadItem_float(cfg, "Mass", empty_mass)){
		TerminateAtError("Mass", GetName(), "car");
	}
    SetEmptyMass(empty_mass);


    if(!oapiReadItem_float(cfg, "Fuel", main_fuel_tank_max)){
        TerminateAtError("Fuel", GetName(), "car");
    }
    main_fuel_tank = CreatePropellantResource(main_fuel_tank_max);

    if(!oapiReadItem_float(cfg, "Displacement", displacement)){
        TerminateAtError("Displacement", GetName(), "car");
    }

    if(!oapiReadItem_float(cfg, "Compression", r)){
        TerminateAtError("Compression", GetName(), "car");
    }

    if(!oapiReadItem_float(cfg, "MaxRPM", max_rpm)){
        TerminateAtError("MaxRPM", GetName(), "car");
    }

    if(!oapiReadItem_float(cfg, "IdleRPM", idle_rpm)){
        TerminateAtError("IdleRPM", GetName(), "car");
    }

    if(!oapiReadItem_float(cfg, "WheelRadius", wheel_radius)){
        TerminateAtError("WheelRadius", GetName(), "car");
    }

    if(!oapiReadItem_float(cfg, "Travel", travel)){
        TerminateAtError("Travel", GetName(), "car");
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

    if (!front_left_wheel_id) { 
        front_left_wheel_id = new int; 
    }

    if (!front_right_wheel_id) { 
        front_right_wheel_id = new int; 
    }

    if (!rear_right_wheel_id) { 
        rear_right_wheel_id = new int;
    }

    if (!rear_left_wheel_id) { 
        rear_left_wheel_id = new int;
    }

    if(!oapiReadItem_int(cfg, "FrontLeftWheelID", *front_left_wheel_id)){
		TerminateAtError("FrontLeftWheelID", GetName(), "car");
	}
    oapiWriteLogV("%s: FrontLeftWheelID: %d", GetName(), &front_left_wheel_id);


    if(!oapiReadItem_int(cfg, "FrontRightWheelID", *front_right_wheel_id)){
		TerminateAtError("FrontRightWheelID", GetName(), "car");
	}
    oapiWriteLogV("%s: FrontRightWheelID: %d", GetName(), &front_right_wheel_id);

    if(!oapiReadItem_int(cfg, "RearLeftWheelID", *rear_left_wheel_id)){
		TerminateAtError("RearLeftWheelID", GetName(), "car");
	}
    oapiWriteLogV("%s: RearLeftWheelID: %d", GetName(), &rear_left_wheel_id);

    if(!oapiReadItem_int(cfg, "RearRightWheelID", *rear_right_wheel_id)){
		TerminateAtError("RearRightWheelID", GetName(), "car");
	}
    oapiWriteLogV("%s: RearRightWheelID: %d", GetName(), &rear_right_wheel_id);


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



    if(!oapiReadItem_vec(cfg, "CameraPosition", camera_pos)){
		TerminateAtError("CameraPosition", GetName(), "car");
	}

	SetCameraOffset(camera_pos);


    if(!oapiReadItem_vec(cfg, "ExitPosition1", ExitPosition1)){
        TerminateAtError("ExitPosition1", GetName(), "car");
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


    //Dummy thruster to provide throttle input to engine model. 1 N thrust is to allow thruster status
    //to indicate throttle position

    th_dummy = CreateThruster(_V(0, 0, 0), _V(0, 1, 0), 1, main_fuel_tank, INFINITY);
    thg_dummy = CreateThrusterGroup(&th_dummy, 1, THGROUP_MAIN);


    //Engine initialization
    //VW Thing engine specs
    //displacement = 1.584e-3; //engine displacement per cycle in cubic meters
    n_rev = 2; //shaft revolutions per cycle (1 for 2-stroke, 2 for 4-stroke)
    //r = 7.5; //Compression ratio
    //idle_rpm = 700;
    //max_rpm = 5000;

    //Fuel properties
    HV = 45e+6; //lower heating value of gasoline J/kg
    AF = 14.7; //stochiometric fuel air ratio for gasoline

    //SetCameraOffset(_V(-0.25, 1.0, 0.0));

    SetMaxWheelbrakeForce(50);

    MakeContact_TouchdownPoints();

    //Screen message formatting
    MakeAnnotation_Format();

    MakeAnim_RightFrontWheel();
    MakeAnim_LeftFrontWheel();
    MakeAnim_RightRearWheel();
    MakeAnim_LeftRearWheel();

    MakeLightHeadlights();
    MakeLightBackuplights();
    MakeLightTaillights();

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


    //UACS support... Yay!

    UACS::AirlockInfo airInfo;

    airInfo.name = "CarDoor";
    airInfo.pos = ExitPosition1;
    airInfo.dir = FORWARD_DIRECTION;
    airInfo.rot = _V(-1, 0, 0);
    airInfo.hDock = CreateDock(ExitPosition1, FORWARD_DIRECTION, _V(-1, 0, 0));
    airInfo.gndInfo.pos = std::nullopt;

    vslAstrInfo.airlocks.push_back(airInfo);
    vslAstrInfo.stations.emplace_back("Pilot");




}

void UCFO::clbkPostCreation(){

    uacs.clbkPostCreation();

    //Following is needed to allow the suspension to be set and tuned to the correct height independent of the planetary body it is on.
    wheel_base = front_right_wheel_contact.z - rear_right_wheel_contact.z;

    wheel_track = front_right_wheel_contact.x - front_left_wheel_contact.x;

    GetWeightVector(wv);

    max_weight = length(wv);

    max_weight_front = max_weight * (-rear_right_wheel_contact.z / wheel_base); // weight supported by two front wheels.

    max_weight_rear = max_weight - max_weight_front; //weight supported by two rear wheels.

    SetContact_TouchdownPoints(pBrake);

    SetEmptyMass(GetEmptyMass() + uacs.GetTotalAstrMass());
    
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

    SetAnim_RightFrontWheel();
    SetAnim_LeftFrontWheel();
    SetAnim_RightRearWheel();
    SetAnim_LeftRearWheel();

    lvlwheeltrails = UpdateLvlWheelsTrails();

    SetLightHeadlights();
    SetLightBrakelights();
    SetLightBackuplights();

    if(astrHUD.timer < 5) astrHUD.timer += simdt;

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

    if(key == OAPI_KEY_NUMPADENTER && down){

        if(pBrake == false){
            pBrake = true;
            SetThrusterGroupLevel(THGROUP_MAIN, 0);
            SetContact_TouchdownPoints(pBrake);
            SetWheelbrakeLevel(1, 0, true);
        } else {
            pBrake = false;
            SetContact_TouchdownPoints(pBrake);
            SetWheelbrakeLevel(0, 0, true);
        }

    }

    if(KEYMOD_ALT(kstate) && key == OAPI_KEY_M){
        hudMode < 2 ? hudMode++ : hudMode = 0;
        return 1;
    } else if(hudMode != HUD_OP) return 0;

    if(KEYMOD_ALT(kstate)){
        
        switch(key){

            case OAPI_KEY_NUMPAD8:
                astrHUD.idx + 1 < uacs.GetAvailAstrCount() ? ++astrHUD.idx : astrHUD.idx = 0;
                return 1;

            case OAPI_KEY_NUMPAD2:
                astrHUD.idx > 0 ? --astrHUD.idx : astrHUD.idx = uacs.GetAvailAstrCount() - 1;
                return 1;
            
        }

    }

    if(KEYMOD_ALT(kstate)){

        switch(key){

            case OAPI_KEY_A:
                
                switch(uacs.AddAstronaut(astrHUD.idx)){

                    case UACS::INGRS_SUCCED : 
                        astrHUD.msg = "Success: Selected astronaut added.";
                        break;
                    
                        case UACS::INGRS_STN_OCCP : 
                            astrHUD.msg = "Error: Station occupied.";
                            break;
                        
                        case UACS::INGRS_FAIL : 
                            astrHUD.msg = "Error: The addition failed.";
                            break;
                }

                astrHUD.timer = 0;
                return 1;

            case OAPI_KEY_E:{
                
                switch(uacs.EgressAstronaut()){

                    case UACS::EGRS_SUCCED:
                        astrHUD.msg = "Success: Astronaut egressed.";
                        break;
                    
                    case UACS::EGRS_STN_EMPTY:
					    astrHUD.msg = "Error: No astronaut onboard.";
						break;

                    case UACS::EGRS_ARLCK_DCKD:
						astrHUD.msg = "Error: Airlock blocked by a docked vessel.";
						break;

					case UACS::EGRS_NO_EMPTY_POS:
						astrHUD.msg = "Error: No empty position nearby.";
						break;

					case UACS::EGRS_INFO_NOT_SET:
						astrHUD.msg = "Error: Astronaut egressed but info not set.";
						break;

					case UACS::EGRS_FAIL:
						astrHUD.msg = "Error: The egress failed.";
						break;
                }

                astrHUD.timer = 0;
                return 1;

            }

        }
    }



    return 0;
}

void UCFO::clbkLoadStateEx(FILEHANDLE scn, void *status){

    char *line;

    while(oapiReadScenario_nextline(scn, line)){
        if(!uacs.ParseScenarioLine(line)){
            ParseScenarioLineEx(line, status);
        }
    }
}

void UCFO::clbkSaveState(FILEHANDLE scn){

    VESSEL4::clbkSaveState(scn);

    uacs.clbkSaveState(scn);

}

int UCFO::clbkGeneric(int msgid, int prm, void *context){

    if(msgid == UACS::MSG){
        
        switch(prm){

            case UACS::ASTR_INGRS:{

                auto astrIdx = *static_cast<size_t*>(context);
                auto &astrInfo = vslAstrInfo.stations.at(astrIdx).astrInfo;

                SetEmptyMass(GetEmptyMass() + astrInfo->mass);
                return 1;
            }

            case UACS::ASTR_EGRS:{

                auto astrIdx = *static_cast<size_t*>(context);
                auto &astrInfo = vslAstrInfo.stations.at(astrIdx).astrInfo;

                SetEmptyMass(GetEmptyMass() - astrInfo->mass);
                return 1;

            }

            default:
                return 0;
            
        }
    }

    return 0;
}

bool UCFO::clbkDrawHUD(int mode, const HUDPAINTSPEC *hps, oapi::Sketchpad *skp){

    VESSEL4::clbkDrawHUD(mode, hps, skp);

    int x = HIWORD(skp->GetCharSize());
	int rightX = hps->W - x;
	int startY = int(0.215 * hps->H);
	int y = startY;

    int space = LOWORD(skp->GetCharSize());
    int largeSpace = int(1.5 * space);

    if(hudMode == UCFO::HUD_OP){
        x = rightX;
        y = startY;
        skp->SetTextAlign(oapi::Sketchpad::RIGHT);

        buffer = std::format("Selected available astronaut: {}", uacs.GetAvailAstrName(astrHUD.idx));
        skp->Text(x, y, buffer.c_str(), buffer.size());

        if(astrHUD.timer < 5){
            y += largeSpace;
            skp->Text(x, y, astrHUD.msg.c_str(), astrHUD.msg.size());
        }

        if(const auto &info = vslAstrInfo.stations.front().astrInfo){
            const auto &astrInfo = *info;

            y += largeSpace;
            skp->Text(x, y, "Onboard astronaut information", 29);
            y += largeSpace;

            buffer = std::format("Name: {}", astrInfo.name);
            skp->Text(x, y, buffer.c_str(), buffer.size());
            y += space;

            buffer = astrInfo.role;
            buffer[0] = std::toupper(buffer[0]);

            buffer = std::format("Role: {}", buffer);
            skp->Text(x, y, buffer.c_str(), buffer.size());
            y += space;

            buffer = std::format("Mass: {:g}kg", astrInfo.mass);
            skp->Text(x, y, buffer.c_str(), buffer.size());
            y += largeSpace;

            buffer = std::format("Fuel: {:g}%", astrInfo.fuelLvl * 100);
            skp->Text(x, y, buffer.c_str(), buffer.size());
            y += space;

            buffer = std::format("Oxygen: {:g}%", astrInfo.oxyLvl * 100);
            skp->Text(x, y, buffer.c_str(), buffer.size());
            y += space;

            buffer = std::format("Alive: {}", astrInfo.alive ? "Yes" : "No");
            skp->Text(x, y, buffer.c_str(), buffer.size());
        }

    } else if(hudMode == HUD_SRT){

	    x = rightX;
		y = startY;
		skp->SetTextAlign(oapi::Sketchpad::RIGHT);

		skp->Text(x, y, "Alt + Numpad 8/2: Select next/previous available astronaut", 58);
		y += space;

		skp->Text(x, y, "Right Alt + A: Add selected astronaut", 37);
		y += space;

		skp->Text(x, y, "Right Alt + E: Egress onboard astronaut", 39);
		y += space;

		skp->Text(x, y, "Right Alt + D: Delete onboard astronaut", 39);
    }

    return true;

}

void UCFO::TerminateAtError(const char *error, const char * className, const char *type){

    oapiWriteLogV("ERROR: UCFO cannot continue: The %s of %s %s is not specified", error, className, type);

    std::terminate();
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