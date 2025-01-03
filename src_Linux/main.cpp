/////////////////////////////////////////////////////
//Copyright (c)2024 Matías Saibene                 //
//ORBITER MODULE: UNIVERSAL CARS FOR ORBITER (UCFO)//
//      Licenced under the MIT Licence             //
// main.cpp  Main implementation file              //
/////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <string>
#define ORBITER_MODULE
#include "main.h"

//Constructor
UCFO::UCFO(OBJHANDLE hVessel, int flightmodel) : 
VESSEL4(hVessel, flightmodel){

	mhUCFO = NULL;

	wheel_base = 0.0;

	wheel_track = 0.0;

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
	

}

//Destructor
UCFO::~UCFO(){

}

void UCFO::clbkSetClassCaps(FILEHANDLE cfg){


	//Get vessel parameters from configuration file,
    //and set the physical vessel parameters.

	if(!oapiReadItem_string(cfg, "Mesh", MeshName)){
		TerminateAtError("Mesh", GetName(), "car");
	}

	SetMeshVisibilityMode (AddMesh (mhUCFO = oapiLoadMeshGlobal (MeshName)), MESHVIS_EXTERNAL);


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



}

void UCFO::clbkPostCreation(){

	//Following is needed to allow the suspension to be set and tuned to the correct height independent of the planetary body it is on.

	GetWeightVector(max_weight_vector);
    max_weight = length(max_weight_vector);

    max_weight_front = max_weight * (-rear_right_wheel_contact.z / wheel_base); //Weight supported by two front wheells.
    max_weight_rear = (max_weight - max_weight_front); //Weight supported by two rear wheels.

    SetContactTouchdownPoints();

}

void UCFO::clbkPreStep(double simt, double simdt, double mjd){

	//Determine strut travel for animations and to calculate normal contact forces on all wheels
	pitch = GetPitch();
	yaw = GetYaw();
	bank = GetBank();
	GetGroundspeedVector(FRAME_LOCAL, vel);
	GetAngularVel(angvel);
	omega = {x = 0, angvel.y, z = 0};
	mass = empty_mass + GetPropellantMass(Fuel);

	EnginePower();
	Brakes();

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

    double l_throttle = GetThrusterLevel(th_dummy);

    double l_rpm = idle_rpm + l_throttle * (max_rpm - idle_rpm);

    double l_omega = l_rpm * (2 * PI) / 60;

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

    double l_omega_wheel = omega_wheel_aux.z / wheel_radius;

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

    double speed = GetGroundspeed();


    if((speed > 10.0) && (GroundContact())){

        return 1.0;

    } else {

        return 0.0;

    }

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

void UCFO::clbkPostStep(double simt, double simdt, double mjd){

	lvlwheeltrails = UpdateLvlWheelsTrails();

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