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
#define ORBITER_MODULE
#include "main.h"

//Constructor
UCFO::UCFO(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){

    vhUCFO = NULL;

    mhUCFO = NULL;

    mhcockpitUCFO = NULL;

    dmhUCFO = NULL;

    right_rear_wheel_rotation = 0.0;

    right_front_wheel_rotation = 0.0;

    left_rear_wheel_rotation = 0.0;

    left_front_wheel_rotation = 0.0;

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

    uimesh_UCFO = 0;

    MeshName[0] = '\0';

    size = 0.0;

    empty_mass = 0.0;

    front_left_wheel_id = 0;

    front_right_wheel_id = 0;

    rear_left_wheel_id = 0;

    rear_right_wheel_id = 0;

    steering_wheel_id = 0;

    lvlwheeltrails = 0.0;

    anim_steering_wheel = 0;

    anim_custom_1 = 0;

    anim_custom_2 = 0;

    anim_custom_3 = 0;

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

    th_dummy = nullptr;

    thg_dummy = nullptr;

    left_headlight_beacon_spec = {0};

    right_headlight_beacon_spec = {0};

    right_tail_light_spec = {0};

    left_tail_light_spec = {0};

    left_turning_wheel = {0};

    right_turning_wheel = {0};

    beacon = {0};

    stop_light = {0};

    left_backup_light_spec = {0};

    right_backup_light_spec = {0};

    skin[0] = nullptr;

    skinpath[0] = '\0';

    left_headlight = nullptr;

    right_headlight = nullptr;

    left_tail_light_point = nullptr;

    right_tail_light_point = nullptr;

    sa = 0;

    R = 0.0;

    wheel_base = 0.0;

    angle_right = 0.0;

    angle_left = 0.0;

    wheel_track = 0.0;

    turn_radius = 0.0;

    max_weight = 0.0;

    main_fuel_tank_max = 0.0;

    pitch = 0.0;

    yaw = 0.0;

    bank = 0.0;

    x = 0.0;

    z = 0.0;

    mass = 0.0;

    Fuel = nullptr;

    max_weight_rear = 0.0;

    max_weight_front = 0.0;

    front_stiffness = 0.0;

    rear_stiffness = 0.0;

    body_stiffness = 0.0;

    front_damping = 0.0;

    rear_damping = 0.0;
    
    body_damping = 0.0;

    front_left_tilt_angle = 0.0;

    rear_right_tilt_angle = 0.0;

    rear_left_tilt_angle = 0.0;

    rear_left_tilt_angle = 0.0;

    rear_left_tilt_angle = 0.0;

    rear_left_tilt_angle = 0.0;

    rear_left_displacement = 0.0;

    front_right_wheel_force = 0.0;

    front_left_wheel_force = 0.0;

    rear_right_wheel_force = 0.0;

    rear_left_wheel_force = 0.0;

    FR_status = '\0';

    FL_status = '\0';

    RR_status = '\0';

    RL_status = '\0';

    headlight_status = '\0';

    front_right_displacement = 0.0;

    front_left_displacement = 0.0;

    rear_right_displacement = 0.0;

    for(int i = 0; i < ntdvtx_td_points; i++){

        td_points[i] = {0};

    }

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

    //Get vessel parameters from configuration file,
    //and set the physical vessel parameters.

    char chmesh[5] = "Mesh";

    if(!oapiReadItem_string(cfg, chmesh, MeshName)){
        TerminateAtError("%s: Mesh %s", GetName(), "car");
    }
    oapiWriteLogV("%s: Mesh: %s", GetName(), MeshName);

    mhUCFO = oapiLoadMeshGlobal(MeshName);
    uimesh_UCFO = AddMesh(mhUCFO);
    VECTOR3 mesh_origin = _V(0, 0, 0);
    SetMeshVisibilityMode(uimesh_UCFO, MESHVIS_ALWAYS);

    char chsize[5] = "Size";
   
    if(!oapiReadItem_float(cfg, chsize, size)){

        TerminateAtError("Size", GetName(), "car");

    }
    oapiWriteLogV("%s: Size: %f", GetName(), size);

    SetSize(size);


    char chmass[5] = "Mass";

    if(!oapiReadItem_float(cfg, chmass, empty_mass)){

        TerminateAtError("Mass", GetName(), "car");

    }

    oapiWriteLogV("%s: Mass: %f", GetName(), empty_mass);
    
    SetEmptyMass(empty_mass);

    
    char chFrontLeftWheelID[17] = "FrontLeftWheelID";

    if(!oapiReadItem_int(cfg, chFrontLeftWheelID, front_left_wheel_id)){

        TerminateAtError("FrontLeftWheelID", GetName(), "car");

    }

    oapiWriteLogV("%s: FrontLeftWheelID: %d", GetName(), front_left_wheel_id);
    

    char chFrontRightWheelID[18] = "FrontRightWheelID";
    if(!oapiReadItem_int(cfg, chFrontRightWheelID, front_right_wheel_id)){

        TerminateAtError("FrontRightWheelID", GetName(), "car");

    }
    oapiWriteLogV("%s: FrontRightWheelID: %d", GetName(), front_right_wheel_id);


    char chRearRightWheelID[17] = "RearRightWheelID";
    if(!oapiReadItem_int(cfg, chRearRightWheelID, rear_right_wheel_id)){

        TerminateAtError("RearRightWheelID", GetName(), "car");

    }
    oapiWriteLogV("%s: RearRightWheelID: %d", GetName(), rear_right_wheel_id);


    char chRearLeftWheelID[17] = "RearLeftWheelID";
    if(!oapiReadItem_int(cfg, chRearLeftWheelID, rear_left_wheel_id)){

        TerminateAtError("RearLeftWheelID", GetName(), "car");

    }
    oapiWriteLogV("%s: RearLeftWheelID: %d", GetName(), rear_left_wheel_id);


    char chFrontLeftWheelPosition[23] = "FrontLeftWheelPosition";
    if(!oapiReadItem_vec(cfg, chFrontLeftWheelPosition, front_left_wheel_pos)){

        TerminateAtError("FrontLeftWheelPosition", GetName(), "car");

    }
    oapiWriteLogV("FrontLeftWheelPosition %lf, %lf, %lf",
        front_left_wheel_pos.x,
        front_left_wheel_pos.y,
        front_left_wheel_pos.z);
    
    
    char chFrontRightWheelPosition[24] = "FrontRightWheelPosition";
    if(!oapiReadItem_vec(cfg, chFrontRightWheelPosition, front_right_wheel_pos)){

        TerminateAtError("FrontRightWheelPosition", GetName(), "car");

    }
    oapiWriteLogV("FrontRightWheelPosition %lf, %lf, %lf",
        front_right_wheel_pos.x,
        front_right_wheel_pos.y,
        front_right_wheel_pos.z);

    char chRearLeftWheelPosition[22] = "RearLeftWheelPosition";
    if(!oapiReadItem_vec(cfg, chRearLeftWheelPosition, rear_left_wheel_pos)){
     
        TerminateAtError("RearLeftWheelPosition", GetName(), "car");

    }
    oapiWriteLogV("RearLeftWheelPosition %lf, %lf, %lf",
        rear_left_wheel_pos.x,
        rear_left_wheel_pos.y,
        rear_left_wheel_pos.z);
    
    
    char chRearRightWheelPosition[23] = "RearRightWheelPosition";
    if(!oapiReadItem_vec(cfg, chRearRightWheelPosition, rear_right_wheel_pos)){

        TerminateAtError("RearRightWheelPosition", GetName(), "car");

    }
    oapiWriteLogV("RearRightWheelPosition %lf, %lf, %lf",
        rear_right_wheel_pos.x,
        rear_right_wheel_pos.y,
        rear_right_wheel_pos.z);


    char chLeftHeadlightPosition[22] = "LeftHeadlightPosition";
    if(!oapiReadItem_vec(cfg, chLeftHeadlightPosition, left_headlight_pos)){

        TerminateAtError("LeftHeadlightPosition", GetName(), "car");

    }
    oapiWriteLogV("LeftHeadlightPosition %lf, %lf, %lf",
        left_headlight_pos.x,
        left_headlight_pos.y,
        left_headlight_pos.z);

    
    char chRightHeadlightPosition[23] = "RightHeadlightPosition";
    if(!oapiReadItem_vec(cfg, chRightHeadlightPosition, right_headlight_pos)){

        TerminateAtError("RightHeadlightPosition", GetName(), "car");

    }
    oapiWriteLogV("RightHeadlightPosition %lf, %lf, %lf",
        right_headlight_pos.x,
        right_headlight_pos.y,
        right_headlight_pos.z);

    
    char chLeftTailLightPosition[22] = "LeftTailLightPosition";
    if(!oapiReadItem_vec(cfg, chLeftTailLightPosition, left_tail_light_pos)){

        TerminateAtError("LeftTailLightPosition", GetName(), "car");

    }
    oapiWriteLogV("LeftTailLightPosition %lf, %lf, %lf",
        left_tail_light_pos.x,
        left_tail_light_pos.y,
        left_tail_light_pos.z);
    

    char chRightTailLightPosition[23] = "RightTailLightPosition";
    if(!oapiReadItem_vec(cfg, chRightTailLightPosition, right_tail_light_pos)){

        TerminateAtError("RightTailLightPosition", GetName(), "car");

    }
    oapiWriteLogV("RightTailLightPosition %lf, %lf, %lf",
        right_tail_light_pos.x,
        right_tail_light_pos.y,
        right_tail_light_pos.z);

    
    char chLeftBackupLightPosition[24] = "LeftBackupLightPosition";
    if(!oapiReadItem_vec(cfg, chLeftBackupLightPosition, left_backup_pos)){

        TerminateAtError("LeftBackupLightPosition", GetName(), "car");

    }
    oapiWriteLogV("LeftBackupLightPosition %lf, %lf, %lf",
        left_backup_pos.x,
        left_backup_pos.y,
        left_backup_pos.z);

    
    char chRightBackupLightPosition[25] = "RightBackupLightPosition";
    if(!oapiReadItem_vec(cfg, chRightBackupLightPosition, right_backup_pos)){

        TerminateAtError("RightBackupLightPosition", GetName(), "car");

    }
    oapiWriteLogV("RightBackupLightPosition %lf, %lf, %lf",
        right_backup_pos.x,
        right_backup_pos.y,
        right_backup_pos.z);

    
    char chCameraPosition[15] = "CameraPosition";
    if(!oapiReadItem_vec(cfg, chCameraPosition, camera_pos)){
    
        TerminateAtError("CameraPosition", GetName(), "car");

    }
    oapiWriteLogV("CameraPosition %lf, %lf, %lf",
        camera_pos.x,
        camera_pos.y,
        camera_pos.z);

    SetCameraOffset(camera_pos);


    char chFrontRightWheelAxle[20] = "FrontRightWheelAxle";
    if(!oapiReadItem_vec(cfg, chFrontRightWheelAxle, front_right_wheel_axle)){

        TerminateAtError("FrontRightWheelAxle", GetName(), "car");

    }
    oapiWriteLogV("FrontRightWheelAxle %lf, %lf, %lf",
        front_right_wheel_axle.x,
        front_right_wheel_axle.y,
        front_right_wheel_axle.z);


    char chFrontLeftWheelAxle[19] = "FrontLeftWheelAxle";
    if(!oapiReadItem_vec(cfg, chFrontLeftWheelAxle, front_left_wheel_axle)){
     
        TerminateAtError("FrontLeftWheelAxle", GetName(), "car");

    }
    oapiWriteLogV("FrontLeftWheelAxle %lf, %lf, %lf",
        front_left_wheel_axle.x,
        front_left_wheel_axle.y,
        front_left_wheel_axle.z);


    char chRearRightWheelAxle[19] = "RearRightWheelAxle";
    if(!oapiReadItem_vec(cfg, chRearRightWheelAxle, rear_right_wheel_axle)){

        TerminateAtError("RearRightWheelAxle", GetName(), "car");

    }
    oapiWriteLogV("RearRightWheelAxle %lf, %lf, %lf",
        rear_right_wheel_axle.x,
        rear_right_wheel_axle.y,
        rear_right_wheel_axle.z);


    char chRearLeftWheelAxle[18] = "RearLeftWheelAxle";
    if(!oapiReadItem_vec(cfg, chRearLeftWheelAxle, rear_left_wheel_axle)){
     
        TerminateAtError("RearLeftWheelAxle", GetName(), "car");

    }
    oapiWriteLogV("RearLeftWheelAxle %lf, %lf, %lf",
        rear_left_wheel_axle.x,
        rear_left_wheel_axle.y,
        rear_left_wheel_axle.z);

    
    char chFrontRightWheelContact[23] = "FrontRightWheelContact";
    if(!oapiReadItem_vec(cfg, chFrontRightWheelContact, front_right_wheel_contact)){

        TerminateAtError("FrontRightWheelContact", GetName(), "car");

    }
    oapiWriteLogV("FrontRightWheelContact %lf, %lf, %lf",
        front_right_wheel_contact.x,
        front_right_wheel_contact.y,
        front_right_wheel_contact.z);


    char chFrontLeftWheelContact[22] = "FrontLeftWheelContact";
    if(!oapiReadItem_vec(cfg, chFrontLeftWheelContact, front_left_wheel_contact)){
    
        TerminateAtError("FrontLeftWheelContact", GetName(), "car");

    }
    oapiWriteLogV("FrontLeftWheelContact %lf, %lf, %lf",
        front_left_wheel_contact.x,
        front_left_wheel_contact.y,
        front_left_wheel_contact.z);


    char chRearRightWheelContact[22] = "RearRightWheelContact";
    if(!oapiReadItem_vec(cfg, chRearRightWheelContact, rear_right_wheel_contact)){
        
        TerminateAtError("RearRightWheelContact", GetName(), "car");

    }
    oapiWriteLogV("RearRightWheelContact %lf, %lf, %lf",
        rear_right_wheel_contact.x,
        rear_right_wheel_contact.y,
        rear_right_wheel_contact.z);


    char chRearLeftWheelContact[21] = "RearLeftWheelContact";
    if(!oapiReadItem_vec(cfg, chRearLeftWheelContact, rear_left_wheel_contact)){
     
        TerminateAtError("RearLeftWheelContact", GetName(), "car");

    }
    oapiWriteLogV("RearLeftWheelContact %lf, %lf, %lf",
        rear_left_wheel_contact.x,
        rear_left_wheel_contact.y,
        rear_left_wheel_contact.z);



    char chTouchdownPoint1[16] = "TouchdownPoint1";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint1, TDP1)){
     
        TerminateAtError("TouchdownPoint1", GetName(), "car");

    }
    oapiWriteLogV("TouchdownPoint1 %lf, %lf, %lf",
        TDP1.x,
        TDP1.y,
        TDP1.z);


    char chTouchdownPoint2[16] = "TouchdownPoint2";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint2, TDP2)){
     
        TerminateAtError("TouchdownPoint2", GetName(), "car");
    
    }
    oapiWriteLogV("TouchdownPoint2 %lf, %lf, %lf",
        TDP2.x,
        TDP2.y,
        TDP2.z);


    char chTouchdownPoint3[16] = "TouchdownPoint3";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint3, TDP3)){
     
        TerminateAtError("TouchdownPoint3", GetName(), "car");
        
    }
    oapiWriteLogV("TouchdownPoint3 %lf, %lf, %lf",
        TDP3.x,
        TDP3.y,
        TDP3.z);    

    
    char chTouchdownPoint4[16] = "TouchdownPoint4";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint4, TDP4)){
     
        TerminateAtError("TouchdownPoint4", GetName(), "car");
    
    }
    oapiWriteLogV("TouchdownPoint4 %lf, %lf, %lf",
        TDP4.x,
        TDP4.y,
        TDP4.z);
    
    
    char chTouchdownPoint5[16] = "TouchdownPoint5";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint5, TDP5)){
         
        TerminateAtError("TouchdownPoint5", GetName(), "car");
        
    }
    oapiWriteLogV("TouchdownPoint5 %lf, %lf, %lf",
        TDP5.x,
        TDP5.y,
        TDP5.z);
    
    
    char chTouchdownPoint6[16] = "TouchdownPoint6";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint6, TDP6)){
         
        TerminateAtError("TouchdownPoint6", GetName(), "car");
            
    }
    oapiWriteLogV("TouchdownPoint6 %lf, %lf, %lf",
        TDP6.x,
        TDP6.y,
        TDP6.z);

    
    char chTouchdownPoint7[16] = "TouchdownPoint7";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint7, TDP7)){
         
        TerminateAtError("TouchdownPoint7", GetName(), "car");
                
    }
    oapiWriteLogV("TouchdownPoint7 %lf, %lf, %lf",
        TDP7.x,
        TDP7.y,
        TDP7.z);


    char chTouchdownPoint8[16] = "TouchdownPoint8";
    if(!oapiReadItem_vec(cfg, chTouchdownPoint8, TDP8)){

        TerminateAtError("TouchdownPoint8", GetName(), "car");

    }
    oapiWriteLogV("TouchdownPoint8 %lf, %lf, %lf",
        TDP8.x,
        TDP8.y,
        TDP8.z);

    Fuel = CreatePropellantResource(UCFO_FUEL_MASS);

    //Dummy thruster to provide throttle input to engine model. 1 N thrust is to allow thruster status
    //to indicate throttle position

    th_dummy = CreateThruster(ENGINE_LOCATION, FORWARD_DIRECTION, 1, Fuel, INFINITY);

    thg_dummy = CreateThrusterGroup(&th_dummy, 1, THGROUP_MAIN);

    //SetCW(0.5, 0.5, 0.3, 0.3);

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

    //Light locations and specifications

    MakeLightsHeadlights();
    MakeLightsTaillights();
    MakeLightsBackuplights();

    MakeAnim_RightFrontWheel();
    MakeAnim_LeftFrontWheel();
    MakeAnim_RightRearWheel();
    MakeAnim_LeftRearWheel();

    //Screen message formatting
    MakeAnnotation_Format();
    
}

double UCFO::UpdateLvlWheelsTrails(){

    double speed = GetGroundspeed();


    if((GroundContact()) && (speed > 10.0)){

        return 1.0;

    } else {

        return 0.0;

    }

}

void UCFO::clbkPostCreation(){

    GetWeightVector(max_weight_vector);
    max_weight = length(max_weight_vector);

    wheel_base = front_right_wheel_contact.z - rear_right_wheel_contact.x;

    wheel_track = front_right_wheel_contact.x - front_left_wheel_contact.x;

    max_weight_front = max_weight * (-rear_right_wheel_contact.z / wheel_base); //Weight supported by two front wheells.
    max_weight_rear = max_weight - max_weight_front; //Weight supported by two rear wheels.

    SetContact_TouchdownPoints();

}

void UCFO::clbkPreStep(double simt, double simdt, double mjd){

    SetFeature_Caster();
    SetFeature_Ackermann();
    SetFeature_Brakes();
    SetEngine_Power();

    //Determine strut travel for animations and to calculate normal contact forces on all wheels
    pitch = GetPitch();
    yaw = 0; //does not affect strut travel
    bank = GetBank();
    GetGroundspeedVector(FRAME_LOCAL, vel);
    GetAngularVel(angvel);
    omega = {x = 0, angvel.y, z = 0};
    mass = empty_mass + GetPropellantMass(Fuel);

    //Get normal force on wheels for friction calculations

    GetHelp_NormalForce();

    //Get direction of wheel axes for steering force directions

    GetHelp_WheelAxis();

    //Get wheel velocity relative to ground for friction vector

    GetHelp_WheelVelocity(vel, omega);

    GetHelp_DynamicFriction();
    GetHelp_StaticFriction();

    //Compare and apply lesser of dynamic and static friction forces on wheels

    GetHelp_StickOrSkid();

    //Update animations and lights

    SetAnim_LeftFrontWheel();
    SetAnim_RightFrontWheel();
    SetAnim_LeftRearWheel();
    SetAnim_RightRearWheel();

    SetLightHeadlights();
    SetLightBrakelights();
    SetLightBackuplights();

    SetAnnotation_Messages();

}

void UCFO::clbkPostStep(double simt, double simdt, double mjd){

    lvlwheeltrails = UpdateLvlWheelsTrails();

    
}

void UCFO::clbkLoadStateEx(FILEHANDLE scn, void *vs){

    char *line;

    while(oapiReadScenario_nextline(scn, line)){
        ParseScenarioLineEx(line, vs);
    }

}

void UCFO::clbkSaveState(FILEHANDLE scn){

    char cbuf[256];

    SaveDefaultState(scn);
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

        left_backup_light_spec.active = true;
        right_backup_light_spec.active = true;

    } else {

        left_backup_light_spec.active = false;
        right_backup_light_spec.active = false;

    }

}

void UCFO::TerminateAtError(const char *error, const char * className, const char *type){

    oapiWriteLogV("ERROR: UCFO cannot continue: The %s of %s %s is not specified", error, className, type);

    std::terminate();
}

void UCFO::NotifyInLog(const char *error, const char *className, const char *type){

    oapiWriteLogV("Warning: UCFO can continue, but the %s of %s %s is not specified", error, className, type);

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