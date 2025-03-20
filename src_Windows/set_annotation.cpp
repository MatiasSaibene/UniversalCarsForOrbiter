#include "main.h"

#include <cstring>
#include <string.h>
#include <string>
#include <cstdio>

void UCFO::SetAnnotation_Messages(){

    double kph = 3.6 * vel.z;
    std::string message1 = std::to_string(kph) + " km/h";

    char buffer[50];
    sprintf(buffer, "%.1f deg turn", steering_angle * DEG);
    std::string message2 = buffer;

    std::string message3, message4, message5, message6;
    std::string message7 = "Steer with rudder controls";
    std::string message8 = "Brake with spacebar";
    std::string message9 = "Shift forward/reverse with comma (,) and period (.)";
    std::string message10 = "Switch between forward view (F) and tire view (V)";
    std::string message11 = "Toggle headlights with L";

    if (drive_status == 'F') {
        message3 = "D";
    } else if (drive_status == 'N') {
        message4 = "N";
    } else if (drive_status == 'R') {
        message5 = "R";
    }

    char *cmessage1 = nullptr;
    strcpy(cmessage1, message1.c_str());

    char *cmessage2 = nullptr;
    strcpy(cmessage2, message2.c_str());

    char *cmessage3 = nullptr;
    strcpy(cmessage3, message3.c_str());

    char *cmessage4 = nullptr;
    strcpy(cmessage4, message4.c_str());

    char *cmessage5 = nullptr;
    strcpy(cmessage5, message5.c_str());

    char *cmessage6 = nullptr;
    strcpy(cmessage6, message6.c_str());

    char *cmessage7 = nullptr;
    strcpy(cmessage7, message7.c_str());

    char *cmessage8 = nullptr;
    strcpy(cmessage8, message8.c_str());

    char *cmessage9 = nullptr;
    strcpy(cmessage9, message9.c_str());
    
    char *cmessage10 = nullptr;
    strcpy(cmessage10, message10.c_str());
    
    char *cmessage11 = nullptr;
    strcpy(cmessage11, message11.c_str());
    

    oapiAnnotationSetText(msg1_annotation, cmessage1);
    oapiAnnotationSetText(msg2_annotation, cmessage2);
    oapiAnnotationSetText(msg3_annotation, cmessage3);
    oapiAnnotationSetText(msg4_annotation, cmessage4);
    oapiAnnotationSetText(msg5_annotation, cmessage5);
    oapiAnnotationSetText(msg6_annotation, cmessage6);
    oapiAnnotationSetText(msg7_annotation, cmessage7);
    oapiAnnotationSetText(msg8_annotation, cmessage8);
    oapiAnnotationSetText(msg9_annotation, cmessage9);
    oapiAnnotationSetText(msg10_annotation, cmessage10);
    oapiAnnotationSetText(msg11_annotation, cmessage11);


}
