#include "main.hpp"

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
    std::string message8 = "Right ALT + M to display UACS HUD";
    std::string message9 = "Brake with spacebar";
    std::string message10 = "Shift forward/reverse with comma (,) and period (.)";
    std::string message11 = "Switch between forward view (F) and tire view (V)";
    std::string message12 = "Toggle headlights with L";

    if (drive_status == 'F') {
        message3 = "D";
    } else if (drive_status == 'N') {
        message4 = "N";
    } else if (drive_status == 'R') {
        message5 = "R";
    }

    if (msg1_annotation) oapiAnnotationSetText(msg1_annotation, message1.c_str());
    if (msg2_annotation) oapiAnnotationSetText(msg2_annotation, message2.c_str());
    if (msg3_annotation) oapiAnnotationSetText(msg3_annotation, message3.c_str());
    if (msg4_annotation) oapiAnnotationSetText(msg4_annotation, message4.c_str());
    if (msg5_annotation) oapiAnnotationSetText(msg5_annotation, message5.c_str());
    if (msg6_annotation) oapiAnnotationSetText(msg6_annotation, message6.c_str());
    if (msg7_annotation) oapiAnnotationSetText(msg7_annotation, message7.c_str());
    if (msg8_annotation) oapiAnnotationSetText(msg8_annotation, message8.c_str());
    if (msg9_annotation) oapiAnnotationSetText(msg9_annotation, message9.c_str());
    if (msg10_annotation) oapiAnnotationSetText(msg10_annotation, message10.c_str());
    if (msg11_annotation) oapiAnnotationSetText(msg11_annotation, message11.c_str());
    if (msg12_annotation) oapiAnnotationSetText(msg12_annotation, message12.c_str());

}
