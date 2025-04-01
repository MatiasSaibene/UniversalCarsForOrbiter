#include "main.hpp"

void UCFO::MakeContact_TouchdownPoints(){

    front_right_wheel_contact = _V(0.689, -wheel_radius - travel, 1.392);
    front_left_wheel_contact = _V(-0.689, -wheel_radius - travel, 1.392);
    rear_right_wheel_contact = _V(0.689, -wheel_radius - travel, -1.084);
    rear_left_wheel_contact = _V(-0.689, -wheel_radius - travel, -1.084);

    TDP1 = _V(0.703, 0.000, -1.670);
    TDP2 = _V(0.703, 0.000, 1.817);
    TDP3 = _V(-0.703, 0.000, -1.670);
    TDP4 = _V(-0.703, 0.000, 1.817);
    TDP5 = _V(0.703, 0.762, -1.670);
    TDP6 = _V(0.703, 0.762, 1.817);
    TDP7 = _V(-0.703, 0.762, -1.670);
    TDP8 = _V(-0.703, 0.762, 1.817);
    
}