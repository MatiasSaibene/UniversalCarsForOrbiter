#include "main.hpp"

void UCFO::ChangeSkin(){

    
}

void UCFO::NextSkin(){

    if(currentSkin >= 15){
        currentSkin = 0;
    }

    ChangeSkin();
    currentSkin++;
}