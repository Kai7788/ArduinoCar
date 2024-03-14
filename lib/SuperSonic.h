//
// Created by kai on 13.03.24.
//

#ifndef ARDUINOCAR_SUPERSONIC_H
#define ARDUINOCAR_SUPERSONIC_H
#include "../lib/pin_belegungen.h"

class SuperSonic{
public:
    float get_distance(){
        float duration;
        digitalWrite(ul_sonic_trig, LOW);
        delayMicroseconds(2);
        digitalWrite(ul_sonic_trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(ul_sonic_trig, LOW);
        return (pulseIn(ul_sonic_echo, HIGH) * 0.0343)/2;
    }

};

#endif //ARDUINOCAR_SUPERSONIC_H
