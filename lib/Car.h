//
// Created by kai on 13.03.24.
//
#include "Interfaces.h"
#include "SuperSonic.h"
#include "ServoMotor.h"

#ifndef ARDUINOCAR_CAR_H
#define ARDUINOCAR_CAR_H


class Car : IDrivable
        {
public:
    String mode;
    SuperSonic sonic_sensor;
    ServoMotor servo_motor;



    Car(){
        mode = "none";
    }

private:
    void drive(String direction) override{

    }
};


#endif //ARDUINOCAR_CAR_H
