//
// Created by kai on 13.03.24.
//
#include <Servo.h>

#ifndef ARDUINOCAR_SERVOMOTOR_H
#define ARDUINOCAR_SERVOMOTOR_H
#include "../lib/pin_belegungen.h"

class ServoMotor{
private:
    Servo servo;
    int pos = 0; //Standard position 90 degrees
    boolean rotate = false;

public:
    ServoMotor(){
        this->servo.attach(servo_out);
        servo.write(pos);
    }

    void start(){
        this->rotate = true;
        while(rotate){
            for(pos; pos >= 0; pos--){
                servo.write(pos);
            }
            for(pos; pos <= 180; pos++){
                servo.write(pos);
            }
        }
    }
    void stop(){
        this->rotate = false;
    }

};

#endif //ARDUINOCAR_SERVOMOTOR_H
