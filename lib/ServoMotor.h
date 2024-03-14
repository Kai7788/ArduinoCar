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
    int pos = 90; //Standard position 90 degrees
    boolean rotate = false;

public:
    void set_servo(Servo new_servo){
        this->servo = new_servo;
        servo.write(pos);
    }
    void start_rotate(int delay_time = 20){
        this->rotate = true;
        while(rotate){
            Serial.println("servo start");
            for(pos = 0; pos <= 180; pos++){
                servo.write(pos);
                delay(delay_time);
            }
            for(pos = 180; pos >= 0; pos--){
                servo.write(pos);
                delay(delay_time);
            }
        }
    }
    void stop(){
        this->rotate = false;
    }

};

#endif //ARDUINOCAR_SERVOMOTOR_H
