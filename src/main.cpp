#include <Arduino.h>
#include "../lib/Car.h"
#include "../lib/pin_belegungen.h"

Servo servo_motor;
void setup() {
    // write your initialization code here
    Serial.begin(9600);
    pinMode(lt_rechts, INPUT_PULLUP);
    pinMode(lt_mitte, INPUT_PULLUP);
    pinMode(lt_links, INPUT_PULLUP);
    pinMode(h_br_en1, OUTPUT);
    pinMode(h_br_en2, OUTPUT);
    pinMode(h_br_in1, OUTPUT);
    pinMode(h_br_in2, OUTPUT);
    pinMode(h_br_in3, OUTPUT);
    pinMode(h_br_in4, OUTPUT);
    pinMode(infra_red, INPUT_PULLUP);
    pinMode(ul_sonic_echo, INPUT_PULLUP);
    pinMode(ul_sonic_trig, OUTPUT);
    servo_motor.attach(servo_out);
}

Car car(servo_motor);
void loop() {
// write your code here
    car.servo_motor.start_rotate();

}