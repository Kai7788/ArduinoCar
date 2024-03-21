#include <Servo.h>
#include <SoftwareSerial.h>


#define lt_rechts 10 //Digital Input
#define lt_mitte 4   //Digital Input
#define lt_links 2   //Digital Input
#define servo_out 3  //Digital Output
#define h_br_en1 6
#define h_br_in1 11
#define h_br_in2 9
#define h_br_in3 8
#define h_br_in4 7
#define h_br_en2 5
#define ul_sonic_trig A5
#define ul_sonic_echo A4
#define infra_red 12
#define bt_rx 0
#define bt_tx 1

class Hbridge{
  public:
    void drive_forward(int speed = 200){
      Serial.println("Hier");
      motor_left_forward();
      motor_right_forward();

      analogWrite(h_br_en1, speed);
      analogWrite(h_br_en2, speed);
    }



  private:

    void motor_left_forward(){
      digitalWrite(h_br_in1, HIGH);  // Motor 1 vor
      digitalWrite(h_br_in2, LOW);

    }

      void motor_right_forward(){
      digitalWrite(h_br_in3, HIGH);  // Motor 1 vor
      digitalWrite(h_br_in4, LOW);

    }

};

class IDrivable{
  public:
      virtual ~IDrivable() = default;
      virtual void drive(String) = 0;

  };

class ServoMotor{
  private:
      Servo servo;
      int pos = 90; //Standard position 90 degrees


  public:
    boolean rotate = false;
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


class SuperSonic{

  public:
      float get_distance(){
          digitalWrite(ul_sonic_trig, LOW);
          delayMicroseconds(2);
          digitalWrite(ul_sonic_trig, HIGH);
          delayMicroseconds(10);
          digitalWrite(ul_sonic_trig, LOW);
          return (pulseIn(ul_sonic_echo, HIGH) * 0.0343)/2;
      }

  };


class Car : IDrivable {
  public:
    String mode{};
    SuperSonic sonic_sensor = SuperSonic();
    ServoMotor servo_motor = ServoMotor();
    SoftwareSerial bt_modul = SoftwareSerial(bt_rx,bt_tx);
    Hbridge h_bruecke = Hbridge();

    explicit Car(Servo new_servo) {
      mode = "none";
      this->servo_motor.set_servo(new_servo);
    }

    void start(){
      this->on_off = true;
      while(on_off){
        drive("l");
      }
    }

  private:
    bool on_off = false;

    void drive(String direction) override {
      h_bruecke.drive_forward();
    }
  };


Servo servo_motor;
Car car(servo_motor);


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    //SoftwareSerial.begin(9600);
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

void loop() {
  // put your main code here, to run repeatedly:

  car.start();

}