#include <Servo.h>
#include <SoftwareSerial.h>
#include <IRremote.h>


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
int ls_rechts, ls_links, ls_mitte = 0;

enum class IR_VALUE{
  NICHTS      = 0,
  VOR         = 70,
  LINKS       = 68,
  RECHTS      = 67,
  ZURRUECK    = 21,
  OK          = 64,
  VAL_1       = 22,
  VAL_2       = 25,
  VAL_3       = 13,
  VAL_4       = 12,
  VAL_5       = 24,
  VAL_6       = 94,
  VAL_7       = 8,
  VAL_8       = 28,
  VAL_9       = 90,
  VAL_0       = 82,
  STERN       = 66,
  HASHTAG     = 74
};

class Hbridge {
public:
  void drive_forward(int speed = 200) {
    motor_left_forward();
    motor_right_forward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, 150);
  }

  void drive_backward(int speed = 200) {
    motor_left_backward();
    motor_right_backward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed);
  }

  void stop() {
    // Stop the motors by setting their enable pins to 0
    analogWrite(h_br_en1, 0);
    analogWrite(h_br_en2, 0);
  }

  void turn_right(int speed = 200) {
    //Serial.println("Rechts");
    motor_left_forward();
    motor_right_forward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed/4);
  }

  void turn_right_90_degrees(int speed = 200) {
    // Stop the car
    stop();
    
    // Set the motors to turn right
    digitalWrite(h_br_in1, LOW);
    digitalWrite(h_br_in2, HIGH);
    digitalWrite(h_br_in3, HIGH);
    digitalWrite(h_br_in4, LOW);
    analogWrite(h_br_en1, 125);
    analogWrite(h_br_en2, 1);

    // Adjust the delay time to make the car turn approximately 90 degrees
    delay(3000); // Adjust as needed

    // Stop the car after turning
    stop();
  }

  void turn_left(int speed = 200) {
    // Implement logic to turn left
    // For example, stop left motor and move right motor forward
    //Serial.println("Links");
    motor_left_forward();
    motor_right_forward();

    analogWrite(h_br_en1, speed/4);
    analogWrite(h_br_en2, speed);
  }

  void turn_left_90_degrees(int speed = 200) {
    // Stop the car
    stop();
    
    // Set the motors to left right
    digitalWrite(h_br_in1, HIGH);
    digitalWrite(h_br_in2, LOW);
    digitalWrite(h_br_in3, LOW);
    digitalWrite(h_br_in4, HIGH);
    analogWrite(h_br_en1, 125);
    analogWrite(h_br_en2, 1);

    // Adjust the delay time to make the car turn approximately 90 degrees
    delay(3000); // Adjust as needed

    // Stop the car after turning
    stop();
  }

private:
  void motor_left_forward() {
    digitalWrite(h_br_in1, HIGH);  // Motor 1 vor
    digitalWrite(h_br_in2, LOW);
  }

  void motor_right_forward() {
    digitalWrite(h_br_in3, LOW);  // Motor 1 vor
    digitalWrite(h_br_in4, HIGH);
  }

  void motor_left_backward() {
    digitalWrite(h_br_in1, LOW);  // Motor 1 rückwärts
    digitalWrite(h_br_in2, HIGH);
  }

  void motor_right_backward() {
    digitalWrite(h_br_in3, HIGH);  // Motor 1 rückwärts
    digitalWrite(h_br_in4, LOW);
  }

  void motor_right_stop() {
    digitalWrite(h_br_in3, LOW);  // Motor 1 vor
    digitalWrite(h_br_in4, LOW);
  }

  void motor_left_stop() {
    digitalWrite(h_br_in1, LOW);  // Motor 1 vor
    digitalWrite(h_br_in2, LOW);
  }
};


class LineTracking{
  public:
    explicit LineTracking(Hbridge h_bridge){
      this->h_bridge = h_bridge;
    }

    void line_tracking(){
      int left,middle,right;
      get_sensor_vals(left,middle, right);
      if(!left && !middle && !right){
        //No line found
        this->h_bridge.drive_forward();
      } else if (!left && middle && !right) {
        //Only line in middle
        this->h_bridge.drive_forward();
      }else if (left && middle && !right) {
      //drive right
        this->h_bridge.turn_right();
      }else if (!left && middle && right) {
      //drive left
        this->h_bridge.turn_left();
      }else if (left && !middle && !right) {
      //drive right
        this->h_bridge.turn_right();
      }else if (!left && !middle && right) {
      //drive left
        this->h_bridge.turn_left();
      }

    }

  private:
    Hbridge h_bridge;

    void get_sensor_vals(int &links,int &mitte,int &rechts){
      // Links, Mitte, Rechts
      links = !digitalRead(lt_links);
      mitte = !digitalRead(lt_mitte);
      rechts = !digitalRead(lt_rechts);
      
      return;
    }
};

class SuperSonic{

  public:
      int get_distance(){
          //Returns the Distance in mm
          digitalWrite(ul_sonic_trig, LOW);
          delayMicroseconds(2);
          digitalWrite(ul_sonic_trig, HIGH);
          delayMicroseconds(10);
          digitalWrite(ul_sonic_trig, LOW);
          unsigned long duration = pulseIn(ul_sonic_echo, HIGH);
          return int((duration * 0.0343)/2);
      }

  };


class ServoMotor{
  private:
      Servo servo;
      int start_pos = 90; //Standard position 90 degrees
      SuperSonic sonic_sensor = SuperSonic();
      int min_pos = 30;
      int max_pos = 150;
      int pos = 90;
      int** data_array;

  public:
      void set_servo(Servo new_servo){
          this->servo = new_servo;
          servo.write(start_pos);
      }

      int** get_cords(int delay_time = 20){
        // Returns an Array with the cordinates exmp. [[80,40],[70,44],[60, 55]]
        // Coardinate array -> Inner array [degrees, distance]
        // Left is > 90 and right is < 90
        int entries = 13;
        int step = 0;
        int** data_array = new int*[entries]; // Creates the outer array [12] with 12 spaces

        for (int i = 0; i < entries; ++i) {
          data_array[i] = new int[2];        // Creates the inner arrays [[2], [2], [2]...] -> [degrees, value]
        }
        if(pos < 90){
          servo.write(30);
          delay(delay_time*3);

          for(pos = 30; pos <= 150; pos++){
            servo.write(pos);
            delay(delay_time);
            if((pos % 10) == 0){
              data_array[step][0] = pos;
              data_array[step][1] = sonic_sensor.get_distance();
              step++;
            }
          }  
        }  else {
          servo.write(150);
          delay(delay_time*3);

          for(pos = 150; pos >= 30; pos--){
            servo.write(pos);
            delay(delay_time);
            if((pos % 10) == 0){
              data_array[step][0] = pos;
              data_array[step][1] = sonic_sensor.get_distance();
              step++;
            }
          }
        } 
        //this->data_array = data_array;
        return data_array;
    }

    void deconstruct_data_array(){
      for (int i = 0; i < 13; ++i) {
        delete[] this->data_array[i];
      }
        delete[] data_array;
      }
      

  };



class Car {
  public:
    String mode{};
    ServoMotor servo_motor = ServoMotor();
    Hbridge h_bruecke = Hbridge();
    LineTracking line_tracking_modul = LineTracking(h_bruecke);
    bool on_off = false;
    int** data_array;
    String richtung{}; //WIRD FUER DEN MANUELLEN MODUS VERWENDET


    explicit Car(Servo new_servo) {
      mode = "manuell";
      this->servo_motor.set_servo(new_servo);
      this->richtung = "STOP";
    }

    void start(){
      this->on_off = true;
      while(on_off){
        while(this->mode.equals("line_tracking")){
          line_tracking_modul.line_tracking();
          infra_red_handler();
        }
        while (this->mode.equals("manuell")) {
          manuell();
          infra_red_handler();
        }
        while (this->mode.equals("ausweichen")) {

          //Funktion fuer das Automatisch Fahren mit Ausweichen hier einfuergen
          //ausweichen
          infra_red_handler();
        }
        
      }
    }

    void manuell(){
      Serial.println(this->richtung);
      if(this->richtung.equals("VOR")){
        h_bruecke.drive_forward();
      }else if (this->richtung.equals("ZURRUECK")) {
        h_bruecke.drive_backward();
      }else if (this->richtung.equals("RECHTS")) {
        h_bruecke.turn_left();
      }else if (this->richtung.equals("LINKS")) {
        h_bruecke.turn_right();
      }else {
        h_bruecke.stop();
      }
    }


    void print_cords(){
      int** data_array = servo_motor.get_cords();
      for (int i = 0; i < 13; ++i) {
       /* Serial.print("[");
        Serial.print(i);
        Serial.print("] Degrees: ");
        Serial.print(data_array[i][0]);
        Serial.print(" Distance: ");
        Serial.print(data_array[i][1]);
        Serial.println(); */
    }
      deconstruct_data_array(data_array);
      //servo_motor.deconstruct_data_array();
    }


    void deconstruct_data_array(int** data_array){
      for (int i = 0; i < 13; ++i) {
        delete[] data_array[i];
    }
        delete[] data_array;
    }
    void infra_red_handler(){
      if (IrReceiver.decode()) {
        IR_VALUE value = (IR_VALUE) IrReceiver.decodedIRData.command;
        IrReceiver.resume();
        switch (value) {
          //Modus wechsel => # + 1/2/3
          case IR_VALUE::HASHTAG:
            while (!IrReceiver.decode()) {
              continue;
            }
            switch ((IR_VALUE) IrReceiver.decodedIRData.command){
              case IR_VALUE::VAL_1:
              this->mode = "manuell";
              Serial.println("manuell");
              break;
            case IR_VALUE::VAL_2:
              this->mode = "ausweichen";
              Serial.println("ausweichen");
              break;
            case IR_VALUE::VAL_3:
              this->mode = "line_tracking";
              Serial.println("line_tracking");
              break;
            default:
              break;
            }
          case IR_VALUE::VOR:
            this->richtung = "VOR";
            break;
          case IR_VALUE::ZURRUECK:
            this->richtung = "ZURRUECK";
            break;
          case IR_VALUE::RECHTS:
            this->richtung = "RECHTS";
            break;
          case IR_VALUE::LINKS:
            this->richtung = "LINKS";
            break;
          case IR_VALUE::OK:
            this->richtung = "STOP";
          default:
            break;
        }
    }
  }}
  ;




Servo servo_motor;
Car car(servo_motor);



void setup() {
  // put your setup code here, to run once:
    //delay(5000);
    Serial.begin(9600);
    IrReceiver.begin(infra_red);
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
    car.start();
  }
