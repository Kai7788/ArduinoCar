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
#define bt_modul SoftwareSerial(bt_rx,bt_tx)
int ls_rechts, ls_links, ls_mitte = 0;

class Hbridge{
  // Speed kann zwischen 0 und 255 sein 0 => min; 255 => max
  public:
    void drive_forward(int speed = 200){
      analogWrite(h_br_en1, speed);
      analogWrite(h_br_en2, speed);      
      motor_left_forward();
      motor_right_forward();
    }



  private:

    void motor_left_forward(){
      //en1 pin 6
      digitalWrite(h_br_in1, HIGH);  // Motor 1 vor
      digitalWrite(h_br_in2, LOW);

    }

      void motor_right_forward(){
      digitalWrite(h_br_in3, LOW);  // Motor 1 vor
      digitalWrite(h_br_in4, HIGH);

    }

};
class LineTracking{

  public:
    void get_sensor_vals(int &links,int &mitte,int &rechts){
      // Links, Mitte, Rechts
      links = !digitalRead(lt_links);
      mitte = !digitalRead(lt_mitte);
      rechts = !digitalRead(lt_rechts);
      
      return;
    }
};
class IDrivable{
  public:
      virtual ~IDrivable() = default;
      virtual void drive(String) = 0;

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



class Car : IDrivable {
  public:
    String mode{};
    ServoMotor servo_motor = ServoMotor();
    Hbridge h_bruecke = Hbridge();
    LineTracking line_tracking_modul = LineTracking();
    bool on_off = false;


    explicit Car(Servo new_servo) {
      mode = "none";
      this->servo_motor.set_servo(new_servo);
    }

    void start(){
      this->on_off = true;
      while(on_off){
        //line_tracking_modul.get_sensor_vals(ls_links, ls_mitte, ls_rechts);
        
      }
    }

    void print_cords(){
      int** data_array = servo_motor.get_cords();
      for (int i = 0; i < 13; ++i) {
        Serial.print("[");
        Serial.print(i);
        Serial.print("] Degrees: ");
        Serial.print(data_array[i][0]);
        Serial.print(" Distance: ");
        Serial.print(data_array[i][1]);
        Serial.println();
    }
      deconstruct_data_array(data_array);
      //servo_motor.deconstruct_data_array();
    }

  private:
    int** data_array;

    void deconstruct_data_array(int** data_array){
      for (int i = 0; i < 13; ++i) {
        delete[] data_array[i];
    }
        delete[] data_array;
    }
    

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
    bt_modul.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!car.on_off){
    //car.start();
    car.print_cords();
  }
  

}