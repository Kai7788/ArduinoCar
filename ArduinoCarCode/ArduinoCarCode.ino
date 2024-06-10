#include <Servo.h>
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
  // Nachschlage Liste fuer Infrarot werte
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

enum class BT_VALUE{
  //Nachschlage Liste fuer Bluetooth werte
  VOR =             'f',
  VORLINKS =        'q',
  VORRECHTS =       'e',
  ZURRUECK =        'z',
  ZURRUECKLINKS =   'x',
  ZURRUECKRECHTS =  'c',
  RECHTS =          'r',
  LINKS =           'l',
  STOP =            's',
  MANUELL =         'm',
  TRACKING =        't',
  AUTOMATIK =       'a'

};

class Hbridge {
  //H-Bruecken Klasse, um das Steuern der Motoren zu vereinfachen
public:
  void drive_forward(int speed = 200) {
    //Funktion zum Vorwaertsfahren
    motor_left_forward();
    motor_right_forward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed * 0.75);
  }

  void drive_backward(int speed = 200) {
    //Funktion zum Rueckwertsfahren
    motor_left_backward();
    motor_right_backward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed * 0.75);
  }

  void stop() {
    //Funktion zum Stoppen der Motoren
    analogWrite(h_br_en1, 0);
    analogWrite(h_br_en2, 0);
  }

  void turn_right(int speed = 200) {
    // Im bogen rechts einbiegen
    motor_left_forward();
    motor_right_forward();

    analogWrite(h_br_en1, speed/4);
    analogWrite(h_br_en2, speed);
  }

  void turn_right_90_degrees(int speed = 125) {
    // Faehrt im Bogen 90 grad rueckwerts
    stop();

    digitalWrite(h_br_in1, LOW);
    digitalWrite(h_br_in2, HIGH);
    digitalWrite(h_br_in3, HIGH);
    digitalWrite(h_br_in4, LOW);
    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, 1);

    delay(3000); // Adjust as needed

    stop();
  }

  void turn_left(int speed = 200) {
    // Im bogen Links einbiegen

    motor_left_forward();
    motor_right_forward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed/4);
  }

  void turn_left_backward(int speed = 200) {
    // Im bogen Links einbiegen

    motor_left_backward();
    motor_right_backward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed/4);
  }

  void turn_right_backward(int speed = 200) {
    // Im bogen Links einbiegen

    motor_left_backward();
    motor_right_backward();

    analogWrite(h_br_en1, speed/4);
    analogWrite(h_br_en2, speed);
  }

  void turn_left_90_degrees(int speed = 125) {
    // Faehrt im Bogen 90 grad rueckwerts
    stop();
    digitalWrite(h_br_in1, HIGH);
    digitalWrite(h_br_in2, LOW);
    digitalWrite(h_br_in3, LOW);
    digitalWrite(h_br_in4, HIGH);
    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, 1);

    delay(3000); // Adjust as needed

    stop();
  }
  
  void spin_right(int speed = 200){
    // Auf der Stelle rechts drehen
    motor_right_forward();
    motor_left_backward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed);
  }

  void spin_left(int speed = 200){
    // Auf der Stelle links drehen
    motor_right_backward();
    motor_left_forward();

    analogWrite(h_br_en1, speed);
    analogWrite(h_br_en2, speed);
  }

private:
  void motor_left_forward() {
    //Motor links vorwaerts
    digitalWrite(h_br_in1, HIGH);  // Motor 1 vor
    digitalWrite(h_br_in2, LOW);
  }

  void motor_right_forward() {
    //Motor rechts vorwaerts
    digitalWrite(h_br_in3, LOW);  // Motor 1 vor
    digitalWrite(h_br_in4, HIGH);
  }

  void motor_left_backward() {
    //Motor links rueckwaerts
    digitalWrite(h_br_in1, LOW);  // Motor 1 rückwärts
    digitalWrite(h_br_in2, HIGH);
  }

  void motor_right_backward() {
    //Motor rechts rueckwerts
    digitalWrite(h_br_in3, HIGH);  // Motor 1 rückwärts
    digitalWrite(h_br_in4, LOW);
  }

  void motor_right_stop() {
    //Motor rechts stoppen
    digitalWrite(h_br_in3, LOW);  // Motor 1 vor
    digitalWrite(h_br_in4, LOW);
  }

  void motor_left_stop() {
    // Motor links stoppen
    digitalWrite(h_br_in1, LOW);  // Motor 1 vor
    digitalWrite(h_br_in2, LOW);
  }
};

class LineTracking{
  // Linetracking Modul, um das Steuern zu vereinfachen
  public:
    explicit LineTracking(Hbridge h_bridge){
      this->h_bridge = h_bridge;
    }

    void line_tracking(){
      //Linetracking Funktion
      get_sensor_vals();
      if (ls_mitte) {
          h_bridge.drive_forward(150);
      } else if (ls_rechts) {
          h_bridge.spin_right();
          
          while (ls_rechts) {
            h_bridge.spin_right();
            get_sensor_vals();
          }
          h_bridge.stop(); 
      } else if (ls_links) {
          h_bridge.spin_left();
          
          while (ls_links) {
            get_sensor_vals();
            h_bridge.spin_left();
          }
          h_bridge.stop(); 
      } else {
          
          h_bridge.stop();
      }
    }

  private:
    Hbridge h_bridge; // Benoetig Zugriff auf die H-Bruecke zum steuern der Motoren
    void get_sensor_vals(){
      //Die Aktuellen Sensorwerte einlesen
      // Links, Mitte, Rechts
      ls_links = !digitalRead(lt_links);
      ls_mitte = !digitalRead(lt_mitte);
      ls_rechts = !digitalRead(lt_rechts);
      
      return;
    }
};

class SuperSonic{ // Klasse fuer den Ultraschall sensor zum vereinfachen der Steuerung

  public:
      int get_distance(){ 
          // Funktion zum erhalten des Frontralen abstand in mm 
          digitalWrite(ul_sonic_trig, LOW);
          delayMicroseconds(2);
          digitalWrite(ul_sonic_trig, HIGH);
          delayMicroseconds(10);
          digitalWrite(ul_sonic_trig, LOW);
          unsigned long duration = pulseIn(ul_sonic_echo, HIGH);
          return int((duration * 0.0343)/2);
      }

  };

class ServoMotor{ // Klasse zur vereinfachten Steuerung des Servo motors
  private:
      Servo servo; // Zugriff auf den Servo motor
      int start_pos = 90; //Standard position 90 degrees
      SuperSonic sonic_sensor = SuperSonic(); // Zugriff auf den Ultraschall sensor 
      int min_pos = 30;
      int max_pos = 150;
      int pos = 90;
      int** data_array;

  public:
      void set_servo(Servo new_servo){
          // Funktion zum setzen des Servos
          this->servo = new_servo;
          servo.write(start_pos); // Standart Position anfahren
      }

      int get_frontal_distance(){
        // Erhalten des Frontalen abstand, da die get_distance() Funktion Privat ist
        return this->sonic_sensor.get_distance();
      }

      int** get_cords(int delay_time = 20){
        // Erstellen eines 2D Arrays, um die Postionen zu erfassen bsp: [[80,40],[70,44],[60, 55]]
        //                                                              [[Grad, Abstand],[Grad, Abstand],[Grad,Abstand]...]
        int entries = 13;
        int step = 0;
        int** data_array = new int*[entries]; // Das grosses Array fuer 12  Eintraege erstellen

        for (int i = 0; i < entries; ++i) {
          data_array[i] = new int[2];        // Innere Arrays Erstellen [[2], [2], [2]...] -> [Grad, Wert]
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
        return data_array;
    }

    void deconstruct_data_array(){
      // Funktion zum Freigeben des Speichers nach nutzung des get_Cords Arrays
      for (int i = 0; i < 13; ++i) {
        delete[] this->data_array[i];
      }
        delete[] data_array;
      }
      

  };

class Car { // Klasse zum Steuern des Autos zum vereinfachen der Steuerung
  public:
    String mode{}; // Modus wie z.b 'manuell'
    ServoMotor servo_motor = ServoMotor(); //Servo Motor Initialisieren
    Hbridge h_bruecke = Hbridge(); // H-Bruecke Initialisieren
    LineTracking line_tracking_modul = LineTracking(h_bruecke); // LineTracking Modul Initialisieren
    bool on_off = false; // Auto An/Aus Schalten
    int** data_array; // Platzhalter fuer die get_cords() Funktion aus dem Servo Motor
    String richtung{}; //WIRD FUER DEN MANUELLEN MODUS VERWENDET


    explicit Car(Servo new_servo) {
      // Konstruktor der Klasse
      mode = "manuell";
      this->servo_motor.set_servo(new_servo);
      this->richtung = "STOP";
    }

    void start(){ // Funktion zum Starten des Autos bzw. unser Main Loop
      this->on_off = true;
      while(on_off){
        //Verschiedene Modis Starten
        while(this->mode.equals("line_tracking")){
          //Linetracking starten
          line_tracking_modul.line_tracking();
          // Schauen, ob es veraenderungen im Bluetooth Modul gibt, wie z.b neue Daten im Buffer
          bluetooth_handler();
        }
        while (this->mode.equals("manuell")) {
          //Manuell starten
          manuell();
          // Schauen, ob es veraenderungen im Bluetooth Modul gibt, wie z.b neue Daten im Buffer
          bluetooth_handler();
        }
        while (this->mode.equals("automatik")) {
          //Automatik starten
          automatisches_fahren_mit_ausweichen();
          // Schauen, ob es veraenderungen im Bluetooth Modul gibt, wie z.b neue Daten im Buffer
          bluetooth_handler();
        }
        
      }
    }

private:

    void automatisches_fahren_mit_ausweichen(int sicherheitsabstand = 20) {
      //Funktion fuer den Automatik modus
      int entfernung = this->servo_motor.get_frontal_distance();
      if (entfernung < sicherheitsabstand) {
          h_bruecke.drive_backward();
          delay(2000); // 2 Sekunden warten
          h_bruecke.stop(); // Anhalten
          h_bruecke.turn_right_90_degrees(); // Nach rechts abbiegen
          delay(1000); // 1 Sekunde warten
          h_bruecke.stop(); // Anhalten
          h_bruecke.drive_forward(); // Vorwärts fahren
        } else{
          h_bruecke.drive_forward();
        }
  }

    void manuell(){
      //Funktion fuer den Manuellen Modus
      if(this->richtung.equals("VOR")){
        h_bruecke.drive_forward(255);
      }else if (this->richtung.equals("VORRECHTS")) {
        h_bruecke.turn_right(255);
      }else if (this->richtung.equals("VORLINKS")) {
        h_bruecke.turn_left(255);
      }else if (this->richtung.equals("ZURRUECK")) {
        h_bruecke.drive_backward(255);
      }else if (this->richtung.equals("ZURRUECKLINKS")) {
        h_bruecke.turn_left_backward(255);
      }else if (this->richtung.equals("ZURRUECKRECHTS")) {
        h_bruecke.turn_right_backward(255);
      }
      
      else if (this->richtung.equals("RECHTS")) {
        h_bruecke.spin_right(255);
      }else if (this->richtung.equals("LINKS")) {
        h_bruecke.spin_left(255
        );
      }else {
        h_bruecke.stop();
      }
    }

    void print_cords(){
      // Funtion zum Printen der Aktuellen Koordinaten im Umkreis
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
      servo_motor.deconstruct_data_array();
    }

    void deconstruct_data_array(int** data_array){
      for (int i = 0; i < 13; ++i) {
        delete[] data_array[i];
    }
        delete[] data_array;
    }

    void bluetooth_handler(){
      // Funktion zum Handlen von Bluetooth Modul
      if(Serial.available()){ // Wenn Etwas im Buffer steht
        BT_VALUE value = (BT_VALUE) Serial.read();
        switch (value) { // Kontroll Logik
          case BT_VALUE::VOR: 
            this->richtung = "VOR"; 
            break;
          case BT_VALUE::VORLINKS:
            this->richtung = "VORLINKS";
            break;
          case BT_VALUE::VORRECHTS:
            this->richtung = "VORRECHTS";
            break;
          case BT_VALUE::ZURRUECK: 
            this->richtung = "ZURRUECK";
              break;
          case BT_VALUE::LINKS: 
            this->richtung = "LINKS";   
            break;
          case BT_VALUE::ZURRUECKLINKS: 
            this->richtung = "ZURRUECKLINKS";   
            break;
          case BT_VALUE::ZURRUECKRECHTS: 
            this->richtung = "ZURRUECKRECHTS";   
            break;
          case BT_VALUE::RECHTS: 
            this->richtung = "RECHTS";  
            break;
          case BT_VALUE::STOP: 
            this->richtung = "STOP";   
            break;
          case BT_VALUE::MANUELL: 
            this->mode = "manuell"; 
            break;
          case BT_VALUE::AUTOMATIK:
            this->mode = "automatik";
            break;
          case BT_VALUE::TRACKING:
            this->mode = "line_tracking";
            break;
          default:  
            break;
        }
      }
    }

    void infra_red_handler(){
      // Funktion zur Kontrolle via Infrarot
      if (IrReceiver.decode())// Wenn etwas im Infrarot buffer steht
       {
        IR_VALUE value = (IR_VALUE) IrReceiver.decodedIRData.command;
        IrReceiver.resume();
        switch (value) { // Kontroll Logik
          //Modus wechsel => # + 1/2/3
          case IR_VALUE::HASHTAG:
            while (!IrReceiver.decode()) {
              continue;
            }
            switch ((IR_VALUE) IrReceiver.decodedIRData.command){
              case IR_VALUE::VAL_1:
              this->mode = "manuell";
              break;
            case IR_VALUE::VAL_2:
              this->mode = "automatik";
              break;
            case IR_VALUE::VAL_3:
              this->mode = "line_tracking";
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
    Serial.begin(9600);
    IrReceiver.begin(infra_red);
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
