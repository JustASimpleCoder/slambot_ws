

const int right_front_pin = 6;
const int right_front_dir1_pin = 4;
const int right_front_dir2_pin = 7;

const int right_back_pin = 5;
const int right_back_dir1_pin = 2;
const int right_back_dir2_pin = 3;

const int left_front_pin = 9;
const int left_front_dir1_pin = 11;
const int left_front_dir2_pin = 8;

const int left_back_pin = 10;
const int left_back_dir1_pin = 13;
const int left_back_dir2_pin = 12;

// for the DC motor, sometimes a full high 
int start_speed = 255;
int motor_speed = 255;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(right_front_pin,OUTPUT);
  pinMode(right_front_dir1_pin,OUTPUT);
  pinMode(right_front_dir2_pin,OUTPUT);
  
  pinMode(right_back_pin,OUTPUT);
  pinMode(right_back_dir1_pin,OUTPUT);
  pinMode(right_back_dir2_pin,OUTPUT);

  pinMode(left_front_pin,OUTPUT);
  pinMode(left_front_dir1_pin,OUTPUT);
  pinMode(left_front_dir2_pin,OUTPUT);
  
  pinMode(left_back_pin,OUTPUT);
  pinMode(left_back_dir1_pin,OUTPUT);
  pinMode(left_back_dir2_pin,OUTPUT);
  Serial.begin(9600);

}  


void forward_n_milisec(int n_milisec){
   
     digitalWrite(right_front_dir1_pin, HIGH);
     digitalWrite(right_front_dir2_pin, LOW);
      
     digitalWrite(right_back_dir1_pin, LOW);
     digitalWrite(right_back_dir2_pin, HIGH);
      
     digitalWrite(left_front_dir1_pin, HIGH);
     digitalWrite(left_front_dir2_pin, LOW);
      
     digitalWrite(left_back_dir1_pin, LOW);
     digitalWrite(left_back_dir2_pin, HIGH);


     analogWrite(right_front_pin, start_speed);
     analogWrite(right_back_pin, start_speed);
     analogWrite(left_front_pin, start_speed);
     analogWrite(left_back_pin, start_speed);
    
     analogWrite(right_front_pin, motor_speed);
     analogWrite(right_back_pin, motor_speed);
     analogWrite(left_front_pin, motor_speed);
     analogWrite(left_back_pin, motor_speed);
     
     delay(n_milisec);
     
     stop_robot();
     
}
void backward_n_milisec(int n_milisec){
   
     digitalWrite(right_front_dir1_pin, LOW);
     digitalWrite(right_front_dir2_pin, HIGH);
      
     digitalWrite(right_back_dir1_pin, HIGH);
     digitalWrite(right_back_dir2_pin, LOW);
      
      
     digitalWrite(left_front_dir1_pin, LOW);
     digitalWrite(left_front_dir2_pin, HIGH);
      
     digitalWrite(left_back_dir1_pin, HIGH);
     digitalWrite(left_back_dir2_pin, LOW);
      

     analogWrite(right_front_pin, start_speed);
     analogWrite(right_back_pin, start_speed);
     analogWrite(left_front_pin, start_speed);
     analogWrite(left_back_pin, start_speed);
     
     delay(10);
     
     analogWrite(right_front_pin, motor_speed);
     analogWrite(right_back_pin, motor_speed);
     analogWrite(left_front_pin, motor_speed);
     analogWrite(left_back_pin, motor_speed);
     
     delay(n_milisec);
     
     stop_robot();
     
}

void left_side_move_n_milisec(int n_milisec){
     Serial.println("moving Left side");
     digitalWrite(left_front_dir1_pin, HIGH);
     digitalWrite(left_front_dir2_pin, LOW);
      
     digitalWrite(left_back_dir1_pin, LOW);
     digitalWrite(left_back_dir2_pin, HIGH);

     
     analogWrite(left_front_pin, start_speed);
     analogWrite(left_back_pin, start_speed);
     
     delay(10);
     
     analogWrite(left_front_pin, motor_speed);
     analogWrite(left_back_pin, motor_speed);
     
     delay(n_milisec);
     
     stop_robot();
}


void right_side_move_n_milisec(int n_milisec){
     Serial.println("moving right side");
      
     digitalWrite(right_front_dir1_pin, HIGH);
     digitalWrite(right_front_dir2_pin, LOW);
      
     digitalWrite(right_back_dir1_pin, LOW);
     digitalWrite(right_back_dir2_pin, HIGH);

     
     analogWrite(right_front_pin, start_speed);
     analogWrite(right_back_pin, start_speed);
     
     delay(10);
     
     analogWrite(right_front_pin, motor_speed);
     analogWrite(right_back_pin, motor_speed);
     
     delay(n_milisec);
     
     stop_robot();
}

void right_turn(int n_milisec){
     digitalWrite(left_front_dir1_pin, LOW);
     digitalWrite(left_front_dir2_pin, HIGH);
      
     digitalWrite(left_back_dir1_pin, HIGH);
     digitalWrite(left_back_dir2_pin, LOW);

           
     digitalWrite(right_front_dir1_pin, HIGH);
     digitalWrite(right_front_dir2_pin, LOW);
      
     digitalWrite(right_back_dir1_pin, LOW);
     digitalWrite(right_back_dir2_pin, HIGH);
   
     analogWrite(right_front_pin, start_speed);
     analogWrite(right_back_pin, start_speed);
     analogWrite(left_front_pin, start_speed);
     analogWrite(left_back_pin, start_speed);
     
     delay(10);
     
     analogWrite(right_front_pin, motor_speed);
     analogWrite(right_back_pin, motor_speed);
     analogWrite(left_front_pin, motor_speed);
     analogWrite(left_back_pin, motor_speed);
     
     delay(n_milisec);
     
     stop_robot();
}

void left_turn(n_milisec){
     
}

void stop_robot(){
     analogWrite(right_front_pin, 0);
     analogWrite(right_back_pin, 0);
     analogWrite(left_front_pin, 0);
     analogWrite(left_back_pin, 0);
}

void loop() {
  if (Serial.available() > 0) {
    int command = Serial.parseInt();
    Serial.println(command);
    switch(command){
      case 0:
        stop_robot();
        break;
      case 1:
        forward_n_milisec(500);
        break;
      case 2:
        backward_n_milisec(500);
        break;
      case 3:
        left_side_move_n_milisec(500);
        break;
      case 4:
        right_side_move_n_milisec(500);
        break;
      case 5: 
        right_turn(500);
        break;
      case 6: 
        left_turn(500);
        break;
    }

    byte = Serial.read(command);
    switch(command){
      case 'w':
        forward_n_milisec(500);
        break;
      case 's':
        left_turn(500);
        break;
      case 'a':
        backward_n_milisec(500);
        break;
      case 'd':
        left_side_move_n_milisec(500);
        break;
    }
}