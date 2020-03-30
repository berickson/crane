#include <Arduino.h>

#include "SSD1306Wire.h"
#include <Servo.h>

#include "esp32_common.h"

#include <BluetoothSerial.h>

// board at https://www.amazon.com/gp/product/B07DKD79Y9
/*
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_scl = 15;
const int pin_oled_rst = 16;
*/
const int pin_crane_servo = 2;

//SSD1306Wire display(oled_address, pin_oled_sda, pin_oled_scl);
Servo crane_servo;
BluetoothSerial bluetooth;

bool is_between(float x, float a, float b) {
  return (x >= a && x <= b) || (x>=b && x <= a);
}

class Crane {
  public:
    float servo_angle = 90;
    float set_angle = 90;
    float set_rate = 0;
    bool moving = false;
    uint32_t last_execute_ms = 0;

    void setup() {
      crane_servo.attach(pin_crane_servo);
    }

    void execute() {
      uint32_t ms = millis();

      float elapsed = (ms-last_execute_ms)/1000.;
      if(moving) {
        auto new_angle = servo_angle + set_rate * elapsed;
        if(is_between(set_angle, servo_angle, new_angle)) {
          new_angle = set_angle;
          moving = false;
        } 
        servo_angle = constrain(new_angle,-360,360);

      }
      crane_servo.writeMicroseconds(map(servo_angle*100, 0, 18000, 544, 2400));
      last_execute_ms = ms;
    }

    void go_to_angle(float angle, float rate) {
      angle = constrain(angle, -360., 360.);
      rate = constrain(rate, 1., 1000.);

      if(angle != servo_angle) {
        set_rate = ((angle - servo_angle)>0?1:-1) * rate;
        moving = true;
      }
      set_angle = angle;
    }
};

Crane crane;

void cmd_go(CommandEnvironment & env) {
  float angle = 90;
  float rate = 1000;
  if(env.args.getParamCount() >= 1) {
    angle = atof(env.args.getCmdParam(1));
  }
  if(env.args.getParamCount() >= 2) {
    rate = atof(env.args.getCmdParam(2));
  }
  crane.go_to_angle(angle, rate);
}

void cmd_status(CommandEnvironment & env) {
  env.cout.print("{\"angle_sp\":");
  env.cout.print(crane.set_angle);
  env.cout.print(",\"angle_pv\":");
  env.cout.print(crane.servo_angle);
  env.cout.print(",\"rate\":");
  env.cout.print(crane.set_rate);
  env.cout.print(",\"moving\":");
  env.cout.print(crane.moving?"true":"false");
  env.cout.println("}");
}
 

void setup() {
  esp32_common_setup();

  commands.emplace_back(Command{"g", cmd_go, "go {angle=90} {rate=1000}"});
  commands.emplace_back(Command{"s", cmd_status, "status"});

  Serial.println("crane started");

  bluetooth.begin("crane");
  crane.setup();

  // initialize the display
  /*
  pinMode(pin_oled_rst, OUTPUT);
  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically();
  */

  // initialize servo



  // done
  Serial.println("setup complete");
}

void loop() {
  static uint32_t last_loop_ms = 0;
  uint32_t loop_ms = millis();
  esp32_common_loop();
  crane.execute();


/*
  while(bluetooth.available()) {
    Serial.write(bluetooth.read());
  }
  if(servo_angle < min_servo_angle) {
    servo_angle = min_servo_angle;
    servo_increment = fabs(base_servo_increment);
    // delay(5000);
  }
  if(servo_angle > max_servo_angle) {
    servo_angle = max_servo_angle;
    servo_increment = -100*fabs(base_servo_increment);
    // delay(5000);

  }
*/
  if(every_n_ms(loop_ms, last_loop_ms, 100)) {
    display.clear();
    display.drawString(0, 0, "crane");
    display.drawString(0, 10, String(crane.servo_angle,2));
    display.drawString(0, 30, WiFi.localIP().toString());
    display.display();
  }
  last_loop_ms = loop_ms;
}