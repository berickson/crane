#include "esp32_common.h"
#include <Servo.h>

const int pin_crane_servo = 2;
Servo crane_servo;

bool is_between(float x, float a, float b) {
  return (x >= a && x <= b) || (x>=b && x <= a);
}

class Crane {
  public:
    float angle_pv = 90;
    float angle_sp = 90;
    float rate = 0;
    bool moving = false;
    uint32_t last_execute_ms = 0;

    void setup() {
      crane_servo.attach(pin_crane_servo);
    }

    void execute() {
      uint32_t ms = millis();

      float elapsed = (ms-last_execute_ms)/1000.;
      if(moving) {
        auto new_angle = angle_pv + rate * elapsed;
        if(is_between(angle_sp, angle_pv, new_angle)) {
          new_angle = angle_sp;
          moving = false;
        } 
        angle_pv = constrain(new_angle,-360,360);

      }
      crane_servo.writeMicroseconds(map(angle_pv*100, 0, 18000, 544, 2400));
      last_execute_ms = ms;
    }

    void go_to_angle(float angle, float rate) {
      angle = constrain(angle, -360., 360.);
      rate = constrain(rate, 1., 1000.);

      if(angle != angle_pv) {
        this->rate = ((angle - angle_pv)>0?1:-1) * rate;
        moving = true;
      }
      angle_sp = angle;
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
  env.cout.print(crane.angle_sp);
  env.cout.print(",\"angle_pv\":");
  env.cout.print(crane.angle_pv);
  env.cout.print(",\"rate\":");
  env.cout.print(crane.rate);
  env.cout.print(",\"moving\":");
  env.cout.print(crane.moving?"true":"false");
  env.cout.println("}");
}
 

void setup() {
  esp32_common_setup();
  Serial.println("crane started");

  commands.emplace_back(Command{"g", cmd_go, "go {angle=90} {rate=1000}"});
  commands.emplace_back(Command{"s", cmd_status, "status"});

  crane.setup();

  // done
  Serial.println("setup complete");
}

void loop() {
  static uint32_t last_loop_ms = 0;
  uint32_t loop_ms = millis();
  esp32_common_loop();
  crane.execute();

  if(every_n_ms(loop_ms, last_loop_ms, 100)) {
    display.clear();
    display.drawString(0, 0, "crane");
    display.drawString(0, 10, String(crane.angle_pv,2));
    display.drawString(0, 30, WiFi.localIP().toString());
    display.display();
  }
  last_loop_ms = loop_ms;
}