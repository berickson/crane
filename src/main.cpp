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

void setup() {
  esp32_common_setup();
  Serial.println("crane started");

  bluetooth.begin("crane");

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
  crane_servo.attach(pin_crane_servo);


  // done
  Serial.println("setup complete");
}

void loop() {
  esp32_common_loop();

  static float base_servo_increment = 0.02;
  static float servo_increment = base_servo_increment;
  static float servo_angle = 90;
  const int max_servo_angle = 117;
  const int min_servo_angle = 20;

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
  display.clear();
  display.drawString(0, 0, "crane");
  display.drawString(0, 10, String(servo_angle,2));
  display.drawString(0, 30, WiFi.localIP().toString());
  display.display();
  servo_angle += servo_increment;
  crane_servo.write(servo_angle);
  // crane_servo.write(servo_angle);
  crane_servo.writeMicroseconds(map(servo_angle*100, 0, 18000, 544, 2400));
  delay(1);
}