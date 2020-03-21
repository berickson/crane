#include <Arduino.h>

#include "SSD1306Wire.h"
#include <Servo.h>



// board at https://www.amazon.com/gp/product/B07DKD79Y9
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_scl = 15;
const int pin_oled_rst = 16;
const int pin_crane_servo = 2;

SSD1306Wire display(oled_address, pin_oled_sda, pin_oled_scl);
Servo crane_servo;


void setup() {
  Serial.begin(921600);
  Serial.println("crane started");
  
  // initialize the display
  pinMode(pin_oled_rst, OUTPUT);
  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically();

  // initialize servo
  crane_servo.attach(pin_crane_servo);


  // done
  Serial.println("setup complete");
}

void loop() {
  display.clear();
  display.drawString(0, 0, "crane");
  display.display();
  crane_servo.write(70);
  delay(100);
}