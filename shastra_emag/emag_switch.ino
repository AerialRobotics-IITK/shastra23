#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

uint8_t state;

void subCallback(const std_msgs::UInt8& emag_msg) {
  if (state != emag_msg.data) {
    if (emag_msg.data == 1) {
      digitalWrite(8, LOW);
    } else if (emag_msg.data == 0) {
      digitalWrite(8, HIGH);
    }
    state = emag_msg.data;
  }
}

ros::Subscriber<std_msgs::UInt8> sub("emag_switch", subCallback);

void setup() {
  state = 0;
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
