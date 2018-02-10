/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;

void messageCb( const geometry_msgs::Vector3& msg){
  if (msg.x > 310 && msg.x < 330){
    //digitalWrite(4, HIGH-digitalRead(4));   // blink the led
    digitalWrite(4, HIGH);   // blink the led
    //nh.loginfo("Callback info");
    delay(100);
  }
}

ros::Subscriber<geometry_msgs::Vector3> sub("face_centroid", &messageCb );

void setup()
{ 
  pinMode(4, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{    
  digitalWrite(4, LOW);   // blink the led
  //nh.loginfo("Program info");
  nh.spinOnce();
  delay(100);
}





