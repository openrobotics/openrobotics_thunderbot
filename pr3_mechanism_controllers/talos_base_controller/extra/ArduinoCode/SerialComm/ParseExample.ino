#include <parser.h>

void setup() {
  //open serial connection and create test string
  Serial.begin(9600);
  String test = "11.11, 22.22, 33.33";
  
  //copy String object's contents into char array
  char carr[test.length()];
  test.toCharArray(carr, test.length());
  
  //results container. 
  //float array contains parsed values.
  float parsedvals[3];
  
  //parse character array into results container
  parse_twist((const char*) carr, parsedvals);
  
  //print results
  for (int i = 0; i< 3; i++) {
    Serial.print(String((int)parsedvals[i]) + " :: ");
  }
  
}

void loop() {
}
