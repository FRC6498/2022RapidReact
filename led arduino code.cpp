//code for arduino uno
// pins 2-4 are the rbg pins
// 2 being red and 4 being blue
// pins 5 and 6 are binary color inputs
// can be changed below

//binary mappings
//each index represents a color
int bval [4] = {0b100,0b010,0b001,0b101};

void setup() {
  // put your setup code here, to run once:

  //sets pins 2-4 to rgb output
  pinMode(2,OUTPUT); // r
  pinMode(3,OUTPUT); // g
  pinMode(4,OUTPUT); // b

  //binary input pins
  pinMode(5, INPUT);
  pinMode(6, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  int b1 = digitalRead(5);
  int b2 = digitalRead(6);

  int id = b1*0b10 + b2*0b01;
  rgb(bval[id]);
  

}

void rgb(int rgb){
  //rgb values are binary and digital

  // bit 1 = r
  // bit 2 = g
  // bit 3 = b
  int r = rgb & 0b100;
  int g = rgb & 0b010;
  int b = rgb & 0b001;

  // convert binary values to pin values
  // inverts values because ground = on
  r = r >= 1 ? LOW : HIGH;
  g = g >= 1 ? LOW : HIGH;
  b = b >= 1 ? LOW : HIGH;

  //write the values to the pins
  digitalWrite(2, r); // r
  digitalWrite(3, g); // g
  digitalWrite(4, b); // b


}
