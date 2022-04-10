//binary mappings
int bval [4] = {0b100,0b010,0b001,0b101};

void setup() {
  // put your setup code here, to run once:

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
  int r = rgb & 0b100;
  int g = rgb & 0b010;
  int b = rgb & 0b001;

  r = r >= 1 ? LOW : HIGH;
  g = g >= 1 ? LOW : HIGH;
  b = b >= 1 ? LOW : HIGH;

  digitalWrite(2, r); // r
  digitalWrite(4, g); // g
  digitalWrite(5, b); // b


}
