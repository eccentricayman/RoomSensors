import processing.serial.*;
import controlP5.*;

ControlP5 mainController;
PFont avenirFont;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port

void setup() {
  size(800, 600);
  
  avenirFont = createFont("Avenir-Light-18.vlw", 22, true);
  textFont(avenirFont);
  fill(0);
  text("Innovation Lab Sensors", 275, 225);
  text("v 1.0", 375, 250);
  
  noStroke();
  //mainController = new ControlP5(this);
  //mainController.addButton("temperatureSensors").setPosition(100, 200).setSize(150, 75).setLabel("Temperature Sensors");
  //mainController.addButton("lightLevel").setPosition(550, 200).setSize(150, 75).setLabel("Light Level");
  //mainController.addButton("oxygenLevel").setPosition(100, 300).setSize(150, 75).setLabel("Oxygen Level");
  //mainController.addButton("airQuality").setPosition(550, 300).setSize(150, 75).setLabel("Air Quality");
  //mainController.addButton("gasSensors").setPosition(325, 300).setSize(150, 75).setLabel("Gas Sensors");

  String portName = Serial.list()[2]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
}

void draw() {
  background(255);
  if ( myPort.available() > 0) {  // If data is available,
    val = myPort.readStringUntil('%');
  } 
  println(val); //print it out in the console
  if (val != null) {
    text("Sensor 1", 350, 100);
    text(val + "Celsius", 350, 200);
    //text(valueArray[1] + "Fahrenheit", 350, 300);
    //text(val, 100, 100);
    print(val);
    fill(0);
  }
}

public void Temperature() {
  background(255);
}