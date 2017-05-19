import processing.serial.*;
import controlP5.*;

ControlP5 mainController;
PFont avenirFont;
avenirFont = loadFont("Avenir-Light-18.vlw");
textFont(avenirFont);

fill(255);
text("Working Text", 300, 300);

void setup() {
  size(800, 600);  
  noStroke();
  mainController = new ControlP5(this);
  mainController.addButton("temperatureSensors").setPosition(150, 300).setSize(100, 50).setLabel("Temperature Sensors");
  mainController.addButton("lightLevel").setPosition(550, 300).setSize(100, 50).setLabel("Light Level");
  mainController.addButton("oxygenLevel").setPosition(150, 400).setSize(100, 50).setLabel("Oxygen Level");
  mainController.addButton("airQuality").setPosition(550, 400).setSize(100, 50).setLabel("Air Quality");
  mainController.addButton("gasSensors").setPosition(350, 500).setSize(100, 50).setLabel("Gas Sensors");
}

void draw() {
 
}

public void Temperature() {
  background(255);
}