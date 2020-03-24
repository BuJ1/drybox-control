// For Proper Documentation See the Documentation pdf

// Include Various libraries
#include <Adafruit_GFX.h>       // Include core graphics library
#include <Adafruit_ILI9341.h>   // Include Adafruit_ILI9341 library to drive the display
#include <SPI.h>                // Arduino Interface for Communication


//-----------------------------------------------------
//-------------------- TFT Screen ---------------------
//-----------------------------------------------------
// Declare pins for the display (other pins are preselected)
#define TFT_DC 6
#define TFT_RST -1              // RST Pin
#define TFT_CS 10

// Create display:
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
#include <Fonts/FreeSerif24pt7b.h>  // Add a custom font

// Measure time to only save data after x seconds
static unsigned long currentTime = 0;
const unsigned long timeInterval = 27700; // Save data all 27s so that we can store 2h

// char array to print to the screen
char sensorPrintout[5];
char tempPrintout[5];
bool buttonPressed;
int pageNb = 0;
const int dataStorageSize = 260;
float dataStorage[dataStorageSize] = {0};
bool prev = true;

//-----------------------------------------------------
//--------------- CURRENT LOOP SENSOR -----------------
//-----------------------------------------------------
// To do the programming for the current loop sensor, the guide on 
// https://electronza.com/4-20ma-current-loop-arduino-receiver/ was mainly used
#define CS_CL 9
 
// Calibration data acquired by taking two points (the scale is rather small)
const int ADC_4mA  = 858;
const int ADC_20mA = 999;

//-----------------------------------------------------
//--------------------- Blower ------------------------
//-----------------------------------------------------
int pumpSpeed = 0;
int pumpSpeedMax = 5;
unsigned int maxNb = 160;
int current_speed = 0;
float temperature_old = 0;
float RH_old = 0;
int pumpSpeed_old = 0;

//-----------------------------------------------------
//---------------- Other Hardware ---------------------
//-----------------------------------------------------
// Initialize buttons
#define menuButton 4        // analog 0
#define PumpSpeedIncButton 5
#define PumpSpeedDecButton 14
#define voltageOutput 3
#define Cs2Pin A1

// Initialize tempsensor
#define tempSensor A6 


//-----------------------------------------------------
//------------------ Setup Section --------------------
//-----------------------------------------------------
void setup() {
  analogReference(DEFAULT);
  // initialize the pushbutton pins as digital input:
  pinMode(menuButton, INPUT);
  pinMode(PumpSpeedIncButton, INPUT);
  pinMode(PumpSpeedDecButton, INPUT);
  pinMode(voltageOutput, OUTPUT);
  
  //---------------------------------------------------
  //------------------ TFT Screen ---------------------
  //---------------------------------------------------
  // Put this line at the beginning of every sketch that uses the GLCD:
  // Display setup:
  tft.begin();  // Initialize display
  tft.fillScreen(0x0000);  // Fill screen with black
  tft.setRotation(1);  // Set orientation of the display. Values are from 0 to 3. If not declared, orientation would be 0,
                         // which is portrait mode.
  tft.setTextWrap(false);  // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
                           // To override this behavior (so text will run off the right side of the display - useful for
                           // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
                           // with setTextWrap(true).
                           
  // Base values for the RH and temperature have to be given so that they can be set up
  float temperature = analogRead(tempSensor) * (5.0 / 1024.0) * 100;
  float current = read_current_loop();
  
  // Convert current to relative humidity
  float RH = convert_current_to_RH(current, temperature);
  set_page1_empty(RH, temperature);

  //---------------------------------------------------
  //-------------------- PUMP -------------------------
  //---------------------------------------------------
  pinMode(Cs2Pin, OUTPUT);
  digitalWrite(Cs2Pin, 1);

  //---------------------------------------------------
  //------------ CURRENT LOOP SENSOR ------------------
  //---------------------------------------------------
  // Resetting MCP3201
  // From MCP3201 datasheet: If the device was powered up
  // with the CS pin low, it must be brought high and back low
  // to initiate communication.
  // The device will begin to sample the analog
  // input on the first rising edge after CS goes low.
  pinMode (CS_CL, OUTPUT);
  digitalWrite(CS_CL, 0);
  delay(100);
  digitalWrite(CS_CL, 1);
  
  // initialize serial
  Serial.begin(9600);
  // initialize SPI
  SPI.begin();

}

//------------------------------------------------------
//-------------------- Loop Section --------------------
//------------------------------------------------------

void loop() {
  
  //---------------------------------------------------
  //--------------- CURRENT LOOP SENSOR ---------------
  //---------------------------------------------------
  // Read in the temperature
  float temperature = analogRead(tempSensor) * (5.0 / 1024.0) * 100;
  
  // Read current from current loop sensor
  float current = read_current_loop();
  
  // Convert current to relative humidity
  float RH = convert_current_to_RH(current, temperature);
    
  //---------------------------------------------------
  //------------------ TFT Screen ---------------------
  //---------------------------------------------------
  if (millis() - currentTime >= timeInterval || currentTime == 0){
    // Move every entry in dataStorage one to the left, so that the most recent data is always right
    for(int dataIdx = 0; dataIdx < dataStorageSize - 1; dataIdx++){
      dataStorage[dataIdx] = dataStorage[dataIdx + 1];
    }
  
    // For later plotting store the data in an array
    dataStorage[dataStorageSize-1] = RH;
    currentTime = millis();
  }

  // Enable switching between pages if button is pressed)
  if(digitalRead(menuButton)){
    if(pageNb == 0){
      pageNb = 1;
    }
    else{
      pageNb = 0;
    }
  }
  
  if(pageNb == 0){    
    //-------------- Page 1 (main)  --------------------
    // Set page layout again (only if it changed)
    if(prev == false){
      set_page1_empty(RH, temperature);
    }

    // Check whether pump speed was increased/ decreased
    if(digitalRead(PumpSpeedIncButton) && pumpSpeed < pumpSpeedMax){
      pumpSpeed += 1;
    }    
    if(digitalRead(PumpSpeedDecButton) && pumpSpeed > 0){
      pumpSpeed -= 1;
    }

    // Only update pump speed if it changed
    if(pumpSpeed != pumpSpeed_old){
      // actually adjust pump speed
      adjust_speed(int(float(pumpSpeed) / float(pumpSpeedMax) * float(maxNb)));
      update_speed_bar(pumpSpeed);
    }

    // Only update temperature and RH if it changed
    if(temperature_old != temperature || RH_old != RH){
      update_sensor_readings(RH, temperature);
    }

    // Variables that store old data (to decide if screen should be updated or not)
    temperature_old = temperature;
    RH_old = RH;
    pumpSpeed_old = pumpSpeed;
    
    prev = true;
    
    // wait for a moment
    delay(200);
  }
  else{
    //------------- Page 1 (history)  ------------------
    if(prev == true){
      set_page2();
    }
    prev = false;

    // wait for a moment
    delay(200);
  }
}

//------------------------------------------------------
//---------------- Function Section --------------------
//------------------------------------------------------
float read_current_loop(){
  // Function that reads out the current from the current loop sensor
  
  float loop_current = ReadFrom420mA();
  float current = 0;
  // Error checking
  if (loop_current == -1){
    Serial.println("Error: open loop");
  }
  else if (loop_current == -2){
    Serial.println("Error: current loop is in short circuit");
  // All is OK, remapping to initial data range
  }
  else {
    current = mapfloat(loop_current, ADC_4mA, ADC_20mA, 4, 20);
  }
  
  return current;
}

float Pws(float temp){
  // Function that returns the vapor pressure at a certain temperature

  // Constants
  float Tc = 647.096;
  float Pc = 220640;
  float C1 = -7.85951783;
  float C2 = 1.84408259;
  float C3 = -11.7866497;
  float C4 = 22.6807411;
  float C5 = -15.9618719;
  float C6 = 1.80122502;
  
  float theta = 1 - temp / Tc;
  return exp(Tc / temp * (C1 * theta + C2 * pow(theta, 1.5) + C3 * pow(theta, 3) + C4 * pow(theta, 3.5) + C5 * pow(theta, 4) + C6 * pow(theta, 7.5))) * Pc;
}

float convert_current_to_RH(float current, float temperature){
  // Function that converts the read out current to a relative humidity
  
  // Linear Scale from maximum and minimum values of the dew point sensor
  float dewpoint = (current - 4) / 0.16 - 40 + 273.15;
  
  // Approximate formula from "vaisala": HUMIDITY CONVERSION FORMULAS
  float RH = 100 * Pws(dewpoint) / Pws(temperature + 273.15) ;
  
  return RH;
}

void set_page1_empty(float temp, float RH){
    // Function that sets the first page (with vent speed and temperature/ RH reading)

    // clear the screen with a black background
    tft.fillScreen(0x0000);
    
    // write the static text to the screen
    // set the font color to white
    tft.setCursor(10, 10);  // Set position (x,y)
    tft.setTextColor(0xFFFF);  // Set color of text. First is the color of text and after is color of background
    tft.setTextSize(4);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.println("RH [%]");  // Print a text or value
    
    tft.setCursor(170, 10);  // Set position (x,y)
    tft.println("T [C]");

    tft.drawRect(4, 5, 153, 100, 0xFFFF);
    tft.drawRect(3, 4, 155, 102, 0xFFFF);  // Start from top-left pixel (x,y,width,height)
    tft.drawRect(2, 3, 157, 104, 0xFFFF);

    tft.drawRect(164, 5, 153, 100, 0xFFFF);  // Start from top-left pixel (x,y,width,height)
    tft.drawRect(163, 4, 155, 102, 0xFFFF);
    tft.drawRect(162, 3, 157, 104, 0xFFFF);
    
    update_sensor_readings(temp, RH);

    tft.setTextColor(0xFFFF);
    tft.setTextSize(4);
    
    // Set other white Box containing the Blower Speed
    tft.drawRect(4, 112, 313, 78, 0xFFFF);
    tft.drawRect(3, 111, 315, 80, 0xFFFF);  // Start from top-left pixel (x,y,width,height)
    tft.drawRect(2, 110, 317, 82, 0xFFFF);
    
    tft.setCursor(10, tft.height()/2);  // Set position (x,y)
    tft.println("Blower Speed");  // Print text or value

    // Set white box that indicates pump speed
    int x_left = 9;
    int y_left = tft.height()/2 + 45;
    int width = tft.width() - 17;
    int height = 20;
    tft.drawRect(x_left, y_left, width, height, 0xFFFF);
    tft.drawRect(x_left - 1, y_left - 1, width + 2, height + 2, 0xFFFF);
    tft.drawRect(x_left - 2, y_left - 2, width + 4, height + 4, 0xFFFF);

    // If the pumpSpeed is greater than zero draw a red rectangle to indicate the speed
    tft.fillRect(x_left + 1, y_left + 1, pumpSpeed * (width - 2) / pumpSpeedMax, height - 2, 0xF800);
}

void set_page2(){
  // Function that sets the second page (with history of RH)
  
  // clear the screen with a black background
  tft.fillScreen(0x0000);
  float maxRH = array_max(dataStorage);
  float minRH = array_min(dataStorage);   
  int maxTime = dataStorageSize;
  
  // Draw axis
  int x_left = 50;
  int y_left = 10;
  int width = tft.width() - (x_left + y_left);
  int height = tft.height() - (y_left + x_left);
  int x_zero = x_left;
  int y_zero = y_left + height;
  
  tft.drawRect(x_left, y_left, width, height, 0xFFFF);
  tft.drawRect(x_left - 1, y_left - 1, width + 2, height + 2, 0xFFFF);
  tft.drawRect(x_left - 2, y_left - 2, width + 4, height + 4, 0xFFFF);
  tft.drawFastVLine(int(x_left + width / 2), y_left, height + 5, 0xFFFF);
  tft.drawFastVLine(int(x_left + width / 4), y_left, height + 5, 0xFFFF);
  tft.drawFastVLine(int(x_left + width / 4 * 3), y_left, height + 5, 0xFFFF);
  tft.drawFastHLine(x_left - 5, int(y_left + 10), width + 5, 0xFFFF);
  tft.drawFastHLine(x_left - 5, int(y_left + 10 + (height - 10) / 2), width + 5, 0xFFFF);

  // Draw Axis Labels
  tft.setCursor(7, 3);  // Set position (x,y)
  tft.setTextColor(0xFFFF);  // Set color of text. First is the color of text and after is color of background
  tft.setTextSize(1);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.println("RH [%]");  // Print a text or value
  tft.setCursor(tft.width() - 45, tft.height() - 20);  // Set position (x,y)
  tft.println("t [min]");  // Print a text or value

  // Draw x-axis numbers
  tft.setTextSize(2);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.setCursor(tft.width() - 15, tft.height() - 40);  // Set position (x,y)
  tft.println(int(0));  // Print a text or value
  tft.setCursor(x_left-15, tft.height() - 40);  // Set position (x,y)
  tft.println(int(dataStorageSize * timeInterval / 1000 / 60));  // Print a text or value
  tft.setCursor(int(x_left - 10 + width / 2), tft.height() - 40);  // Set position (x,y)
  tft.println(int(dataStorageSize * timeInterval / 1000 / 2 / 60));  // Print a text or value
  tft.setCursor(int(x_left - 10 + dataStorageSize / 4), tft.height() - 40);  // Set position (x,y)
  tft.println(int(dataStorageSize * timeInterval / 1000 / 4 * 3 / 60));  // Print a text or value
  tft.setCursor(int(x_left - 10 + dataStorageSize / 4 * 3), tft.height() - 40);  // Set position (x,y)
  tft.println(int(dataStorageSize * timeInterval / 1000 / 4 / 60));  // Print a text or value

  // Draw y-axis numbers
  tft.setTextSize(2);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.setCursor(25, tft.height() - 60);  // Set position (x,y)
  tft.println(int(0));  // Print a text or value
  tft.setCursor(8, 15);  // Set position (x,y)
  tft.println(maxRH, 1);  // Print a text or value
  tft.setCursor(8, int(y_left + 5 + (tft.height() - 75) / 2));  // Set position (x,y)
  tft.println(maxRH / 2.0, 1);  // Print a text or value
  
  // Section to draw the actual data
  for(int temp = 0; temp < dataStorageSize - 1; temp++){
    // screen.line(xStart, yStart, xEnd, yEnd); 
    tft.drawLine(x_zero + (temp + 1), 
    y_zero - int(dataStorage[(temp + 1)] / maxRH * (height - 10)),
    x_zero + temp,
    y_zero - int(dataStorage[temp] / maxRH * (height - 10)), 0xFFE0);
  }
}

void adjust_speed(int wanted_speed){
  // Function that adjusts the actual speed of the ventilator according to
  // the user input.
  
  // Decrease speed must go much slower than an increase
  if(wanted_speed < current_speed){
    for(int temp = current_speed; temp >= wanted_speed; temp--){
      analogWrite(voltageOutput, temp);
      delay(100);
    }
  }

  // Increase in speed
  if(wanted_speed > current_speed){
    for(int temp = current_speed; temp <= wanted_speed; temp++){
      analogWrite(voltageOutput, temp);
    }
  }
  current_speed = wanted_speed;
}

void update_speed_bar(int wanted_speed){
  // Function that updates the speed bar on the screen

  // Draw a black rectangle to delete old indication
  int x_left = 9;
  int y_left = tft.height()/2 + 45;
  int width = tft.width() - 17;
  int height = 20;
  tft.fillRect(x_left + 1, y_left + 1, (width - 2), height - 2, 0x0000);

  // Now draw a red rectangle according to the speed 
  tft.fillRect(x_left + 1, y_left + 1, pumpSpeed * (width - 2) / pumpSpeedMax, height - 2, 0xF800);
}

void update_sensor_readings(float RH, float temperature){
  // Update the sensor readings (RH and temperature) according to the
  // measured data
  
  // erase previous text by drawing a black rectangle
  tft.fillRect(10, 50, 140, 50, 0x0000);
  tft.fillRect(170, 50, 140, 50, 0x0000);

  // Read the value of the sensor (humidity anyways)
  String sensorVal = String(RH);
  String tempVal = String(temperature);
  
  // convert the reading to a char array
  sensorVal.toCharArray(sensorPrintout, 5);
  tempVal.toCharArray(tempPrintout, 5);

  // set the font color (yellow)
  // print the sensor value
  tft.setCursor(10, 60);  // Set position (x,y)
  tft.setTextColor(0xFFE0);  // Set color of text. First is the color of text and after is color of background
  tft.setTextSize(5);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.println(sensorPrintout);  // Print a text or value
  tft.setCursor(170, 60);
  tft.println(tempPrintout);  // Print a text or value
}

unsigned int get_ADC(void){
  // DAC works on SPI
  // We receive 16 bits
  // Of which we extract only 12 bits
  // MCP3201 has a strange way of formatting data
  // with 5 bits in the first byte and
  // the rest of 7 bits in the second byte

  unsigned int result;
  unsigned int first_byte;
  unsigned int second_byte;
 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_CL, 0);
  first_byte = SPI.transfer(0);
  second_byte = SPI.transfer(0);
  digitalWrite(CS_CL, 1);
  SPI.endTransaction();
 
  // After the second eight clocks have been
  // sent to the device, the MCU receive register
  // will contain the lowest order seven bits and
  // the B1 bit repeated as the A/D Converter has begun
  // to shift out LSB first data with the extra clock.
  // Typical procedure would then call for the lower order
  // byte of data to be shifted right by one bit
  // to remove the extra B1 bit.
  // See MCP3201 datasheet, page 15
  
  result = ((first_byte & 0x1F) << 8) | second_byte;
  result = result >> 1;
  return result;
}

// Function to read from 4-20mA current loop sensor
int ReadFrom420mA(void)
{
  int result;
  int ADC_result;
  float ADC_avrg = 0;
  for (int i = 0; i < 500; i++){
    ADC_result = get_ADC();
    // Measure every 1ms
    delay(2);
    ADC_avrg = ADC_avrg + ADC_result;
  }
  
  result = (int)(ADC_avrg/500);
 
  // now we do some shortcircuit and open loop checking
  // open loop
  if (result < (ADC_4mA - 50)){
    return -1;
  }
  // shortcircuit_
  if (result > (ADC_20mA + 50)){
    return -2;
  }
  // everything is OK
  return result;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){  
  // Function to map floats (the original function can only map ints)
  
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float array_max(float arr[]){
  // Return maximum value of an array
  
  float max_value = max(dataStorage[0], dataStorage[1]);
  
  for(int temp = 2; temp < dataStorageSize; temp++){
    max_value = max(max_value, dataStorage[temp]);
  }
  return max_value;
}

float array_min(float arr[]){
  // Return minimum value of an array
  
  float min_value = min(dataStorage[0], dataStorage[1]);
  
  for(int temp = 2; temp < dataStorageSize; temp++){
    min_value = min(min_value, dataStorage[temp]);
  }
  return min_value;
}
