//TODO: make the measurmenet an average over a period, not an instantaneous value reading, also note millis overflow issue

//example code below should be plug and play with my esp32 setup, taken from https://www.roboticboat.uk/Microcontrollers/Uno/GarminPaddleWheel/GarminPaddleWheel.html courtesy of Dr. Bowman

//notes:
//brown is data, red is power, green is ground
//see phone pictures for where the wires go

// Timer
unsigned long startTime;
unsigned long endTime;
unsigned long paddleCheckFail;
unsigned long diff;

int val;
int oldval;
int paddleReadCounter;

float water_speed;
bool paddleReadFlag;

// the setup() method runs once, when the sketch starts

void setup() {
  Serial.begin(115200);
}

void loop() {
  //delay(1000); //debug for simulating delays in buoy loop

  // Read the speedometer
  // The speedometer signal oscillates between 0 and 1023 (when at 5v)
  // The speedometer signal oscillates between 0 and 659 (when at 3.3v)
  // So the signal is either 0 or a positive number.
  // Want to detect then the signal falls to zero or rises from zero
  // The speed of the oscillation is the rotation of the wheel

  paddleReadFlag = 0;  //use this read flag to determine if we've read 6 pulses, used to exit while loop if we have a full rotation
  paddleReadCounter = 0; //counter to count up to 12 signals

  startTime = millis();  //logs when in the code we got to this point before we enter the loop, this needs to be right above the loop of taking measurements, no delays between

  while (paddleReadFlag == 0) {
    val = analogRead(12);  //read analog value of pin
    if ((val == 0 && oldval > 0) || (oldval == 0 && val > 0)) {  // The paddle wheel has turned.
      paddleReadCounter++;
    }
    
    // Update the oldval
    oldval = val;

    if (paddleReadCounter == 12) {  //12 readings is what we're looking for, due to each count being EITHER a falling OR rising edge, so 12 counts represents 6 paddle wheels have gone by, also known as a full rotation of the wheel
      endTime = millis();
      diff = endTime - startTime;

      // The wheel has moved 2 cm between the change 0 and 1.
      // Numbers are approximate
      // circumference of wheel = 2 * 3.142 * 1.85 [cm] (2.pi.r) ~ 11.6 cm

      water_speed = (float)11600 / diff; //original guy's math was weird, the circumference of the whole circle is 11.6 cm, THEN you make it 11,600 because milliseconds so it works out to cm/s
      water_speed = water_speed / 51.444; //convert to knots from cm/s
      //water_speed = water_speed / 100; //testing, converts to m/s from cm/s
      
      if (water_speed < 5 && water_speed > 0.1){
      Serial.print(water_speed);
      //Serial.println(" m/s"); //for testing
      Serial.println(" knots");
      Serial.print(diff);
      Serial.print("ms,\t");
      }
      
      paddleReadFlag = 1;//sets the read flag to 1 to exit loop, as we have our measurements
    }

//if (water_speed < 3){
//disp("not valid");
//Serial.println("not valid");
//}


    paddleCheckFail = millis(); //for failure detection, like if the paddlewheel is tangled in seaweed or zebra mussels
    if (paddleCheckFail - startTime > 20000 ) { //timeout after 20 seconds
      paddleReadFlag = 1;
      water_speed = -1; //sets speed negative to indicate a bogus reading, instead of sending previously created reading
      //Serial.println("exiting, buoy stuck"); //debug for checking if stuck
    }   
  } //end paddle while
  //delay(1000); //debug for simulating delays in buoy loop
}