// Speed measurement
// Copyright (C) 2018 https://www.roboticboat.uk
// 399ac0e2-f530-4f0e-a970-d9f1c41b5ec0
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


// Timer
unsigned long mytime;
unsigned long lasttime;
unsigned long diff;

int val;
int oldval;

float speed;

// the setup() method runs once, when the sketch starts

void setup() {

Serial.begin(115200);
  
  // initialize the digital pin as an output.
  pinMode(12, INPUT);

  lasttime = 0;
}

// the loop() methor runs over and over again,
// as long as the board has power


void loop() {

  // Read the speedometer
  // The speedometer signal oscillates between 0 and 1023 (when at 5v)
  // The speedometer signal oscillates between 0 and 659 (when at 3.3v)
  // So the signal is either 0 or a positive number.
  // Want to detect then the signal falls to zero or rises from zero
  // The speed of the oscillation is the rotation of the wheel
  
  val = analogRead(12);
  //Serial.println(val);
  
  // The paddle wheel has turned. 
  if ((val == 0 && oldval>0) || (oldval == 0 && val>0)) {

    // Find change in milliseconds since last
    mytime = millis();
    diff = mytime - lasttime;
    lasttime = mytime;

    // The wheel has moved 2 cm between the change 0 and 1.
    // Numbers are approximate
    // Diameter of wheel = 2 * 3.142 * 0.02 (2.pi.r)
    // 12.5cm / 6 paddles = 2 cm per paddle
    
    Serial.print(diff);
    Serial.print("ms,\t");
    
    speed = (float)2000 / diff;
    Serial.print(speed);
    Serial.println("cm/s");
  }

  // Update the oldval
  oldval = val;
  delay(1000);
  
}