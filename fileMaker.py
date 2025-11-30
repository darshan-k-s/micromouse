#!/usr/bin/env python3
"""
Arduino INO File Generator
Generates Arduino .ino file with custom command string
Usage: python fileMaker.py "your_command_string" output_file.ino
"""

import sys
import os
from datetime import datetime

def generate_arduino_code(command_string, output_file):
    """Generate Arduino .ino file with custom command string"""
    
    # Validate command string (optional - remove if not needed)
    valid_chars = set('flr')
    if not all(c in valid_chars for c in command_string.lower()):
        print(f"Warning: Command string contains invalid characters. Valid: f, l, r")
        print(f"Command: {command_string}")
    
    # Arduino code template
    arduino_code = f'''#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "IMU.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "LIDAR.hpp"

#include "SharedMemory.hpp"
#include "MovementController.hpp"

// Motors and their encoders
mickeymouse::Motor m1(MOT1PWM,MOT1DIR);
mickeymouse::Encoder encoder1(EN1_A, EN1_B);
mickeymouse::Motor m2(MOT2PWM,MOT2DIR);
mickeymouse::Encoder encoder2(EN2_A, EN2_B);

// IMU
// mickeymouse::IMU imu;
mickeymouse::IMU* imu = new mickeymouse::IMU;

// LiDARs
mickeymouse::LIDAR lidarLeft(LIDAR_LEFT_EN, LIDAR_LEFT_ADDR);
mickeymouse::LIDAR lidarFront(LIDAR_FRONT_EN, LIDAR_FRONT_ADDR);
mickeymouse::LIDAR lidarRight(LIDAR_RIGHT_EN, LIDAR_RIGHT_ADDR);


// // PID controllers for both motors
mickeymouse::PIDController* pidL = new mickeymouse::PIDController(1.0, 0, 0);
mickeymouse::PIDController* pidR = new mickeymouse::PIDController(1.0, 0, 0);


// mickeymouse::PIDController* pidYaw = new mickeymouse::PIDController(1.0, 0.2, 0.3); // Works for chaining
mickeymouse::PIDController* pidYaw = new mickeymouse::PIDController(3.5, 0, 0); // Works amazing for TASK 2 


mickeymouse::PIDController* pidFront = new mickeymouse::PIDController(2, 0, 0); // Only P Zeigler-Nichols - Works for TASK 1 
// mickeymouse::PIDController* pidFront = new mickeymouse::PIDController(1.1, 0.5, 0.08); // Zeigler-Nichols - Not Great


mickeymouse::MovementController moveController;
mickeymouse::PIDController* pidLane = new mickeymouse::PIDController(0.15, 0, 0); 

mickeymouse::PIDController* pidIMU = new mickeymouse::PIDController(0.15, 0, 0); 

unsigned long lastIMUUpdate = 0;
unsigned long lastLIDARUpdate = 0;


// Auto-generated command string - Generated on {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
// Command length: {len(command_string)} characters
String cmd = "{command_string}";

int i = 0;

void setup() {{
  Serial.begin(9600);
  // while(!Serial);
  Wire.begin();
  
  imu->calibrate();

  lidarLeft.beginLidar();
  lidarFront.beginLidar();
  lidarRight.beginLidar();

  Serial.println(F("Initialized"));

}}


// Motor 2 is Faster than 1
void loop() {{
  // //////////////////////////////////////////////////////////////
  // UPDATE SENSOR DATA
  encoder1.getDistance();
  encoder2.getDistance();
  // Read IMU data
  unsigned long now = millis();
  if (now - lastIMUUpdate >= IMU_INTERVAL ) {{
    lastIMUUpdate = now;
    imu->update();
  }}
  // Read LiDAR data
  if (now - lastLIDARUpdate >= LIDAR_SAMPLING_PERIOD) {{
    lastLIDARUpdate = now;
    lidarLeft.getDistance();
    lidarFront.getDistance();
    lidarRight.getDistance();
  }}

///////////////////////////////////////////////////////////////////

  // If in shift movement mode
    if(mickeymouse::getShiftMode()){{
      switch(cmd[i]){{
        case 'f':
          Serial.println(F("FRONT"));
          // Shift mode logic
          mickeymouse::setChainMode(1,0,0);
          ++i;
          mickeymouse::setShiftMode();
          break;

        case 'l':
          Serial.println(F("LEFT"));
          // Shift mode logic
          mickeymouse::setChainMode(0,1,0);
          ++i;
          mickeymouse::setShiftMode();
          break;

        case 'r':
          Serial.println(F("RIGHT"));
          // Shift mode logic
          mickeymouse::setChainMode(0,0,1);
          ++i;
          mickeymouse::setShiftMode();
          break;

        default:
          mickeymouse::setChainMode(0,0,0);
          // Serial.println(cmd);
          break;
      }}
    }}

//////////////////////////////////////////////////////////////////////////

  // Stationary mode
  if (!(mickeymouse::getChainMode().front || mickeymouse::getChainMode().left || mickeymouse::getChainMode().right)){{
    mickeymouse::setWheelPWM(); // Zero
  }}

  // Robot driving front mode
  if(mickeymouse::getChainMode().front){{
    if(moveController.state != 1){{
      uint8_t counter = 1;
      while (cmd[i] == 'f') {{
        ++i;
        counter++;
      }}
      pidL->zeroAndSetTarget(mickeymouse::wheelState.leftWheelLinPos, counter * 180);
      pidR->zeroAndSetTarget(mickeymouse::wheelState.rightWheelLinPos, counter * 180);
      pidLane->zeroAndSetTarget(0, 0);
      pidIMU->zeroAndSetTarget(0, 0);
    }}
    moveController.moveFront(pidL, pidR);
  }}

  // Robot turn left mode
  if(mickeymouse::getChainMode().left){{
    if(moveController.state != 2){{
      pidYaw->zeroAndSetTarget(0, -90);
    }}
    moveController.turnLeft(pidYaw, imu);
  }}

  // Robot turn right mode
  if(mickeymouse::getChainMode().right){{
    if(moveController.state != 3){{
      pidYaw->zeroAndSetTarget(0, 90);
    }}
    moveController.turnRight(pidYaw, imu);
  }}


/////////////////////////////////////////////////////////////////////////
  // Reckonings

  if(mickeymouse::getChainMode().front){{ // Possible changes here
    // moveController.laneCentering(pidLane);
  }}
  
  if(mickeymouse::getChainMode().front){{ 
    moveController.frontWallStopper();
    moveController.headingControl(pidIMU);
  }}


/////////////////////////////////////////////////////////////////////////

// Actuation
  m1.setPWM(mickeymouse::getLeftPWM());
  m2.setPWM(mickeymouse::getRightPWM());


}}
'''
    
    try:
        # Write the Arduino code to file
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(arduino_code)
        
        print(f"Successfully generated Arduino file: {output_file}")
        print(f"Command string: {command_string}")
        print(f"Command length: {len(command_string)} characters")
        print(f"Command breakdown:")
        
        # Analyze command string
        f_count = command_string.count('f')
        l_count = command_string.count('l')
        r_count = command_string.count('r')
        
        print(f"   - Forward (f): {f_count}")
        print(f"   - Left (l): {l_count}")
        print(f"   - Right (r): {r_count}")
        print(f"   - Total moves: {f_count + l_count + r_count}")
        
    except Exception as e:
        print(f"Error writing file: {e}")

def read_command_from_file(filename):
    """Read command string from a text file"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read().strip()
        return content
    except FileNotFoundError:
        print(f"File not found: {filename}")
        return None
    except Exception as e:
        print(f"Error reading file: {e}")
        return None

def main():
    print("🤖 Arduino INO Generator for Mickey Mouse Robot")
    print("=" * 50)
    
    if len(sys.argv) < 2:
        print("Usage Options:")
        print("1. python generate_arduino.py \"command_string\" [output_file.ino]")
        print("2. python generate_arduino.py -f input_file.txt [output_file.ino]")
        print()
        print("Examples:")
        print('   python generate_arduino.py "fffrfrffflflffrfffff" robot.ino')
        print('   python generate_arduino.py -f commands.txt robot.ino')
        print('   python generate_arduino.py "flrflr"  # Uses default output name')
        sys.exit(1)
    
    # Check if reading from file
    if sys.argv[1] == '-f':
        if len(sys.argv) < 3:
            print("Error: Please specify input file after -f")
            sys.exit(1)
        
        input_file = sys.argv[2]
        command_string = read_command_from_file(input_file)
        if command_string is None:
            sys.exit(1)
        
        # Output file
        if len(sys.argv) >= 4:
            output_file = sys.argv[3]
        else:
            base_name = os.path.splitext(input_file)[0]
            output_file = f"{base_name}_robot.ino"
    else:
        # Command string provided directly
        command_string = sys.argv[1]
        
        # Output file
        if len(sys.argv) >= 3:
            output_file = sys.argv[2]
        else:
            output_file = "mickey_mouse_robot.ino"
    
    # Validate output file extension
    if not output_file.endswith('.ino'):
        output_file += '.ino'
    
    # Generate the Arduino code
    generate_arduino_code(command_string, output_file)
    
    # Additional info
    print(f"Ready to upload to Arduino IDE!")
    print(f"Generated file: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    main()