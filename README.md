# inf2004-team34-car

# Project Description
INF2004 Team 34 Intelligent Autonomous Robot. We are tasked to make a robotic car that is able to self navigate through a maze.

# Project objective
Make and program a robotic car that can do mapping and navigation through a random maze.

# Overview of the modular design
Block Diagram
![image](https://github.com/xinlonghe2512/inf2004-team34-car/assets/30991653/92f3bdae-cdc0-402a-a51f-b4df0cb0d565)

FlowChart
![image](https://github.com/xinlonghe2512/inf2004-team34-car/assets/30991653/6a434190-0f54-4897-b441-23e798d57dca)

# Requiremed libraries


# How to run the program
In the code, change the wifi SSID and password to your own hotspot and build. Once built drag the uf2 file into the pico. Check the pico's IP address through your hotspot and type it into your browser.

# Task allocation
He Xin Long 
Ultrasonic Sensor
  ○ Task: To avoid and detect obstacles while
  navigating the maze.
  ○ Driver used: 3.3V-5V HC-SR04P Module
  ○ After detecting an obstacle, it will send data to the
  PID Controller to instruct it to orientate the car into
  a different direction.
  
Accelerometer
  o Task: To detect bumps on the maze as the car
  traverse through.
  o Driver used: LSM303DLHC eCompass Module
  o Use I2C or SPI communication, allowing the
  Raspberry Pi Pico to read data from the
  accelerometer components within the sensor.

Kong Teng Foong, Victor
PID Controller Sensor
  • Task: Making car move, steer and regulating car
  speed.
  • Driver used: L298N Motor Driver Module
  • With this driver, we can use it to control the speed
  of the wheel moving by adjusting the voltage with
  the use of PWM to regulate.
  
Efficiency and Speed Optimization
  •  To achieve the best performance and resource
  utilization. With constraints such as limited
  processing power, memory and energy supply, we
  can optimize our codes in order to tackle these
  issues.

Tan Shan Kang IR Wheel Encoder
  • The IR-Based Wheel Encoder will be used to
  determine the rotation of the motor, thus
  determining the speed of the car.

Wyvern Khiang Teng Wei
Magnetometer
  ○ Task: To determine and orientate the car to face
  the correct direction while navigating the maze.
  ○ Driver used: LSM303DLHC eCompass
  Module
  ○ Use I2C or SPI communication based on my own
  research, allowing the Raspberry Pi Pico to read
  data from the magnetometer components within
  the sensor.
Infrared Sensor
  • Task: Implement sensor to do line following, for
  navigation
  • Driver used: IR line tracking module.
  • It comes in a pair of Infrared emitter and receiver
  mounted at the bottom of the module. Whenever
  there is an object blocking the infrared source or
  reflective surface such as white surface, it reflects
  the infrared. The receiver then detects the
  reflected infrared and generates an electric signal
  which can be used to gauge its position related to
  the line. By using this, we will be able to determine
  if the car has to change direction based on the
  electric signal.

Ivan Phua You Wen
  ● Infrared Sensor
    o Task: Implement sensor to scan along the floor for
    barcode on the floor
    o Driver: IR Barcode Module
    o The infrared emitter, positioned at the bottom, will
    scan for black signatures and then transform them
    into ADC readings. We will determine the ADC
    readings threshold and convert the data to
    readable format.
    WiFi-Webserver Interface
    Task: Implement Wi-Fi component to enable data sending
    and interactive user interface.
    - Integrate barcode component with Wi-Fiwebserver on the Pico w. Data from barcode will
    then send the data via message buffer to the web
    server.

All Navigation and Mapping
  • Task: implement navigation algorithms to guide the
  robot car along the track, while avoiding obstacles
  and optimizing for efficiency.
  Documentation
  Testing
  Robot Car Construction
