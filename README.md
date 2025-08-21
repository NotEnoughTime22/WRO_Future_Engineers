# SSTactical

This repository contains engineering materials of SSTactical's self-driven vehicle model that is competing in the WRO Future Engineers competition in the 2025 season.
## Content
* `Documents` contain the Engineering Journal of the team.
* `Models` contain the model files used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. [If there is nothing to add to this location, the directory can be removed.]
   * `EVO Brick Model.stl` is the 3D model of the case holding our EvolutonX1 Brick.
   * `VL53L0X Lego Mount Model.stl` is the 3D model of a mount for our Time of Flight (ToF). 
* `Photos` contains 2 folders holding photos of the team and the vehicle.
   * `Team` ----- contains one photo of the team (an official one with all team members)
   * `Vehicle` ----- contains 6 photos of the vehicle (from every side, from top and bottom)
      - `Back View.jpeg` contains 1 photo of the back view of our bot
      - `Bottom View.jpeg` contains 1 photo of the bottom view of our bot
      - `Front View.jpeg` contains 1 photo of the front view of our bot
      - `Left View.jpeg` contains 1 photo of the left view of our bot
      - `Right View.jpeg` contains 1 photo of the right view of our bot
      - `Top View.jpeg` contains 1 photo of the top view of our bot
* `Schemes` contains several schematic diagrams in the form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
   * `Microcontroller Wiring Diagram.png` contains a photo of our Microcontroller System Diagram.
   * `Overall System Wiring Diagram.png` contains a photo of our Overall System Wiring Diagram.
* `Software` contains the code of our software for all the components which were programmed to participate in the competition.
  * `Code` contains the code used to program the robot for each challenge.
    - `WRO_Future_Engineers_Colour2` is the code for the round with the blocks (Edit this)
    -  `WRO_Future_Engineers_MK5` is the code for the open challenge (Edit this)
  * `Library` contains each and every library used in the code.
    - `Library/Evo.zip` is our library required to run our microcontroller, with other external libraries in the folder.
    - `Library/External_libraries` is a folder containing all necessary .zip libraries to be added to the Arduino IDE in order for EVERYTHING to function.
## Introduction
# Usage
1.⁠ ⁠Install the libraries found in the Software/Library/External_libraries by going to the library manager in the Arduino IDE and installing each of them through the Add .ZIP Library. -- ALL LIBRARIES ARE REQUIRED FOR THE CODE TO RUN CORRECTLY. 
2.⁠ ⁠Install Software/Library/Evo.zip and follow the following on the Arduino IDE: Sketch --> Include Library --> Add .ZIP Library --> Select EvoEditted.zip. 
3.⁠ ⁠Open the desired file (i.e. mainopen.ino or mainobject.ino) with the Arduino IDE. (EDiT this )

