<p align="center">

![Figure 1.](/Images/Fig-01-Large.jpg?raw=true)

</p>

# ThermalGrasp

This is a repository for the prototype ThermalGrasp devices created by Alex Mazursky, Jas Brooks, Beza Desta, and Pedro Lopes at the University of Chicago's [Human Computer Integration Lab](https://lab.plopes.org/). Our wearable thermal interfaces made with the ThermalGrasp approach enable users to grab and walk on real objects with minimal obstruction. For specific study details, please refer to our paper (note, it is currently undergoing peer review).

## Repository Structure

* `Arduino` contains code for the microcontroller (Arduino Mega).
* `3D-prints` contains models for the form-fitting shells for the hands and feet. These can be printed in regular PLA using an FDM 3D printer.

## Hardware

Here are the specific hardware components used in each device instantiation and our hardware used in Study 2's backpack.

- Peltiers (foot: TEC12706, hand: CP60231H)
- Copper heat pipes (foot: 70W 11.2 x 3.5 x 100mm, hand: 60W 8.3 x 2.5 x 70mm)
- Copper sheet metal (0.2mm thick)
- Neoprene foam (3.2mm thick, easily compressible)
- Heatsinks (foot: 40 x 40 x 12mm, hand: 20 x 20 x 10mm)
- Fans (foot: 24V, 40 x 40mm, hand: 12V, 20 x 20mm)
- Temperature sensors (100kΩ NTC 3950 thermistors) 
- Motor drivers (VNH3SP30
- Microcontroller (ATmega2560/Arduino Mega)
- MOSFET (RFP30N06LE, triggers fans)

## Contact

For any questions about this repository, please contact alexmazursky@uchicago.edu


