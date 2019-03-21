# Aircon-Controller
Controller for a Mitsubishi air conditioner (Model FDC508HES3)
Provides mqtt control of the airconditioner using the serial communications link that runs
between Mitsubishi's RCD-H-E remote control unit and the CNB port on the air conditioner's control board.
Also provides the ability to control a damper so that airflow can be directed to the correct air conditioning zone.
An inclinometer is provided for the damper in order to detect its position and control the air flow between zones with greater precision.

Requires an mqtt-based controller that measures the temperature readings in each room, compares those temperatures to desired levels and controls the air conditioner and damper to provide the optimum comfort.
That controller is documented here https://github.com/roscoe81/Home-Manager.

## Hardware Schematics
![Aircon Controller Schematic](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/Aircon%20Controller_schem.png)

![Damper Position Sensor Schematic](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/Aircon%20Damper%20Position%20Sensor_schem.png)

![Damper Control Board Schematic](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/Aircon%20Damper%20Control%20Board_schem.png)

## Hardware Photographs
![Aircon Controller](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/IMG_1565.png)

![Damper Position Sensor](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/IMG_1432.png)

**Note that the damper controller switches high voltages and it should only be deployed by a licensed electrician.**

## Air Conditioner Communications Protocol
I've included a [document](https://github.com/roscoe81/Aircon-Controller/blob/master/Serial%20Comms/Aircon%20Controller%20Comms.pdf) that shows what I found when reverse engineering the protocol between the Mitsubishi RCD-H-E remote control unit and the CNB port on the air conditioner's main unit. There are still gaps in some areas of that protocol but all essential functions are covered.

License

This project is licensed under the MIT License - see the LICENSE.md file for details

Acknowledgments

I'd like to acknowledge the work done by [Hadley Rich](https://github.com/hadleyrich/MQMitsi) who did similar work on a Mitsubishi air conditioner that provided me with the inspiration to undertake this task.
