# 2022 Rapid React Robot - Pirate Robotics 

https://www.wcrobotics.org/

## Hardware Overview
- 4 Brushless Motors for drive (1 follower per side)
- 1 Brushless Motor for arm lift (with HW limit switches)
- 2 Brushless Motors for climbing arms
- 1 Brushed Motor for intake

## Software Overview
- Code based on "Tank Drive with CAN" example
- Two Controllers, Driver and Operator

Code Outline:
- Initial imports for libraries
- TimedRobot: All code sits inside this class
    - __robotInit__: Runs once at beginning
    - __autonomousInit__: Runs once at beginning of auto
    - __autonomousPeriodic__: Runs in a loop during auto
    - __teleopInit__: Runs once at beginning of teleop
    - __teleopPeriodic__: Runs in a loop during teleop
    - __testInit__: Runs once at beginning of test
    - __testPeriodic__: Runs in a loop during test