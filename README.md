# **FTC-Power-Play-2022-23**

Our repository for the 2022-23 FIRST Tech Challenge Season "Power Play".

## **Highlights** 

* Field-Relative Mecanum Drive
* 1 + 5 Game Piece Autonomous
* April Tag Detection Pipeline
* PIDF Controller for Slides
* Slides Manual Control

## **Major Package Functions**

### TeleOpMain

* [TeleOpMain](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/TeleOpMain.java)

Contains the general code for the TeleOperated period, meaning all instances, button bindings and InstantCommands. 

### Subsystems

* [Subsystems](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems)
  
Contains code for all subsystems with one single implementation per subsystem, such as Drive Subsystem, Score Subsystem and Slide Subsystem.

Subsystems extend the Subsystem abstract class.

* [DriveSubsystem](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/DriveSubsystem.java)

Contains code for a Mecanum Drive chassis.

* [ScoreSubsystem](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ScoreSubsystem.java)

Contains declarations and functions for all the mechanisms of the Score Subsystem. This Subsystem is composed of a claw servo, flip servo, 2 pivot servos and an align servo. 

* [SlideSubsystem](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/SlideSubsystem.java)

Contains code for the Slide Subsystem with an implemented PIDF Controller.

### Commands

* [Commands](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands)

Contains code for the DriveBase Command and Slides Manual Control.

Commands extend the Command abstract class.

* [DriveCommand](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/DriveCommand.java)

Contains code for the Field Centric Drive.

* [SlideManualControl](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/SlideManualCommand.java)

Contains code for the manual control of the slides.

### Threads

* [Threads](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/threads)

Contains code for all Threads with various action sequences for each subsystem, used for robot control during the TeleOperated period.

Threads extend the Thread abstract class.

### Autonomous

* [Autonomous](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous)

Contains code for each Autonomous sequence used during the competitions.




  

