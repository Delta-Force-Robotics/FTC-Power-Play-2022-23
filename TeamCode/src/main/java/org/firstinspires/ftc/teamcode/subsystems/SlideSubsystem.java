package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;

public class SlideSubsystem extends SubsystemBase implements SlideInterface {
    private Motor slideMotor1;
    private Motor slideMotor2;

    public SlideSubsystem(Motor slideMotor1, Motor slideMotor2){
        this.slideMotor1 = slideMotor1;
        this.slideMotor2 = slideMotor2;
    }

    /**
     * Sets the extension level for the slides using a PID.
     * @param level Intended level for slide extension.
     */
    @Override
    public void setLevel(int level){

        slideMotor1.setRunMode(Motor.RunMode.PositionControl);
        slideMotor2.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        slideMotor1.setPositionCoefficient(0.05);
        slideMotor2.setPositionCoefficient(0.05);
        double kP1 = slideMotor1.getPositionCoefficient();
        double kP2 = slideMotor2.getPositionCoefficient();

        // set the target position
        slideMotor1.setTargetPosition(level);      // an integer representing
        slideMotor2.setTargetPosition(level);      // an integer representing
        // desired tick count

        slideMotor1.set(0);
        slideMotor2.set(0);

        // set the tolerance
        slideMotor1.setPositionTolerance(13.6);   // allowed maximum error
        slideMotor2.setPositionTolerance(13.6);   // allowed maximum error

        // perform the control loop
        while (!slideMotor1.atTargetPosition()) {
            slideMotor1.set(0.50);
            slideMotor2.set(0.50);
        }

        slideMotor1.stopMotor(); // stop the motor
        slideMotor2.stopMotor(); // stop the motor

    }
}
