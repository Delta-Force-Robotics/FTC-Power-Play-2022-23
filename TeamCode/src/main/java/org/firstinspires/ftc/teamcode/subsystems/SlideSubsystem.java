package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class SlideSubsystem extends SubsystemBase implements SlideInterface {
    public Motor slideMotorLeft;
    public Motor slideMotorRight;
    private Telemetry telemetry;

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, Telemetry telemetry) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;
        this.telemetry = telemetry;
    }

    /**
     * Sets the extension level for the slides using a PID.
     * @param level Intended level for slide extension.
     */
    @Override
    public void setLevel(int level) {
        slideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
        slideMotorRight.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        slideMotorLeft.setPositionCoefficient(Constants.SLIDE_P);
        slideMotorRight.setPositionCoefficient(Constants.SLIDE_P);

        // set the target position
        slideMotorLeft.setTargetPosition(level);      // an integer representing
        slideMotorRight.setTargetPosition(level);      // desired tick count

        // set the tolerance
        slideMotorLeft.setPositionTolerance(Constants.SLIDE_ALLOWED_ERROR);   // allowed maximum error
        slideMotorRight.setPositionTolerance(Constants.SLIDE_ALLOWED_ERROR);   // allowed maximum error

        // perform the control loop
        while (!slideMotorLeft.atTargetPosition() && !Constants.ROBOT_STOPPED) {
            slideMotorLeft.set(1);
            slideMotorRight.set(1);

            telemetry.addData("2", 2);
            telemetry.addData("Left Motor Ticks", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Right Motor Ticks", slideMotorRight.getCurrentPosition());
            telemetry.update();
        }

        slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        slideMotorRight.setRunMode(Motor.RunMode.RawPower);
        slideMotorLeft.set(-0.15);
        slideMotorRight.set(-0.15);
    }
}
