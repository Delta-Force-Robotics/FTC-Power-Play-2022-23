package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Interfaces.IntakeSlideInterface;

public class LinkageSubsystem extends SubsystemBase implements IntakeSlideInterface {

    private Servo slideServoL;
    private Servo slideServoR;

    public LinkageSubsystem(Servo slideServoL, Servo slideServoR) {
        this.slideServoL = slideServoL;
        this.slideServoR = slideServoR;
    }

    /**
     * Sets the servos to the intake position
     * @param slideIntakePosition gets to desired position
     */
    public void setExtensionPosition(double slideIntakePosition) {
        slideServoL.setPosition(slideIntakePosition);
        slideServoR.setPosition(slideIntakePosition);
    }

    public double getExtensionPosition() {
        return slideServoL.getPosition();
    }
}


