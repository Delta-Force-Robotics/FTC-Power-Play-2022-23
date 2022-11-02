package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Interfaces.IntakeSlideInterface;

public class IntakeSlideSubsystem extends SubsystemBase implements IntakeSlideInterface {

    private Servo slideServoL;
    private Servo slideServoR;

    public IntakeSlideSubsystem(Servo slideServoL, Servo slideServoR) {
        this.slideServoL = slideServoL;
        this.slideServoR = slideServoR;
    }

    public void slideIntake(double slideIntakePosition) {
        slideServoL.setPosition(slideIntakePosition);
        slideServoR.setPosition(slideIntakePosition);
    }
}


