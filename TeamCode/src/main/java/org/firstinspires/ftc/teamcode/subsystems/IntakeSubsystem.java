package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Interfaces.ClawInterface;

public class IntakeSubsystem extends SubsystemBase implements ClawInterface {
    private Servo clawServoL;
    private Servo clawServoR;
    private Servo rotateServo;
    private Servo alignServo;

    public IntakeSubsystem(Servo clawServoL, Servo clawServoR) {
        this.clawServoL = clawServoL;
        this.clawServoR = clawServoR;
    }

    /**
     * Open/Close claw
     * @param clawPosition sets the claw to desired position
     */
    public void useClaw(double clawPosition) {
        clawServoL.setPosition(clawPosition);
        clawServoR.setPosition(clawPosition);
    }

    public void rotateClaw(double rotatePosition) {
        rotateServo.setPosition(rotatePosition);
    }

    public void useAlign(double alignPosition) {
        alignServo.setPosition(alignPosition);
    }
}
