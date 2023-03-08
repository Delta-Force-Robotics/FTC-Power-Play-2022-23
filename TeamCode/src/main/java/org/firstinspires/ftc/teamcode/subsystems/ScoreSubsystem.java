package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class ScoreSubsystem extends SubsystemBase{
    private Servo clawServo;
    private Servo pivotServoL;
    private Servo pivotServoR;
    private Servo flipServo;
    private Servo alignServo;

    public ScoreSubsystem(Servo clawServo, Servo pivotServoL, Servo pivotServoR, Servo flipServo, Servo alignServo, boolean isAuto) {
        this.clawServo = clawServo;
        this.pivotServoL = pivotServoL;
        this.pivotServoR = pivotServoR;
        this.flipServo = flipServo;
        this.alignServo = alignServo;

        if(!isAuto) {
            this.clawServo.setPosition(Constants.OPEN_CLAW);
            this.pivotServoL.setPosition(Constants.PIVOT_SERVO_INIT_POSITION);
            this.pivotServoR.setPosition(Constants.PIVOT_SERVO_INIT_POSITION);
            this.flipServo.setPosition(Constants.FLIP_SERVO_INIT_POSITION);
            this.alignServo.setPosition(Constants.ALIGN_SERVO_INIT_POSITION);
        }
        else {
            this.clawServo.setPosition(Constants.CLOSE_CLAW_AUTO);
            this.pivotServoL.setPosition(Constants.PIVOT_SERVO_INIT_AUTO_POSITION);
            this.pivotServoR.setPosition(Constants.PIVOT_SERVO_INIT_AUTO_POSITION);
            this.flipServo.setPosition(Constants.FLIP_SERVO_FLIP_POSITION);
            this.alignServo.setPosition(Constants.ALIGN_SERVO_INIT_POSITION);
        }
    }

    public void useClaw(double clawPosition) { clawServo.setPosition(clawPosition); }

    public void flipClaw(double rotatePosition) { flipServo.setPosition(rotatePosition); }

    public void pivotClaw(double pivotPosition) {
        pivotServoL.setPosition(pivotPosition);
        pivotServoR.setPosition(pivotPosition);
    }

    public void useAlign(double alignPosition) {
        alignServo.setPosition(alignPosition);
    }

    public double getPivotServoLPos() {
        return pivotServoL.getPosition();
    }
}
