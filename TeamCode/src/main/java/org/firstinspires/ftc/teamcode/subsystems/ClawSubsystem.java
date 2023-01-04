package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Interfaces.ClawInterface;

public class ClawSubsystem extends SubsystemBase implements ClawInterface {

    private Servo clawServoL;
    private Servo clawServoR;

    public ClawSubsystem(Servo clawServoL, Servo clawServoR) {
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
}
