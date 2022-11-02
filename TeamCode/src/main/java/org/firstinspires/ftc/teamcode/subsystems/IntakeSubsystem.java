package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Interfaces.IntakeInterface;

public class IntakeSubsystem extends SubsystemBase implements IntakeInterface {

    private Servo intakeL;
    private Servo intakeR;

    public IntakeSubsystem(Servo intakeL, Servo intakeR){
        this.intakeL = intakeL;
        this.intakeR = intakeR;
    }

    public void useClaw(double clawPosition){
        intakeL.setPosition(clawPosition);
        intakeR.setPosition(clawPosition);
    }
}
