package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;

public class IntakeCommand extends CommandBase {

    ClawSubsystem clawSubsystem;
    IntakeSlideSubsystem intakeSlideSubsystem;
    SlideThread slideThread;

    public IntakeCommand(ClawSubsystem clawSubsystem, IntakeSlideSubsystem intakeSlideSubsystem, SlideThread slideThread) {

        this.clawSubsystem = clawSubsystem;
        this.intakeSlideSubsystem = intakeSlideSubsystem;
        this.slideThread = slideThread;
    }

    public void execute() {

        clawSubsystem.useClaw(Constants.CLOSE_CLAW);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        slideThread.run();
        intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
    }
}
