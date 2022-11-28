package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;

public class IntakeCommand extends CommandBase {

    ClawSubsystem clawSubsystem;
    SlideThread slideThread;

    public IntakeCommand(ClawSubsystem clawSubsystem, SlideThread slideThread) {
        this.clawSubsystem = clawSubsystem;
        this.slideThread = slideThread;
    }

    @Override
    public void execute() {
        clawSubsystem.useClaw(Constants.CLOSE_CLAW);

        Timing.Timer timer = new Timing.Timer(500);
        while(timer.elapsedTime() != 500);

        if(!slideThread.isAlive()) {
            slideThread.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            slideThread.interrupt();
        }
    }
}
