package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;

import java.util.concurrent.TimeUnit;

public class IntakeCommand extends CommandBase {

    ClawSubsystem clawSubsystem;
    SlideThread slideThread;
    Timing.Timer timer;

    public IntakeCommand(ClawSubsystem clawSubsystem, SlideThread slideThread) {
        this.clawSubsystem = clawSubsystem;
        this.slideThread = slideThread;
    }

    @Override
    public void execute() {
        clawSubsystem.useClaw(Constants.CLOSE_CLAW);


        timer = new Timing.Timer(250, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            // Sleep
        }
        timer.pause();

        if (!slideThread.isAlive()) {
            slideThread.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            slideThread.interrupt();
        }
    }
}
