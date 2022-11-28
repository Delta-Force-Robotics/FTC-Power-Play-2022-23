package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;
import org.firstinspires.ftc.teamcode.threads.TurretTurnThread;

import java.util.concurrent.TimeUnit;

public class ScoreCommand extends CommandBase {

        ClawSubsystem clawSubsystem;
        IntakeSlideSubsystem intakeSlideSubsystem;
        SlideThread slideThread;
        TurretTurnThread turretTurnThread;
        Timing.Timer timer;

        public ScoreCommand(ClawSubsystem clawSubsystem, IntakeSlideSubsystem intakeSlideSubsystem, SlideThread slideThread, TurretTurnThread turretTurnThread) {
            this.clawSubsystem = clawSubsystem;
            this.intakeSlideSubsystem = intakeSlideSubsystem;
            this.slideThread = slideThread;
            this.turretTurnThread = turretTurnThread;
        }

        @Override
        public void execute() {
            clawSubsystem.useClaw(Constants.OPEN_CLAW);

            timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            timer.start();
            while (!timer.done()) {
                // Sleep
            }
            timer.pause();

            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_INIT_POSITION);
            slideThread.run();

            timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            timer.start();
            while (!timer.done()) {
                // Sleep
            }
            timer.pause();

            if(!turretTurnThread.isAlive()) {
                turretTurnThread.run();
            }
        }

        @Override
        public void end(boolean interrupted) {
            if(interrupted) {
                slideThread.interrupt();
                turretTurnThread.interrupt();
            }
        }
}
