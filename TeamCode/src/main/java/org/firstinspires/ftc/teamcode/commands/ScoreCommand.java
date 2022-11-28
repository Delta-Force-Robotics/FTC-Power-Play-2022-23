package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;
import org.firstinspires.ftc.teamcode.threads.TurretTurnThread;

public class ScoreCommand extends CommandBase {

        ClawSubsystem clawSubsystem;
        IntakeSlideSubsystem intakeSlideSubsystem;
        SlideThread slideThread;
        TurretTurnThread turretTurnThread;

        public ScoreCommand(ClawSubsystem clawSubsystem, IntakeSlideSubsystem intakeSlideSubsystem, SlideThread slideThread, TurretTurnThread turretTurnThread) {
            this.clawSubsystem = clawSubsystem;
            this.intakeSlideSubsystem = intakeSlideSubsystem;
            this.slideThread = slideThread;
            this.turretTurnThread = turretTurnThread;
        }

        @Override
        public void execute() {
            clawSubsystem.useClaw(Constants.OPEN_CLAW);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_INIT_POSITION);
            slideThread.run();

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

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
