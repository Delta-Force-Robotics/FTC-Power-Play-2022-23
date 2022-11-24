package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;

public class ScoreCommand extends CommandBase {

        ClawSubsystem clawSubsystem;
        IntakeSlideSubsystem intakeSlideSubsystem;
        SlideThread slideThread;

        public ScoreCommand(ClawSubsystem clawSubsystem, IntakeSlideSubsystem intakeSlideSubsystem, SlideThread slideThread) {

            this.clawSubsystem = clawSubsystem;
            this.intakeSlideSubsystem = intakeSlideSubsystem;
            this.slideThread = slideThread;
        }
        public void execute() {
            clawSubsystem.useClaw(Constants.OPEN_CLAW);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_INIT_POSITION);
            slideThread.run();
        }
}
