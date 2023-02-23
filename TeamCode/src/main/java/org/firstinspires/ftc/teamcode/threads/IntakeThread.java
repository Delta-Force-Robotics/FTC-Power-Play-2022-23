package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class IntakeThread extends Thread{
    public SlideSubsystem slideSubsystem;
    public ScoreSubsystem scoreSubsystem;
    public int slideLevel = 0;

    public IntakeThread(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        scoreSubsystem.useClaw(Constants.CLOSE_CLAW);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        slideSubsystem.setLevel(slideLevel);

        if(slideLevel != Constants.SLIDE_GR_JUNCTION) {
            scoreSubsystem.flipClaw(Constants.FLIP_SERVO_FLIP_POSITION);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_PIVOT_POSITION);
            scoreSubsystem.useAlign(Constants.ALIGN_SERVO_ALIGN_POSITION);
        }
        else {
            scoreSubsystem.flipClaw(Constants.FLIP_SERVO_FLIP_GR_POSITION);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_PIVOT_POSITION);
            scoreSubsystem.useAlign(Constants.ALIGN_SERVO_ALIGN_GR_POSITION);
        }
    }
}
