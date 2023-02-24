package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreThread extends Thread {
    public SlideSubsystem slideSubsystem;
    public ScoreSubsystem scoreSubsystem;
    public int levelForSlides = 0;

    public ScoreThread(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        scoreSubsystem.useClaw(Constants.OPEN_CLAW);

        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.useClaw(Constants.CLOSE_CLAW);
        scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_INIT_POSITION);

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.flipClaw(Constants.FLIP_SERVO_INIT_POSITION);
        slideSubsystem.setLevel(levelForSlides);
        scoreSubsystem.useAlign(Constants.ALIGN_SERVO_INIT_POSITION);
        scoreSubsystem.useClaw(Constants.OPEN_CLAW);
    }
}
