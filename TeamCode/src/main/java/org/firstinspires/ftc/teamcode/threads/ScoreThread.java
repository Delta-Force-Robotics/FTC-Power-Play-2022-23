package org.firstinspires.ftc.teamcode.threads;

import org.apache.commons.math3.util.Precision;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class ScoreThread extends Thread {
    public SlideThread slideThread;
    public ScoreSubsystem scoreSubsystem;
    public double levelForSlides = 0;

    public ScoreThread(SlideThread slideThread, ScoreSubsystem scoreSubsystem) {
        this.slideThread = slideThread;
        this.scoreSubsystem = scoreSubsystem;
    }

    public void run() {
        if (Precision.compareTo(scoreSubsystem.getPivotServoLPos(), Constants.PIVOT_SERVO_INIT_POSITION + 0.02, 0.01) == 0) {
            scoreSubsystem.useClaw(Constants.OPEN_CLAW);


            slideThread.slideLevel = levelForSlides;
            slideThread.interrupt();
            slideThread.start();

            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_INIT_POSITION);
        } else {
            scoreSubsystem.useClaw(Constants.OPEN_CLAW);

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            slideThread.slideLevel = levelForSlides;
            slideThread.interrupt();
            slideThread.start();

            scoreSubsystem.useClaw(Constants.CLOSE_CLAW_AUTO);
            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_INIT_POSITION);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.flipClaw(Constants.FLIP_SERVO_INIT_POSITION);

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.useClaw(Constants.OPEN_CLAW);
            scoreSubsystem.useAlign(Constants.ALIGN_SERVO_INIT_POSITION);
        }
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
