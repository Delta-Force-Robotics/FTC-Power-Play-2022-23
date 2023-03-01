package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class IntakeThread extends Thread {
    public SlideThread slideThread;
    public ScoreSubsystem scoreSubsystem;
    public double levelForSlides = 0;
    boolean isAuto;

    public IntakeThread(SlideThread slideThread, ScoreSubsystem scoreSubsystem, boolean isAuto) {
        this.slideThread = slideThread;
        this.scoreSubsystem = scoreSubsystem;
        this.isAuto = isAuto;
    }

    public void run() {
        if(isAuto) {
            scoreSubsystem.useClaw(Constants.CLOSE_CLAW_AUTO);
        }
        else {
            scoreSubsystem.useClaw(Constants.CLOSE_CLAW_TELEOP);
        }

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(levelForSlides != Constants.SLIDE_GR_JUNCTION) {
            scoreSubsystem.useAlign(Constants.ALIGN_SERVO_ALIGN_POSITION);

            slideThread.slideLevel = levelForSlides;
            slideThread.interrupt();
            slideThread.start();

            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.flipClaw(Constants.FLIP_SERVO_FLIP_POSITION);

            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_PIVOT_POSITION);
        }

        else {
            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_INIT_POSITION + 0.02);
        }
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
