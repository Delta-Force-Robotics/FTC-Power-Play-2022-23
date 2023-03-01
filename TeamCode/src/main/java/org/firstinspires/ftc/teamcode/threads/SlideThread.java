package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class SlideThread extends Thread {
    public SlideSubsystem slideSubsystem;
    public double slideLevel = 0;

    public SlideThread(SlideSubsystem slideSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        slideSubsystem.setLevel(slideLevel);
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
