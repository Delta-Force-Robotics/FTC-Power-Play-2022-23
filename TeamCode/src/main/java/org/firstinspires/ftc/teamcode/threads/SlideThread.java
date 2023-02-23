package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class SlideThread extends Thread {
    public SlideSubsystem slideSubsystem;
    public int slideLevel = 0;

    public SlideThread(SlideSubsystem slideSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        slideSubsystem.setLevel(slideLevel);
    }
}
