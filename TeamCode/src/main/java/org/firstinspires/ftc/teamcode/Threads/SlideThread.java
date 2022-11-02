package org.firstinspires.ftc.teamcode.Threads;

import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;

public class SlideThread extends Thread {
    SlideInterface slideInterface;
    int slideLevel;

    public SlideThread(SlideInterface slideInterface, int slideLevel) {
        this.slideInterface = slideInterface;
        this.slideLevel = slideLevel;
    }

    @Override
    public void run() {
        slideInterface.setLevel(slideLevel);
    }
}
