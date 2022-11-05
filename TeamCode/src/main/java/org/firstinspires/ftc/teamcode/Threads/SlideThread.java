package org.firstinspires.ftc.teamcode.Threads;

import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;

/**
 * This thread actuates the slide motors to a given level (in ticks). <br><br>
 *
 * The PID blocks execution of other code, thus it is put in a thread so that we can use the
 * other subsystems while also extending the slides. <br><br>
 *
 * Current slide motors have 145.1 ticks/rotation. (1150 RPM GoBilda motors)
 */

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
