package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.Interfaces.ClawInterface;
import org.firstinspires.ftc.teamcode.Interfaces.IntakeSlideInterface;
import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * This thread actuates the slide motors to a given level (in ticks). <br><br>
 *
 * The PID control loop blocks execution of other code, thus it is put in a thread so that we can use the
 * other subsystems while also extending the slides. <br><br>
 *
 * Current slide motors have 145.1 ticks/rotation. (1150 RPM GoBilda motors)
 */

public class SlideThread extends Thread {
    SlideInterface slideInterface;
    IntakeSlideInterface intakeSlideInterface;
    ClawInterface clawInterface;
    int slideLevel;

    public SlideThread(SlideInterface slideInterface, IntakeSlideInterface intakeSlideInterface, ClawInterface clawInterface, int slideLevel) {
        this.slideInterface = slideInterface;
        this.intakeSlideInterface = intakeSlideInterface;
        this.clawInterface = clawInterface;
        this.slideLevel = slideLevel;
    }

    @Override
    public void run() {
        clawInterface.useClaw(Constants.CLOSE_CLAW);

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        intakeSlideInterface.slideIntake(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
        slideInterface.setLevel(slideLevel);
    }
}
