package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Interfaces.ClawInterface;
import org.firstinspires.ftc.teamcode.Interfaces.LinkageInterface;
import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import java.util.concurrent.TimeUnit;

/**
 * This thread actuates the slide motors to a given level (in ticks). <br><br>
 *
 * The PID control loop blocks execution of other code, thus it is put in a thread so that we can use the
 * other subsystems while also extending the slides. <br><br>
 *
 * Current slide motors have 145.1 ticks/rotation. (1150 RPM GoBilda motors)
 */

public class SlideThread extends Thread {
    public SlideInterface slideSubsystem;
    LinkageInterface linkageInterface;
    ClawInterface clawInterface;
    public int slideLevel;
    Timing.Timer timer;

    public SlideThread(SlideSubsystem slideSubsystem, LinkageInterface linkageInterface, ClawInterface clawInterface, int slideLevel) {
        this.slideSubsystem = slideSubsystem;
        this.linkageInterface = linkageInterface;
        this.clawInterface = clawInterface;
        this.slideLevel = slideLevel;
    }

    @Override
    public void run() {
        clawInterface.useClaw(Constants.CLOSE_CLAW);

        timer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
        timer.start();
        while(!timer.done()) {

        }
        timer.pause();

        linkageInterface.setExtensionPosition(Constants.INTAKE_SLIDE_INTERMEDIARY_POSITION);
        slideSubsystem.setLevel(slideLevel, false);
    }
}
