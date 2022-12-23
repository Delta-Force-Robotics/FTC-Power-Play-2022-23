package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

import java.util.concurrent.TimeUnit;

public class ScoreThread extends Thread {
    public SlideSubsystem slideSubsystem;
    public TurretTurnThread turretTurnThread;
    public ScoreSlideThread scoreSlideThread;
    LinkageSubsystem linkageSubsystem;
    ClawSubsystem clawSubsystem;
    Timing.Timer timer;

    public ScoreThread(ClawSubsystem clawSubsystem, LinkageSubsystem linkageSubsystem, ScoreSlideThread scoreSlideThread, TurretTurnThread turretTurnThread, SlideSubsystem slideSubsystem, Motor slideMotorLeft) {
        this.clawSubsystem = clawSubsystem;
        this.linkageSubsystem = linkageSubsystem;
        this.scoreSlideThread = scoreSlideThread;
        this.turretTurnThread = turretTurnThread;
        this.slideSubsystem = slideSubsystem;
    }

    @Override
    public void run() {
        slideSubsystem.setLevel(slideSubsystem.slideMotorLeft.getCurrentPosition() + 40, false);

        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            // Sleep
        }
        timer.pause();

        clawSubsystem.useClaw(Constants.OPEN_CLAW);

        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            // Sleep
        }
        timer.pause();

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_RETURN_POSITION);

        Constants.turretTurnState = Constants.TurretTurnState.ROBOT_CENTRIC;

        turretTurnThread.turnAngle = 0;
        turretTurnThread.start();

        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            // Sleep
        }
        timer.pause();

        Constants.turretTurnState = Constants.TurretTurnState.FIELD_CENTRIC;

        scoreSlideThread.start();
    }
}
