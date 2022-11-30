package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.concurrent.TimeUnit;

public class ScoreThread extends Thread {
    ClawSubsystem clawSubsystem;
    LinkageSubsystem linkageSubsystem;
    ScoreSlideThread scoreSlideThread;
    TurretTurnThread turretTurnThread;
    Timing.Timer timer;
    TurretSubsystem turretSubsystem;

    public ScoreThread(ClawSubsystem clawSubsystem, LinkageSubsystem linkageSubsystem, ScoreSlideThread scoreSlideThread, TurretTurnThread turretTurnThread, TurretSubsystem turretSubsystem) {
        this.clawSubsystem = clawSubsystem;
        this.linkageSubsystem = linkageSubsystem;
        this.scoreSlideThread = scoreSlideThread;
        this.turretTurnThread = turretTurnThread;
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void run() {
        clawSubsystem.useClaw(Constants.OPEN_CLAW);

        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            // Sleep
        }
        timer.pause();

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

        Constants.turretTurnState=Constants.TurretTurnState.ROBOT_CENTRIC;

        turretTurnThread.turnAngle = 0;
        turretTurnThread.start();


        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            // Sleep
        }
        timer.pause();

        Constants.turretTurnState = Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC;

        scoreSlideThread.start();
    }
}
