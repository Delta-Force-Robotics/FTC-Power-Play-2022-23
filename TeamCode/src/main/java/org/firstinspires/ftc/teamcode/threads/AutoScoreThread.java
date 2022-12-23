package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Interfaces.ClawInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoScoreThread extends Thread{
    public ClawSubsystem clawSubsystem;
    public LinkageSubsystem linkageSubsystem;
    public SlideThread slideThread;
    public TurretTurnThread turretTurnThread;
    public int turnAngle;
    public int slideLevel;
    public Timing.Timer timer;

    public AutoScoreThread(ClawSubsystem clawSubsystem, LinkageSubsystem linkageSubsystem, SlideThread slideThread, TurretTurnThread turretTurnThread) {
        this.clawSubsystem = clawSubsystem;
        this.linkageSubsystem = linkageSubsystem;
        this.slideThread = slideThread;
        this.turretTurnThread = turretTurnThread;
    }

    @Override
    public void run() {
        clawSubsystem.useClaw(Constants.OPEN_CLAW);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        turretTurnThread.turnAngle = turnAngle;
        turretTurnThread.start();

        slideThread.slideLevel = slideLevel;
        slideThread.start();

        while(turretTurnThread.isAlive() || slideThread.isAlive());

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
    }
}
