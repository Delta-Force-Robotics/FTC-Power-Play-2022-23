package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.RedRight;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoIntakeThread extends Thread{
    public ClawSubsystem clawSubsystem;
    public LinkageSubsystem linkageSubsystem;
    public SlideThread slideThread;
    public TurretTurnThread turretTurnThread;
    public int turnAngle;
    public int slideLevel;
    public Timing.Timer timer;

    public AutoIntakeThread(ClawSubsystem clawSubsystem, LinkageSubsystem linkageSubsystem, SlideThread slideThread, TurretTurnThread turretTurnThread) {
        this.clawSubsystem = clawSubsystem;
        this.linkageSubsystem = linkageSubsystem;
        this.slideThread = slideThread;
        this.turretTurnThread = turretTurnThread;
    }

    @Override
    public void run() {
        slideThread.start();

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

        try {
            Thread.sleep(800);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        turretTurnThread.turnAngle = turnAngle;
        turretTurnThread.start();

        while(turretTurnThread.isAlive());

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
    }
    public void setTurnAngle(int ticks) {
        this.turnAngle = ticks;
    }
}
