package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretTurnThread extends Thread {

    private TurretSubsystem turretInterface;
    public int turnAngle;

    public TurretTurnThread(TurretSubsystem turretInterface, int turnAngle) {
        this.turretInterface = turretInterface;
        this.turretInterface.isInterrupted = this::isInterrupted;
        this.turnAngle = turnAngle;

    }

    @Override
    public void run() {
        turretInterface.rotateTurret(turnAngle);
    }
}
