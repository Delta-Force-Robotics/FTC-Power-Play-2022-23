package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretTurnThread extends Thread {

    public TurretSubsystem turretInterface;
    public int turnAngle;
    boolean auto;

    public TurretTurnThread(TurretSubsystem turretInterface, int turnAngle, boolean auto) {
        this.turretInterface = turretInterface;
        this.turretInterface.isInterrupted = this::isInterrupted;
        this.turnAngle = turnAngle;
        this.auto = auto;

    }

    @Override
    public void run() {
        turretInterface.rotateTurret(turnAngle, auto);
    }
}
