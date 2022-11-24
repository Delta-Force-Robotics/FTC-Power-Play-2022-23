package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class TurretTurnFieldCentricThread extends Thread {

    private TurretInterface turretInterface;
    private int turnAngle;

    public TurretTurnFieldCentricThread(TurretInterface turretInterface, int turnAngle) {
        this.turretInterface = turretInterface;
        this.turnAngle = turnAngle;

    }
    public void run() {

        if(Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
            turretInterface.rotateTurretFieldCentric(turnAngle);
        } else {
            turretInterface.rotateTurretRobotCentric(turnAngle);
        }
    }
}
