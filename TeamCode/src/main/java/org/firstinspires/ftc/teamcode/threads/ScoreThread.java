package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.Interfaces.ClawInterface;
import org.firstinspires.ftc.teamcode.Interfaces.IntakeSlideInterface;
import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;
import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;

/**
 * This thread
 */
public class ScoreThread extends Thread {
    ClawInterface clawInterface;
    IntakeSlideInterface intakeSlideInterface;
    SlideInterface slideInterface;
    TurretInterface turretInterface;

    public ScoreThread(ClawInterface clawInterface, IntakeSlideInterface intakeSlideInterface, SlideInterface slideInterface, TurretInterface turretInterface) {
        this.clawInterface = clawInterface;
        this.intakeSlideInterface = intakeSlideInterface;
        this.slideInterface = slideInterface;
        this.turretInterface = turretInterface;
    }

    @Override
    public void run() {
       /* slideInterface.setLevel(Constants.SLIDE_INTERMEDIARY);
        turretInterface.rotateTurret(Constants.TURRET_TURN_30);
        intakeSlideInterface.slideIntake(Constants.INTAKE_SLIDE_EXTEND_10);
        intakeInterface.useClaw(Constants.OPEN_CLAW);*/
    }
}
