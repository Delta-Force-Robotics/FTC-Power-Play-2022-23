package org.firstinspires.ftc.teamcode.Threads;

import org.firstinspires.ftc.teamcode.Interfaces.IntakeInterface;
import org.firstinspires.ftc.teamcode.Interfaces.IntakeSlideInterface;
import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;
import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;

/**
 * This thread
 */
public class ScoreThread extends Thread {
    IntakeInterface intakeInterface;
    IntakeSlideInterface intakeSlideInterface;
    SlideInterface slideInterface;
    TurretInterface turretInterface;

    public ScoreThread(IntakeInterface intakeInterface, IntakeSlideInterface intakeSlideInterface, SlideInterface slideInterface, TurretInterface turretInterface) {
        this.intakeInterface = intakeInterface;
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
