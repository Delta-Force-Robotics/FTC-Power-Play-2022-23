package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class AutoTurretThread extends Thread{
    public DcMotor turretMotor;
    public int turnAngle;

    public AutoTurretThread(DcMotor turretMotor) {
        this.turretMotor = turretMotor;
    }

    @Override
    public void run() {
        PIDFController pidfController = new PIDFController(0, 0, 0, 0);//Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D, 0);
        pidfController.setSetPoint(turnAngle);


        while(!pidfController.atSetPoint() && !isInterrupted()) {
            turretMotor.setPower(
                    pidfController.calculate(
                            turretMotor.getCurrentPosition()
                    )
            );
        }
    }
}
