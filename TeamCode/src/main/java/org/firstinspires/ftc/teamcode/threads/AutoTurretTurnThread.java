package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class AutoTurretTurnThread extends Thread{
    public int turnAngle;
    private TurretSubsystem turretSubsystem;
    private PIDFController pidfController;
    private BNO055IMU imu;

    public AutoTurretTurnThread(TurretSubsystem turretSubsystem, BNO055IMU imu) {
        this.turretSubsystem = turretSubsystem;
        this.turretSubsystem.isInterrupted = this::isInterrupted;
        this.imu = imu;
    }

    @Override
    public void run() {
        turretSubsystem.turretMotor.setRunMode(Motor.RunMode.RawPower);
        pidfController = new PIDFController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D, Constants.TURRED_F);

        double ticksOffset = 0.0;

        if(Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = turretSubsystem.calculateOffset(angles); // used for field centric turret.
        }

        // the turret can't turn more than 90 degrees to the left or 180 degrees to the right as it is restricted by the cable chain.
        pidfController.setSetPoint((int) turretSubsystem.shortestPathPossible(turnAngle, (int)ticksOffset));
        pidfController.setTolerance(Constants.TURRET_ALLOWED_ERROR);

        // perform the control loop
        while ((Constants.turretTurnState == Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC) && !isInterrupted()) {
            Orientation angles = turretSubsystem.chassisImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = turretSubsystem.calculateOffset(angles);

            int targetPosition = (int) turretSubsystem.shortestPathPossible(turnAngle, (int) ticksOffset);
            pidfController.setSetPoint(targetPosition);

            turretSubsystem.turretMotor.set(
                    pidfController.calculate(
                            turretSubsystem.turretMotor.getCurrentPosition()
                    )
            );

            Thread.yield();
        }

        turretSubsystem.turretMotor.stopMotor(); // stop the motor


    }
}