package org.firstinspires.ftc.teamcode.threads;

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
    private BNO055IMU imu;

    public AutoTurretTurnThread(TurretSubsystem turretSubsystem, BNO055IMU imu) {
        this.turretSubsystem = turretSubsystem;
        this.turretSubsystem.isInterrupted = this::isInterrupted;
        this.imu = imu;
    }

    @Override
    public void run() {
        turretSubsystem.turretMotor.setRunMode(Motor.RunMode.PositionControl);
        double ticksOffset = 0.0;

        if (Constants.turretTurnState == Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC) {
            Orientation angles = turretSubsystem.chassisImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = turretSubsystem.calculateOffset(angles); // used for field centric turret.
        }

        turretSubsystem.turretMotor.setPositionCoefficient(0.005);
        turretSubsystem.turretMotor.setTargetPosition((int) turretSubsystem.shortestPathPossible(turnAngle, (int) ticksOffset));
        turretSubsystem.turretMotor.setPositionTolerance(Constants.TURRET_ALLOWED_ERROR); // allowed maximum error

        // perform the control loop
        while ((Constants.turretTurnState == Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC) && !isInterrupted()) {
            Orientation angles = turretSubsystem.chassisImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = turretSubsystem.calculateOffset(angles);

            int targetPosition = (int) turretSubsystem.shortestPathPossible(turnAngle, (int) ticksOffset);
            turretSubsystem.turretMotor.setTargetPosition(targetPosition);
            turretSubsystem.turretMotor.set(1);

        }

        turretSubsystem.turretMotor.stopMotor(); // stop the motor

        Thread.yield();

    }


    }