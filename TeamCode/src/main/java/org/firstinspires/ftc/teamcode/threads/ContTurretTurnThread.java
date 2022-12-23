package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ContTurretTurnThread extends Thread{
    public int turnAngle;
    private TurretSubsystem turretSubsystem;
    private PIDFController pidfController;
    private double[] pidfCoefficients;
    private HardwareMap hardwareMap;
    private IMU imu;

    public ContTurretTurnThread(TurretSubsystem turretSubsystem, HardwareMap hardwareMap, IMU imu) {
        this.turretSubsystem = turretSubsystem;
        this.turretSubsystem.isInterrupted = this::isInterrupted;
        this.hardwareMap = hardwareMap;
        this.imu = imu;
    }

    @Override
    public void run() {
        turretSubsystem.turretMotor.setRunMode(Motor.RunMode.RawPower);

        pidfCoefficients = new double[]{Constants.TURRET_PIDF_COEFF.p, Constants.TURRET_PIDF_COEFF.i, Constants.TURRET_PIDF_COEFF.d, 0};
        pidfController = new PIDFController(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double ticksOffset = 0.0;
        if(Constants.turretTurnState != Constants.TurretTurnState.ROBOT_CENTRIC) {
            Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = turretSubsystem.calculateOffset(angles); // used for field centric turret.
        }

        // the turret can't turn more than 90 degrees to the left or 180 degrees to the right as it is restricted by the cable chain.
        pidfController.setSetPoint((int) turretSubsystem.shortestPathPossible(turnAngle, (int)ticksOffset));
        pidfController.setTolerance(Constants.TURRET_ALLOWED_ERROR);

        // perform the control loop
        while ((Constants.turretTurnState == Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC) && !isInterrupted()) {
            Orientation angles = turretSubsystem.chassisImu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = turretSubsystem.calculateOffset(angles);
            int targetPosition = (int) turretSubsystem.shortestPathPossible(turnAngle, (int) ticksOffset);

            turretSubsystem.turretMotor.set(
                    pidfController.calculate(
                            turretSubsystem.turretMotor.getCurrentPosition(),
                            targetPosition
                    ) + Constants.TURRET_PIDF_COEFF.f * 12.0 / batteryVoltage
            );

            Thread.yield();
        }

        turretSubsystem.turretMotor.stopMotor(); // stop the motor


    }
}