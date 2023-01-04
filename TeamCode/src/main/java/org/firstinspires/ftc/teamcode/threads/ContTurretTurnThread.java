package org.firstinspires.ftc.teamcode.threads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class ContTurretTurnThread extends Thread{
    public int turnAngle;
    private HardwareMap hardwareMap;
    private TurretSubsystem turretSubsystem;
    private PIDFController pidfController;
    private double[] pidfCoefficients;
    private SimpleMotorFeedforward ffController;
    private DoubleSupplier rightStickTurn;
    private double prevRightStickValue = 0;
    private IMU imu;

    public ContTurretTurnThread(TurretSubsystem turretSubsystem, HardwareMap hardwareMap, IMU imu, DoubleSupplier rightStickTurn, Telemetry telemetry) {
        this.turretSubsystem = turretSubsystem;
        this.turretSubsystem.isInterrupted = this::isInterrupted;
        this.hardwareMap = hardwareMap;
        this.imu = imu;
        this.rightStickTurn = rightStickTurn;
    }

    @Override
    public void run() {
        turretSubsystem.turretMotor.setRunMode(Motor.RunMode.RawPower);

        pidfCoefficients = new double[]{Constants.TURRET_PIDF_COEFF_CONTINUOUS.p, Constants.TURRET_PIDF_COEFF_CONTINUOUS.i, Constants.TURRET_PIDF_COEFF_CONTINUOUS.d, 0};
        pidfController = new PIDFController(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        ffController = new SimpleMotorFeedforward(Constants.TURRET_FF_KS, Constants.TURRET_FF_KV, Constants.TURRET_FF_KA);

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
            ffController = new SimpleMotorFeedforward(Constants.TURRET_FF_KS, Constants.TURRET_FF_KV, Constants.TURRET_FF_KA);

            double ffControllerValue = ffController.calculate(rightStickTurn.getAsDouble()*360, (rightStickTurn.getAsDouble() - prevRightStickValue)*360);
            turretSubsystem.turretMotor.set(
                    pidfController.calculate(
                            turretSubsystem.turretMotor.getCurrentPosition(),
                            targetPosition
                    ) + Constants.TURRET_PIDF_COEFF.f * 12.0 / batteryVoltage
                    + ffControllerValue
            );

            telemetry.addData("Feedforward value", ffControllerValue);
            telemetry.update();
            prevRightStickValue = rightStickTurn.getAsDouble();

            Thread.yield();
        }

        turretSubsystem.turretMotor.stopMotor(); // stop the motor


    }
}