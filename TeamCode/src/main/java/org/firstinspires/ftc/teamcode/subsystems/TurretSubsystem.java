package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.function.BooleanSupplier;


public class TurretSubsystem extends SubsystemBase implements TurretInterface {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public IMU chassisImu;
    public Motor turretMotor;
    public BooleanSupplier isInterrupted;
    private PIDFController pidfController;
    private double[] pidfCoefficients;
    public double secondPath, firstPath;
    public double firstPathRelative, secondPathRelative;


    public TurretSubsystem(Motor turretMotor, IMU chassisImu, HardwareMap hardwareMap , Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.chassisImu = chassisImu;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void rotateTurret(int rotateTicks, boolean auto) {
        turretMotor.setRunMode(Motor.RunMode.RawPower);

        pidfCoefficients = new double[]{Constants.TURRET_PIDF_COEFF.p, Constants.TURRET_PIDF_COEFF.i, Constants.TURRET_PIDF_COEFF.d, 0};
        pidfController = new PIDFController(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);

        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double ticksOffset = 0.0;
        if(Constants.turretTurnState != Constants.TurretTurnState.ROBOT_CENTRIC) {
            Orientation angles = chassisImu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = calculateOffset(angles); // used for field centric turret.
            pidfController.setSetPoint((int) shortestPathPossible(rotateTicks, (int) ticksOffset));
        }
        else {
            pidfController.setSetPoint(rotateTicks);
        }

        pidfController.setTolerance(Constants.TURRET_ALLOWED_ERROR, 0.01);

        // perform the control loop
        while (!pidfController.atSetPoint() && !isInterrupted.getAsBoolean()) {
            turretMotor.set(
                    pidfController.calculate(
                            turretMotor.getCurrentPosition()
                    ) + Constants.TURRET_PIDF_COEFF.f * 12.0 / batteryVoltage
            );

            telemetry.addData("turretError" , pidfController.getPositionError());
            telemetry.update();
        }

        telemetry.addData("turretError" , pidfController.getPositionError());
        telemetry.update();

        turretMotor.stopMotor(); // stop the motor
    }

    public int calculateOffset(Orientation angles) {
        return (int)(((-angles.firstAngle) / 360.0) * (double)Constants.TURRET_FULL_ROTATION); // DREAPTA E CU MINUS
    }

    public double shortestPathPossible(int turnTicks, int ticksOffset) {
        double turretTicks = turretMotor.getCurrentPosition();

        double To = turnTicks - ticksOffset;

        firstPath = To;
        firstPathRelative = To - turretTicks;

        secondPath = firstPath - Math.signum(firstPath) * Constants.TURRET_FULL_ROTATION;
        secondPathRelative = firstPathRelative - Math.signum(firstPathRelative) * Constants.TURRET_FULL_ROTATION;

        if (Math.abs(firstPathRelative) < Math.abs(secondPathRelative)) {
            if (firstPath <= Constants.TURRET_CONSTRAINT_MAX && firstPath >= Constants.TURRET_CONSTRAINT_MIN) {
                return firstPath;
            }

            return secondPath;
        } else {
            if (secondPath <= Constants.TURRET_CONSTRAINT_MAX && secondPath >= Constants.TURRET_CONSTRAINT_MIN) {
                return secondPath;
            }

            return firstPath;
        }
    }
}
