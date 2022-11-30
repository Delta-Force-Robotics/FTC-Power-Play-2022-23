package org.firstinspires.ftc.teamcode.subsystems;


import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;


public class TurretSubsystem extends SubsystemBase implements TurretInterface {
    public Motor turretMotor;
    public BooleanSupplier isInterrupted;
    public BNO055IMU chassisImu;
    public double rightTurn, leftTurn;
    public double leftTurnTurretRelative, rightTurnTurretRelative;
    private PIDFController pidfController;
    private Telemetry telemetry;


    public TurretSubsystem(Motor turretMotor, BNO055IMU chassisImu, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.chassisImu = chassisImu;
        this.telemetry=telemetry;
    }

    @Override
    public void rotateTurret(int rotateTicks) {
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        pidfController = new PIDFController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D, Constants.TURRED_F);

        double ticksOffset = 0.0;

        if(Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
            Orientation angles = chassisImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = calculateOffset(angles); // used for field centric turret.
        }

        // the turret can't turn more than 90 degrees to the left or 180 degrees to the right as it is restricted by the cable chain.
        pidfController.setSetPoint((int) shortestPathPossible(rotateTicks, (int)ticksOffset));
        pidfController.setTolerance(Constants.TURRET_ALLOWED_ERROR);

        // perform the control loop
        while (!pidfController.atSetPoint() && !Constants.ROBOT_STOPPED && !isInterrupted.getAsBoolean()) {
            turretMotor.set(
                    pidfController.calculate(
                            turretMotor.getCurrentPosition()
                    )
            );
        }

        turretMotor.stopMotor(); // stop the motor
    }

    public int calculateOffset(Orientation angles) {
        return (int)(((-angles.firstAngle) / 360.0) * (double)Constants.TURRET_FULL_ROTATION); // DREAPTA E CU MINUS
    }

    public double shortestPathPossible(int turnTicks, int ticksOffset) {
        double turretTicks = turretMotor.getCurrentPosition();

        double To = turnTicks - ticksOffset;

        rightTurn = To - Constants.TURRET_FULL_ROTATION;
        leftTurn = To;

        leftTurnTurretRelative = To - turretTicks;
        rightTurnTurretRelative = leftTurnTurretRelative - Constants.TURRET_FULL_ROTATION;

        if (Math.abs(leftTurnTurretRelative) < Math.abs(rightTurnTurretRelative)) {
            if (leftTurn <= Constants.TURRET_CONSTRAINT_MAX && leftTurn >= Constants.TURRET_CONSTRAINT_MIN) {
                return leftTurn;
            }

            return rightTurn;
        } else {
            if (rightTurn <= Constants.TURRET_CONSTRAINT_MAX && rightTurn >= Constants.TURRET_CONSTRAINT_MIN) {
                return rightTurn;
            }

            return leftTurn;
        }
    }
}
