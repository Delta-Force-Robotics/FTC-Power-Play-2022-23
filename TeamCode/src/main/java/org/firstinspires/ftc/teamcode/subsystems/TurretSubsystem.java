package org.firstinspires.ftc.teamcode.subsystems;


import static java.lang.Double.min;
import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

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

    public TurretSubsystem(Motor turretMotor, BNO055IMU chassisImu) {
        this.turretMotor = turretMotor;
        this.chassisImu = chassisImu;
    }

    @Override
    public void rotateTurret(IntSupplier rotateTicks) {
        turretMotor.setRunMode(Motor.RunMode.PositionControl);
        double ticksOffset = 0.0;

        if(Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
            Orientation angles = chassisImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ticksOffset = calculateOffset(angles); // used for field centric turret.
        }

        turretMotor.setPositionCoefficient(Constants.TURRET_P);

        // the turret can't turn more than 90 degrees to the left or 180 degrees to the right as it is restricted by the cable chain.
        turretMotor.setTargetPosition((int)rotateTurretPresetPosition(rotateTicks.getAsInt(), (int)ticksOffset));

        turretMotor.setPositionTolerance(Constants.TURRET_ALLOWED_ERROR); // allowed maximum error

        // perform the control loop
        while ((!turretMotor.atTargetPosition() || Constants.inputState == Constants.InputState.MANUAL_CONTROL) && !Constants.ROBOT_STOPPED && !isInterrupted.getAsBoolean()) {
            //turretMotor.setTargetPosition(clipTicksToConstraints(rotateTicks.getAsInt() - (int)ticksOffset));
            turretMotor.set(1);
        }
        turretMotor.stopMotor(); // stop the motor
    }

    public int calculateOffset(Orientation angles) {
        return (int)(((angles.firstAngle) / 360.0) * (double)Constants.TURRET_FULL_ROTATION); // DREAPTA E CU MINUS
    }

    public double rotateTurretPresetPosition(int turnTicks, int ticksOffset) {
        double turretTicks = turretMotor.getCurrentPosition();

        double To = turretTicks - ticksOffset;
        double turn  =  0.0;

        double rightTurn = Constants.TURRET_FULL_ROTATION - To - turnTicks;
        double leftTurn = To + turnTicks;

        if(leftTurn < abs(rightTurn)) {
            if(leftTurn > Constants.TURRET_CONSTRAINT_MAX)
                return rightTurn;
            else
                return leftTurn;
        }
        else {
            if(rightTurn < Constants.TURRET_CONSTRAINT_MIN)
                return leftTurn;
            else
                return rightTurn;
        }
    }
}
