package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;


public class TurretSubsystem extends SubsystemBase implements TurretInterface {
    private Motor turretMotor;
    private BNO055IMU imu;

    public TurretSubsystem(Motor turretMotor, BNO055IMU imu) {
        this.turretMotor = turretMotor;
        this.imu = imu;
    }

    @Override
   public void rotateTurret(int rotateTicks) {
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        turretMotor.setPositionCoefficient(0.05);
        double kP = turretMotor.getPositionCoefficient();

        // the turret can't turn more than 90 degrees to the left or 180 degrees to the right as it is restricted by the cable chain.
        Range.clip(rotateTicks, -378, 189);
        // set the target position
        turretMotor.setTargetPosition(rotateTicks);// an integer representing
            // desired tick count
        turretMotor.set(0);

        // set the tolerance
        turretMotor.setPositionTolerance(13.6);// allowed maximum error

        // perform the control loop
        while (!turretMotor.atTargetPosition()) {
            turretMotor.set(0.50);
        }
        turretMotor.stopMotor(); // stop the motor
    }

    public  void rotateTurretFieldCentric(int rotateTicks) {
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        Orientation angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double ticksOffset = angles.firstAngle/(2*Math.PI)*756; // used for field centric turret.

        // set and get the position coefficient
        turretMotor.setPositionCoefficient(0.05);
        double kP = turretMotor.getPositionCoefficient();

        // the turret can't turn more than 90 degrees to the left or 180 degrees to the right as it is restricted by the cable chain.
        Range.clip(rotateTicks, -378, 189);
        // set the target position
        turretMotor.setTargetPosition(rotateTicks - (int)ticksOffset);// an integer representing
        // desired tick count
        turretMotor.set(0);

        // set the tolerance
        turretMotor.setPositionTolerance(13.6);// allowed maximum error

        // perform the control loop
        while (!turretMotor.atTargetPosition()) {
            turretMotor.set(0.50);
        }
        turretMotor.stopMotor(); // stop the motor
    }
}
