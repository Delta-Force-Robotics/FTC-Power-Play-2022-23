package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;


public class TurretSubsystem extends SubsystemBase implements TurretInterface {
    private Motor turretMotor;

    public TurretSubsystem(Motor turretMotor){
        this.turretMotor = turretMotor;
    }

    @Override
   public void rotateTurret(int rotateTicks)
    {
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

// set and get the position coefficient
        turretMotor.setPositionCoefficient(0.05);
        double kP = turretMotor.getPositionCoefficient();

// set the target position
        turretMotor.setTargetPosition(rotateTicks);      // an integer representing
        // desired tick count

        turretMotor.set(0);

// set the tolerance
        turretMotor.setPositionTolerance(13.6);   // allowed maximum error

// perform the control loop
        while (!turretMotor.atTargetPosition()) {
            turretMotor.set(0.50);
        }
        turretMotor.stopMotor(); // stop the motor

    }


}
