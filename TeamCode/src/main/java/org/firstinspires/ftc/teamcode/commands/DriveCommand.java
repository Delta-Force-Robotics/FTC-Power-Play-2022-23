package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
    * This is our drive command. This works hand in hand with the DriveSubsystem class.
    * Using our DualShock 4 Controllers we control the chassis.
    * The motors get assigned power from -1 to 1 based on how far we push the joysticks.
     **/

public class DriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;
    private final BNO055IMU gyro;

    /**
     * @param driveSubsystem    The drive subsystem this command wil run on.
     * @param forward           The control input for driving forwards/backwards
     * @param strafe            The control input for driving sideways
     * @param rotation          The control input for turning
     * @param gyro              The control hub gyro for field-centric drive
     **/

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation, BNO055IMU gyro) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;
        this.gyro = gyro;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(strafe.getAsDouble(), forward.getAsDouble(), rotation.getAsDouble(), gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }
}
