package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TelemetryDefaultCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;
    private SlideSubsystem slideSubsystem;
    private IntakeSlideSubsystem intakeSlideSubsystem;
    private DriveSubsystem driveSubsystem;
    private ClawSubsystem clawSubsystem;
    private BNO055IMU turretIMU;
    private BNO055IMU chassisIMU;
    private Telemetry telemetry;

    public TelemetryDefaultCommand(TurretSubsystem turretSubsystem, SlideSubsystem slideSubsystem, IntakeSlideSubsystem intakeSlideSubsystem, DriveSubsystem driveSubsystem, ClawSubsystem clawSubsystem, BNO055IMU turretIMU, BNO055IMU chassisIMU, Telemetry telemetry) {
        this.turretSubsystem = turretSubsystem;
        this.slideSubsystem = slideSubsystem;
        this.intakeSlideSubsystem = intakeSlideSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.chassisIMU = chassisIMU;
        this.telemetry = telemetry;

        addRequirements(intakeSlideSubsystem);
    }

    @Override
    public void execute() {
        Orientation angles1 = chassisIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Gyroscope 1 Z ", angles1.firstAngle);
        telemetry.addData("Gyroscope 1 Y ", angles1.secondAngle);
        telemetry.addData("Gyroscope 1 X ", angles1.thirdAngle);
        telemetry.addData("Turret Motor", turretSubsystem.turretMotor.getCurrentPosition());
        telemetry.addData("Left Slide Motor", slideSubsystem.slideMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Motor", slideSubsystem.slideMotorRight.getCurrentPosition());
        telemetry.update();
    }
}
