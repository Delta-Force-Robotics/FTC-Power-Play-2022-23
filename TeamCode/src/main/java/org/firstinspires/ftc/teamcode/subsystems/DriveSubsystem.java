package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;
    private final MecanumDrive mecanumDrive;

    public DriveSubsystem(Motor leftFront, Motor leftBack, Motor rightFront, Motor rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

        this.leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.leftFront.setRunMode(Motor.RunMode.RawPower);
        this.rightFront.setRunMode(Motor.RunMode.RawPower);
        this.leftBack.setRunMode(Motor.RunMode.RawPower);
        this.rightBack.setRunMode(Motor.RunMode.RawPower);

        mecanumDrive = new MecanumDrive(true, leftFront, rightFront, leftBack, rightBack);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param str the commanded strafe movement
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation movement
     * @param gyro the IMU gyro angle
     */
    public void drive(double str, double fwd, double rot, double gyro) {
        str = (Math.abs(str) >= 0.1) ? str : 0;
        fwd = (Math.abs(fwd) >= 0.1) ? fwd : 0;
        rot = (Math.abs(rot) >= 0.1) ? rot : 0;

        mecanumDrive.driveFieldCentric(str, fwd, -rot, gyro, true);
    }
}
