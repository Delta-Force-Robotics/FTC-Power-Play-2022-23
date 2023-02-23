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
        mecanumDrive.driveRobotCentric(str, -fwd, rot, true);
    }
}
