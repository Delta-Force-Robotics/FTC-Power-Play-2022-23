package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    public Motor LeftFront;
    private Motor leftBack;
    public Motor rightFront;
    private Motor rightBack;
    private final MecanumDrive mecanumDrive;

    public DriveSubsystem(Motor LeftFront, Motor leftBack, Motor rightFront, Motor rightBack) {
        this.LeftFront = LeftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

        mecanumDrive = new MecanumDrive(LeftFront, rightFront, leftBack, rightBack);
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
        mecanumDrive.driveFieldCentric(-str, -fwd, -rot, gyro);
    }
}
