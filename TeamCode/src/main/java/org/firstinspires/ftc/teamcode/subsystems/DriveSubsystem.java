package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private Motor lf;
    private Motor lb;
    private Motor rf;
    private Motor rb;
    private final MecanumDrive mecanumDrive;

    public DriveSubsystem(Motor lf, Motor lb, Motor rf, Motor rb){
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;

        mecanumDrive = new MecanumDrive(lf, rf, lb, rb);
    }
    /**
     * Drives the robot using arcade controls.
     *
     * @param str the commanded strafe movement
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation movement
     */

    public void drive(double str, double fwd, double rot){
        mecanumDrive.driveRobotCentric(-str, -fwd, -rot);
    }
}
