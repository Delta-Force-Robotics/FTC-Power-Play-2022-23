package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class MecanumSubsystem extends SubsystemBase {
    private Motor lf;
    private Motor lb;
    private Motor rf;
    private Motor rb;
    private MecanumDrive mecanumDrive;

    public MecanumSubsystem(Motor lf, Motor lb, Motor rf, Motor rb){
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;

        mecanumDrive = new MecanumDrive(lf, rf, lb, rb);
    }

    public void drive(double str, double fwd, double rot){
        mecanumDrive.driveRobotCentric(str, fwd, rot);
    }
}
