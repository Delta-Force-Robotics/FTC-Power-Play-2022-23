package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

public class TeleOp extends CommandOpMode {

    private Motor lf;
    private Motor lb;
    private Motor rf;
    private Motor rb;
    private GamepadEx driver1;
    private MecanumSubsystem mecanumSubsystem;
    private DriveCommand driveCommand;

    @Override
    public void initialize(){
        lf = new Motor(hardwareMap, "lf");
        lb = new Motor(hardwareMap, "lb");
        rf = new Motor(hardwareMap, "rf");
        rb = new Motor(hardwareMap, "rb");
        driver1 = new GamepadEx(gamepad1);

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lf.setInverted(true);
        lb.setInverted(true);

        mecanumSubsystem = new MecanumSubsystem(lf, lb, rf, rb);
        driveCommand = new DriveCommand(mecanumSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        register(mecanumSubsystem);
        mecanumSubsystem.setDefaultCommand(driveCommand);
    }
}
