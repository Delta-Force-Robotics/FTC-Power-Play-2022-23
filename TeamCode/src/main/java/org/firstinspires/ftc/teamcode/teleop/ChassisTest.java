package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Threads.SlideIntermediateThread;
import org.firstinspires.ftc.teamcode.Threads.SlideThread;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="ChassisTest")

public class ChassisTest extends CommandOpMode {

    private Motor lf;
    private Motor lb;
    private Motor rf;
    private Motor rb;
    private Motor turretMotor;
    private Motor slideMotor1;
    private Motor slideMotor2;
    private Servo slideServoL;
    private Servo slideServoR;
    private Servo intakeL;
    private Servo intakeR;

    private GamepadEx driver1;
    private GamepadEx driver2;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private TurretSubsystem turretSubsystem;
    private SlideSubsystem slideSubsystem;
    private IntakeSlideSubsystem intakeSlideSubsystem;
    private IntakeSubsystem intakeSubsystem;

    Thread turretTurn10;
    SlideThread grJunctionThread;
    SlideThread midJunctionThread;
    SlideThread lowJunctionThread;
    SlideThread highJunctionThread;
    Thread slideIntermediaryThread;

    InstantCommand startPos;
    InstantCommand extendedSlide;

    InstantCommand openClaw;
    InstantCommand closeClaw;

    @Override
    public void initialize(){
        lf = new Motor(hardwareMap, "lf");
        lb = new Motor(hardwareMap, "lb");
        rf = new Motor(hardwareMap, "rf");
        rb = new Motor(hardwareMap, "rb");
        turretMotor = new Motor(hardwareMap, "turretMotor");
        slideMotor1 = new Motor(hardwareMap, "slideMotor1");
        slideMotor2 = new Motor(hardwareMap, "slideMotor2");

        slideServoL = hardwareMap.get(Servo.class, "linkageServoL");
        slideServoR = hardwareMap.get(Servo.class, "linkageServoR");

        intakeL = hardwareMap.get(Servo.class, "ClawServoL");
        intakeR = hardwareMap.get(Servo.class, "ClawServoR");

        slideServoL.setDirection(Servo.Direction.REVERSE);
        intakeL.setDirection(Servo.Direction.REVERSE);

        slideServoL.setPosition(0);
        slideServoR.setPosition(0);

        intakeL.setPosition(0);
        intakeR.setPosition(0);

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        driveSubsystem= new DriveSubsystem(lf, lb, rf, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        turretSubsystem = new TurretSubsystem(turretMotor);
        slideSubsystem = new SlideSubsystem(slideMotor1, slideMotor2);
        intakeSlideSubsystem = new IntakeSlideSubsystem(slideServoL, slideServoR);
        intakeSubsystem = new IntakeSubsystem(intakeL, intakeR);


         grJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_GR_JUNCTION);
         lowJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_LOW_JUNCTION);
         midJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_MID_JUNCTION);
         highJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_HIGH_JUNCTION);

        slideIntermediaryThread = new SlideIntermediateThread(intakeSubsystem::useClaw, intakeSlideSubsystem::slideIntake, slideSubsystem::setLevel, turretSubsystem::rotateTurret);

        startPos = new InstantCommand(() ->{
            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_INIT_POSITION);
        }, slideSubsystem);

        extendedSlide = new InstantCommand(() ->{
            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
        }, slideSubsystem);

        openClaw = new InstantCommand(() -> {
            intakeSubsystem.useClaw(Constants.OPEN_CLAW);
        });

        closeClaw = new InstantCommand(() -> {
            intakeSubsystem.useClaw(Constants.CLOSE_CLAW);
        });

        Button turretRotateButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(() -> turretTurn10.start());
        Button slideGrJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> grJunctionThread.start());
        Button slideMidJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> midJunctionThread.start());
        Button slideLowJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(() -> lowJunctionThread.start());
        Button slideHighJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(() -> highJunctionThread.start());
        Button startPosButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(startPos);
        Button extendedSlideButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(extendedSlide);
        Button openClawButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(openClaw);
        Button closeClawButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(closeClaw);


        register(driveSubsystem, turretSubsystem, intakeSlideSubsystem, intakeSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
