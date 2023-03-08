package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.SlideManualCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.threads.SlideThread;

import java.util.function.Consumer;

@TeleOp
public class TeleOpMainReset extends CommandOpMode {
    private IMU imu;
    private Motor driveLeftFront;
    private Motor driveLeftBack;
    private Motor driveRightFront;
    private Motor driveRightBack;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;

    private Servo clawServo;
    private Servo flipServo;
    private Servo pivotServoLeft;
    private Servo pivotServoRight;
    private Servo alignServo;
    private Servo odometryServo;

    private DriveSubsystem driveSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;

    private DriveCommand driveCommand;
    private SlideManualCommand slideManualCommand;

    private IntakeThread intakeThread;
    private ScoreThread scoreThread;
    private SlideThread slideThread;

    private InstantCommand pivotUp;
    private InstantCommand pivotDown;

    private Consumer<Double> intakeThreadExecutor;
    private Consumer<Double> scoreSequenceExecutor;
    private Consumer<Double> slideThreadExecutor;
    private GamepadEx driver1;
    private GamepadEx driver2;


    @Override
    public void initialize() {
        PhotonCore.enable();
        PhotonCore.experimental.setSinglethreadedOptimized(false);
        PhotonCore.experimental.setMaximumParallelCommands(8);

        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;

        driveLeftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        driveLeftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        driveRightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        driveRightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);

        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        pivotServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_LEFT);
        pivotServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_RIGHT);
        clawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO);
        flipServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        alignServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ALIGN_SERVO);

        odometryServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ODOMETRY_SERVO);
        odometryServo.setPosition(Constants.ODOMETRY_SERVO_RETRACTED_POSITION);

        driveSubsystem = new DriveSubsystem(driveLeftFront, driveLeftBack, driveRightFront, driveRightBack);
        scoreSubsystem = new ScoreSubsystem(clawServo, pivotServoLeft, pivotServoRight, flipServo, alignServo, false);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), false, false);
        slideThread = new SlideThread(slideSubsystem);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);
        imu.resetYaw();

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX, imu);
        slideManualCommand = new SlideManualCommand(slideSubsystem, () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        intakeThread = new IntakeThread(slideThread, scoreSubsystem, false);
        scoreThread = new ScoreThread(slideThread, scoreSubsystem);

        intakeThreadExecutor = (Double slideLevel) -> {
            intakeThread.levelForSlides = slideLevel;
            intakeThread.interrupt();
            intakeThread.start();
        };

        scoreSequenceExecutor = (Double levelForSlides) -> {
            scoreThread.levelForSlides = levelForSlides;
            scoreThread.interrupt();
            scoreThread.start();
        };

        slideThreadExecutor = (Double levelForSlides) -> {
            slideThread.slideLevel = levelForSlides;
            slideThread.interrupt();
            slideThread.start();
        };

        pivotUp = new InstantCommand(() -> {
            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_UP_POSSITION);
        });

        pivotDown = new InstantCommand(() -> {
            scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_DOWN_POSSITION);
        });

        new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_GR_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_LOW_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_MID_JUNCTION_TELEOP));
        new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_HIGH_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> scoreSequenceExecutor.accept(Constants.SLIDE_INTAKE));
        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(pivotUp);
        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(pivotDown);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(() -> scoreSequenceExecutor.accept(Constants.SLIDE_POSITIONS_CONESTACK[0]));
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> scoreSequenceExecutor.accept(Constants.SLIDE_POSITIONS_CONESTACK[1]));
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> scoreSequenceExecutor.accept(Constants.SLIDE_POSITIONS_CONESTACK[2]));
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> scoreSequenceExecutor.accept(Constants.SLIDE_POSITIONS_CONESTACK[3]));
        driveSubsystem.setDefaultCommand(driveCommand);
        slideSubsystem.setDefaultCommand(slideManualCommand);
    }
}

/*
        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);
 */