package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;

import java.util.function.Consumer;

@TeleOp
public class TeleOpMain extends CommandOpMode {
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

    private DriveSubsystem driveSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;

    private DriveCommand driveCommand;

    private IntakeThread intakeThread;
    private ScoreThread scoreThread;

    private Consumer<Integer> intakeThreadExecutor;
    private InstantCommand scoreThreadExecutor;
    private GamepadEx driver1;

    @Override
    public void initialize() {
        PhotonCore.enable();
        PhotonCore.experimental.setSinglethreadedOptimized(false);
        PhotonCore.experimental.setMaximumParallelCommands(8);

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

        driveSubsystem = new DriveSubsystem(driveLeftFront, driveLeftBack, driveRightFront, driveRightBack);
        scoreSubsystem = new ScoreSubsystem(clawServo, pivotServoLeft, pivotServoRight, flipServo, alignServo);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, false);

        driver1 = new GamepadEx(gamepad1);
        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX, imu);
        intakeThread = new IntakeThread(slideSubsystem, scoreSubsystem);
        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);

        intakeThreadExecutor = (Integer slideLevel) -> {
            intakeThread.slideLevel = slideLevel;
            intakeThread.interrupt();
            intakeThread.start();
        };

        scoreThreadExecutor = new InstantCommand(() -> {
            scoreThread.interrupt();
            scoreThread.start();
        });

        new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_GR_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_LOW_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_MID_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(() -> intakeThreadExecutor.accept(Constants.SLIDE_HIGH_JUNCTION));
        new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(scoreThreadExecutor);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
