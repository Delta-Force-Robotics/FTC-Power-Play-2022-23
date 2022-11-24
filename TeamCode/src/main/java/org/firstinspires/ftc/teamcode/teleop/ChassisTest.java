package org.firstinspires.ftc.teamcode.teleop;


import android.transition.Slide;

import com.arcrobotics.ftclib.command.CommandOpMode;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.threads.SlideThread;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.openftc.apriltag.AprilTagDetection;

@TeleOp(name="ChassisTest")
public class ChassisTest extends CommandOpMode {
    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;
    private Motor turretMotor;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;

    private Servo slideServoL;
    private Servo slideServoR;
    private Servo clawServoL;
    private Servo clawServoR;

    private BNO055IMU imu;

    private GamepadEx driver1;
    private GamepadEx driver2;

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private IntakeCommand intakeCommand;
    private ScoreCommand scoreCommand;

    private TurretSubsystem turretSubsystem;
    private SlideSubsystem slideSubsystem;
    private IntakeSlideSubsystem intakeSlideSubsystem;
    private ClawSubsystem clawSubsystem;

   private Thread turretTurn10;
   private SlideThread grJunctionThread;
   private SlideThread midJunctionThread;
   private SlideThread lowJunctionThread;
   private SlideThread highJunctionThread;
  /// private ScoreThread scoreThread;

   private InstantCommand startPos;
   private InstantCommand extendedSlide;

   private InstantCommand openClaw;
   private InstantCommand closeClaw;
   private InstantCommand toggleTurretTurnState;

   private AprilTagDetection aprilTagDetection;
    @Override
    public void initialize() {
        leftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        leftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        rightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);
        turretMotor = new Motor(hardwareMap, HardwareConstants.ID_TURRET_MOTOR);
        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        slideServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_LEFT);
        slideServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_RIGHT);

        clawServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_LEFT);
        clawServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_RIGHT);

        slideServoL.setDirection(Servo.Direction.REVERSE);
        clawServoL.setDirection(Servo.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);

        slideServoL.setPosition(0);
        slideServoR.setPosition(0);

        clawServoL.setPosition(0);
        clawServoR.setPosition(0);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        driveSubsystem = new DriveSubsystem(leftFront, leftBack, rightFront, rightBack);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX, imu);

        turretSubsystem = new TurretSubsystem(turretMotor, imu);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight);
        intakeSlideSubsystem = new IntakeSlideSubsystem(slideServoL, slideServoR);
        clawSubsystem = new ClawSubsystem(clawServoL, clawServoR);

        // Instantiate threads and instant commands.
        grJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_GR_JUNCTION);
        lowJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_LOW_JUNCTION);
        midJunctionThread = new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_MID_JUNCTION);
        highJunctionThread = new SlideThread(slideSubsystem::setLevel,  Constants.SLIDE_HIGH_JUNCTION);

        intakeCommand = new IntakeCommand(clawSubsystem, intakeSlideSubsystem, new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_INTERMEDIARY));
        scoreCommand = new ScoreCommand(clawSubsystem, intakeSlideSubsystem, new SlideThread(slideSubsystem::setLevel, Constants.SLIDE_INTAKE));

        startPos = new InstantCommand(() -> {
            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_INIT_POSITION);
        }, slideSubsystem);

        extendedSlide = new InstantCommand(() -> {
            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
        }, slideSubsystem);

        openClaw = new InstantCommand(() -> {
            clawSubsystem.useClaw(Constants.OPEN_CLAW);
        });

        closeClaw = new InstantCommand(() -> {
            clawSubsystem.useClaw(Constants.CLOSE_CLAW);
        });

        toggleTurretTurnState = new InstantCommand(() -> {
            if(Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
                Constants.turretTurnState = Constants.TurretTurnState.ROBOT_CENTRIC;
            } else {
                Constants.turretTurnState = Constants.TurretTurnState.FIELD_CENTRIC;
            }

        });


        // Assign commands and threads to buttons.
        Button turretRotateButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(() -> turretTurn10.start());
        Button slideGrJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> grJunctionThread.run());
        Button slideMidJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> midJunctionThread.run());
        Button slideLowJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(() -> lowJunctionThread.run());
        Button slideHighJunctionButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(() -> highJunctionThread.run());
        Button startPosButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(startPos);
        Button extendedSlideButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(extendedSlide);
        Button openClawButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(openClaw);
        Button closeClawButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(closeClaw);
        Button TurretStateButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(toggleTurretTurnState);
        Button intakeCommandButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeCommand);
        Button scoreCommandButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(scoreCommand);



        register(driveSubsystem, turretSubsystem, intakeSlideSubsystem, clawSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
