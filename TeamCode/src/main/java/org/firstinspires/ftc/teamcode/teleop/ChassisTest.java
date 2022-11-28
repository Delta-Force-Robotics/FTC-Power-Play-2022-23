package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;


import com.arcrobotics.ftclib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.commands.TelemetryDefaultCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.threads.AutoTurretTurnThread;
import org.firstinspires.ftc.teamcode.threads.SlideThread;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.threads.TurretTurnThread;
import org.openftc.apriltag.AprilTagDetection;

import java.util.function.IntConsumer;

@TeleOp(name="ChassisTest")
public class ChassisTest extends CommandOpMode {
    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;
    private Motor turretMotor;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;

    private Servo linkageServoL;
    private Servo linkageServoR;
    private Servo clawServoL;
    private Servo clawServoR;

    private BNO055IMU imuTurret;    // Expansion Hub IMU
    private BNO055IMU imuChassis;   // Control Hub IMU

    private GamepadEx driver1;
    private GamepadEx driver2;

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private IntakeCommand intakeCommand;
    private ScoreCommand scoreCommand;

    private TelemetryDefaultCommand telemetryDefaultCommand;

    private TurretSubsystem turretSubsystem;
    private SlideSubsystem slideSubsystem;
    private IntakeSlideSubsystem intakeSlideSubsystem;
    private ClawSubsystem clawSubsystem;

    private Thread turretTurn10;
    private SlideThread grJunctionThread;
    private SlideThread midJunctionThread;
    private SlideThread lowJunctionThread;
    private SlideThread highJunctionThread;
    private AutoTurretTurnThread autoTurretTurnThread;
    private TurretTurnThread turretTurnThread;

    private InstantCommand intakeSlideInitPosCommand;
    private InstantCommand intakeSlideExtendCommand;

    private InstantCommand openClaw;
    private InstantCommand closeClaw;
    private InstantCommand toggleTurretTurnState;

    private IntConsumer setTurretAngle;

    private AprilTagDetection aprilTagDetection;

    @Override
    public void initialize() {
        Constants.ROBOT_STOPPED = false;

        leftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        leftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        rightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);
        turretMotor = new Motor(hardwareMap, HardwareConstants.ID_TURRET_MOTOR);
        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        linkageServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_LEFT);
        linkageServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_RIGHT);

        clawServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_LEFT);
        clawServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_RIGHT);

        linkageServoR.setDirection(Servo.Direction.REVERSE);
        clawServoR.setDirection(Servo.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imuTurret = hardwareMap.get(BNO055IMU.class, "imu");
        imuChassis = hardwareMap.get(BNO055IMU.class, "imu2");
        imuChassis.initialize(parameters);
        imuTurret.initialize(parameters);

        linkageServoL.setPosition(0);
        linkageServoR.setPosition(0);

        clawServoL.setPosition(Constants.OPEN_CLAW);
        clawServoR.setPosition(Constants.OPEN_CLAW);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setInverted(true);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        driveSubsystem = new DriveSubsystem(leftFront, leftBack, rightFront, rightBack);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX, imuChassis);

        turretSubsystem = new TurretSubsystem(turretMotor, imuTurret);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, telemetry);
        intakeSlideSubsystem = new IntakeSlideSubsystem(linkageServoL, linkageServoR);
        clawSubsystem = new ClawSubsystem(clawServoL, clawServoR);

        // Instantiate threads and instant commands.
        autoTurretTurnThread = new AutoTurretTurnThread(turretSubsystem, imuTurret);
        turretTurnThread = new TurretTurnThread(turretSubsystem, 0);

        grJunctionThread =      new SlideThread(slideSubsystem, intakeSlideSubsystem, clawSubsystem, Constants.SLIDE_GR_JUNCTION);
        lowJunctionThread =     new SlideThread(slideSubsystem, intakeSlideSubsystem, clawSubsystem, Constants.SLIDE_LOW_JUNCTION);
        midJunctionThread =     new SlideThread(slideSubsystem, intakeSlideSubsystem, clawSubsystem, Constants.SLIDE_MID_JUNCTION);
        highJunctionThread =    new SlideThread(slideSubsystem, intakeSlideSubsystem, clawSubsystem, Constants.SLIDE_HIGH_JUNCTION);

        intakeCommand = new IntakeCommand(clawSubsystem, new SlideThread(slideSubsystem, intakeSlideSubsystem, clawSubsystem, Constants.SLIDE_INTERMEDIARY));
        scoreCommand = new ScoreCommand(clawSubsystem, intakeSlideSubsystem, new SlideThread(slideSubsystem, intakeSlideSubsystem, clawSubsystem, Constants.SLIDE_INTAKE), new TurretTurnThread(turretSubsystem, 0));

        telemetryDefaultCommand = new TelemetryDefaultCommand(turretSubsystem, slideSubsystem, intakeSlideSubsystem, driveSubsystem, clawSubsystem, imuTurret, imuChassis, telemetry);

        intakeSlideInitPosCommand = new InstantCommand(() -> {
            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_INIT_POSITION);
        }, slideSubsystem);

        intakeSlideExtendCommand = new InstantCommand(() -> {
            intakeSlideSubsystem.slideIntake(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
        }, slideSubsystem);

        IntConsumer setTurretAngle = (int ticks) -> {
            if (Constants.turretTurnState == Constants.TurretTurnState.ROBOT_CENTRIC || Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
                turretTurnThread.interrupt();
                turretTurnThread.turnAngle = ticks;
                turretTurnThread.start();
            } else {
                autoTurretTurnThread.turnAngle = ticks;
                if(!autoTurretTurnThread.isAlive()) {
                    autoTurretTurnThread.start();
                }
            }
        };

        toggleTurretTurnState = new InstantCommand(() -> {
            switch(Constants.turretTurnState) {
                case ROBOT_CENTRIC:
                    Constants.turretTurnState = Constants.TurretTurnState.FIELD_CENTRIC;
                    break;
                case FIELD_CENTRIC:
                    Constants.turretTurnState = Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC;
                    break;
                case CONTINUOUS_FIELD_CENTRIC:
                    Constants.turretTurnState = Constants.TurretTurnState.ROBOT_CENTRIC;
                    break;
            }
        });

        // Assign commands and threads to buttons.
        Button startPosButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(intakeSlideInitPosCommand);

        Button intakeCommandButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeCommand);
        Button scoreCommandButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(scoreCommand);

        Button slideGrJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(() -> grJunctionThread.run());
        Button slideMidJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(() -> midJunctionThread.run());
        Button slideLowJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(() -> lowJunctionThread.run());
        Button slideHighJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(() -> highJunctionThread.run());

        Button turretTurn0 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(() -> setTurretAngle.accept(0));
        Button turretTurnPos90 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> setTurretAngle.accept(Constants.TURRET_TURN_90));
        Button turretTurnNeg90 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> setTurretAngle.accept(-Constants.TURRET_TURN_90));
        Button turretTurn180 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> setTurretAngle.accept(Constants.TURRET_TURN_180));

        Button toggleTurretStateButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(toggleTurretTurnState);

        register(driveSubsystem, turretSubsystem, intakeSlideSubsystem, clawSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        intakeSlideSubsystem.setDefaultCommand(telemetryDefaultCommand);
    }

    @Override
    public void reset() {
        Constants.ROBOT_STOPPED = true;

        CommandScheduler.getInstance().reset();
    }
}
