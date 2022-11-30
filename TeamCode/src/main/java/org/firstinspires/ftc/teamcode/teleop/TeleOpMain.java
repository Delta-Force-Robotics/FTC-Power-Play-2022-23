package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TelemetryDefaultCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.threads.AutoTurretTurnThread;
import org.firstinspires.ftc.teamcode.threads.ScoreSlideThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.threads.SlideThread;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.threads.TurretTurnThread;
import org.openftc.apriltag.AprilTagDetection;

import java.util.function.IntConsumer;

@TeleOp(name="TeleOp Main")
public class TeleOpMain extends CommandOpMode {
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

    private BNO055IMU imuChassis;   // Control Hub IMU

    private GamepadEx driver1;
    private GamepadEx driver2;

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private IntakeCommand intakeCommand;
    private ScoreThread scoreThread;

    private TelemetryDefaultCommand telemetryDefaultCommand;

    private TurretSubsystem turretSubsystem;
    private SlideSubsystem slideSubsystem;
    private LinkageSubsystem linkageSubsystem;
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
    private InstantCommand toggleLinkageExtension;

    private IntConsumer setTurretAngle;

    private AprilTagDetection aprilTagDetection;
    private Timing.Timer timer;

    @Override
    public void initialize() {
        Constants.ROBOT_STOPPED = false;

        leftFront       = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        leftBack        = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rightFront      = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        rightBack       = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);
        turretMotor     = new Motor(hardwareMap, HardwareConstants.ID_TURRET_MOTOR);
        slideMotorLeft  = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
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

        imuChassis = hardwareMap.get(BNO055IMU.class, "imu");
        imuChassis.initialize(parameters);

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

        driveSubsystem      = new DriveSubsystem(leftFront, leftBack, rightFront, rightBack);
        driveCommand        = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX, imuChassis);

        turretSubsystem     = new TurretSubsystem(turretMotor, imuChassis, telemetry);
        slideSubsystem      = new SlideSubsystem(slideMotorLeft, slideMotorRight, telemetry);
        linkageSubsystem    = new LinkageSubsystem(linkageServoL, linkageServoR);
        clawSubsystem       = new ClawSubsystem(clawServoL, clawServoR);

        // Instantiate threads and instant commands.
        autoTurretTurnThread    = new AutoTurretTurnThread(turretSubsystem, imuChassis);
        turretTurnThread        = new TurretTurnThread(turretSubsystem, 0);

        autoTurretTurnThread.setDaemon(true);

        grJunctionThread    = new SlideThread(slideSubsystem, linkageSubsystem, clawSubsystem, Constants.SLIDE_GR_JUNCTION);
        lowJunctionThread   = new SlideThread(slideSubsystem, linkageSubsystem, clawSubsystem, Constants.SLIDE_LOW_JUNCTION);
        midJunctionThread   = new SlideThread(slideSubsystem, linkageSubsystem, clawSubsystem, Constants.SLIDE_MID_JUNCTION);
        highJunctionThread  = new SlideThread(slideSubsystem, linkageSubsystem, clawSubsystem, Constants.SLIDE_HIGH_JUNCTION);
        intakeCommand       = new IntakeCommand(clawSubsystem, new SlideThread(slideSubsystem, linkageSubsystem, clawSubsystem, Constants.SLIDE_INTERMEDIARY));
        scoreThread         = new ScoreThread(clawSubsystem, linkageSubsystem, new ScoreSlideThread(slideSubsystem, Constants.SLIDE_INTAKE), turretTurnThread, turretSubsystem);

        telemetryDefaultCommand = new TelemetryDefaultCommand(turretSubsystem, slideSubsystem, linkageSubsystem, driveSubsystem, clawSubsystem, imuChassis, telemetry);

        intakeSlideInitPosCommand = new InstantCommand(() -> {
            linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);
        }, slideSubsystem);

        intakeSlideExtendCommand = new InstantCommand(() -> {
            linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
        }, slideSubsystem);

        IntConsumer setTurretAngle = (int ticks) -> {
            if(Constants.turretTurnState == Constants.TurretTurnState.CONTINUOUS_FIELD_CENTRIC) {
                autoTurretTurnThread.interrupt();
                autoTurretTurnThread.turnAngle = ticks;
                autoTurretTurnThread.start();
            }
            else {
                turretTurnThread.interrupt();
                turretTurnThread.turnAngle = ticks;
                turretTurnThread.start();
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

        toggleLinkageExtension = new InstantCommand(() -> {
            if(linkageSubsystem.getExtensionPosition() == Constants.INTAKE_SLIDE_INIT_POSITION) {
                linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
            }
            else {
                linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);
            }
        });

        // Assign commands and threads to buttons.

        Button intakeCommandButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeCommand);
        Button scoreCommandButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> scoreThread.start());

        Button slideGrJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(() -> grJunctionThread.start());
        Button slideMidJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(() -> midJunctionThread.start());
        Button slideLowJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(() -> lowJunctionThread.start());
        Button slideHighJunctionButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(() -> highJunctionThread.start());

        Button turretTurn0 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(() -> setTurretAngle.accept(0));
        Button turretTurnPos90 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> setTurretAngle.accept(Constants.TURRET_TURN_90));
        Button turretTurnNeg90 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> setTurretAngle.accept(-Constants.TURRET_TURN_90));
        Button turretTurn180 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> setTurretAngle.accept(Constants.TURRET_TURN_180));

        Button toggleLinkageExtensionButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(toggleLinkageExtension);
        Button toggleTurretStateButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(toggleTurretTurnState);

        register(driveSubsystem, turretSubsystem, linkageSubsystem, clawSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        linkageSubsystem.setDefaultCommand(telemetryDefaultCommand);
    }

    @Override
    public void reset() {
        Constants.ROBOT_STOPPED = true;

        CommandScheduler.getInstance().reset();
    }
}
