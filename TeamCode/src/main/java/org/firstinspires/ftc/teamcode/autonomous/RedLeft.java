package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.threads.SlideThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.function.Consumer;

@Autonomous
public class RedLeft extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.13;

    int framesWithoutDetection = 0;

    int NUM_FRAMES_BEFORE_DECIMATION = 4;
    int THRESHOLD_DISTANCE_HIGH_DECIMATION = 4;
    int DECIMATION_HIGH = 3;
    int DECIMATION_LOW = 1;

    int tagID;

    AprilTagDetection tagOfInterest = null;

    int[] slidePositions = {0, 0, 0, 0, 0};  //{-450, -305, -215, -157, -10}
    public int slideLevel;

    private SampleMecanumDrive drive;

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;

    private TrajectorySequence trajPreload;
    private TrajectorySequence trajToIntake;
    private TrajectorySequence trajToScore;

    private TrajectorySequence parkSpot1;
    private TrajectorySequence parkSpot2;
    private TrajectorySequence parkSpot3;

    private Motor slideMotorLeft;
    private Motor slideMotorRight;

    private Servo clawServo;
    private Servo pivotServoLeft;
    private Servo pivotServoRight;
    private Servo flipServo;
    private Servo alignServo;

    private SlideThread slideThread;
    private IntakeThread intakeThread;
    private ScoreThread scoreThread;

    private Consumer<Integer> intakeThreadExecutor;
    private InstantCommand scoreThreadExecutor;

    private Timing.Timer timer;

    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        //PhotonCore.experimental.setSinglethreadedOptimized(false);
        //PhotonCore.enable();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        clawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO);
        flipServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        pivotServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_LEFT);
        pivotServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_RIGHT);
        alignServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ALIGN_SERVO);

        slideMotorLeft.setInverted(true);
        pivotServoLeft.setDirection(Servo.Direction.REVERSE);

        slideMotorLeft.resetEncoder();
        slideMotorRight.resetEncoder();

        slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, false);
        scoreSubsystem = new ScoreSubsystem(clawServo, pivotServoLeft, pivotServoRight, flipServo, alignServo);

        intakeThread = new IntakeThread(slideSubsystem, scoreSubsystem);

        intakeThreadExecutor = (Integer slideLevel) -> {
            intakeThread.slideLevel = slideLevel;
            intakeThread.interrupt();
            intakeThread.start();
        };

        scoreThreadExecutor = new InstantCommand(() -> {
            scoreThread.interrupt();
            scoreThread.start();
        });

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        /*traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading()*/

        telemetry.setMsTransmissionInterval(50);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(!currentDetections.isEmpty()) {
                framesWithoutDetection = 0;
                tagOfInterest = currentDetections.get(0);
                tagID = tagOfInterest.id;

                telemetry.addLine("Tag is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }

            sleep(20);
        }

        Thread closeCamera = new Thread(() -> camera.closeCameraDevice());
        closeCamera.start();

        waitForStart();

        if(!isStopRequested()){

            intakeRoutinePreload(drive, trajPreload);
            scoreRoutine(drive, trajToIntake, 0);

            for(int slideLevel : slidePositions) {

                intakeRoutine(drive, trajToScore);

                scoreRoutine(drive, trajToIntake, slideLevel);

            }

            if( tagID == Constants.APRIL_TAG_PARK_ZONE_1 ){
                drive.followTrajectorySequence(parkSpot1);
            }
            else if( tagID == Constants.APRIL_TAG_PARK_ZONE_2){
                drive.followTrajectorySequence(parkSpot2);
            }
            else {
                drive.followTrajectorySequence(parkSpot3);
            }
        }
    }

    public void intakeRoutinePreload(SampleMecanumDrive drive, TrajectorySequence traj){
        intakeThreadExecutor.accept(Constants.SLIDE_HIGH_JUNCTION);

        drive.followTrajectorySequence(traj);

        sleep(200);
    }

    public void intakeRoutine(SampleMecanumDrive drive, TrajectorySequence traj){

        intakeThreadExecutor.accept(Constants.SLIDE_HIGH_JUNCTION);

        drive.followTrajectorySequence(traj);

        sleep(200);
    }

    public void scoreRoutine(SampleMecanumDrive drive, TrajectorySequence traj, int slideLevel){

        scoreSubsystem.useClaw(Constants.OPEN_CLAW);
        scoreSubsystem.useAlign(Constants.ALIGN_SERVO_INIT_POSITION);

        sleep(200);

        scoreSubsystem.pivotClaw(Constants.PIVOT_SERVO_INIT_POSITION);
        scoreSubsystem.flipClaw(Constants.FLIP_SERVO_INIT_POSITION);
        slideSubsystem.setLevel(slideLevel);

        drive.followTrajectorySequence(traj);

        sleep(200);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
