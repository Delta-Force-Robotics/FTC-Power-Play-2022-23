package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
public class LeftMid extends LinearOpMode {
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

    double[] slidePositions = {0.18, 0.145, 0.1, 0.01, 0.0};  //{-450, -305, -215, -157, -10}
    public int slideLevel;

    private SampleMecanumDrive drive;

    private Thread intakeThreadPreload;

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;

    private TrajectorySequence trajPreload;
    private TrajectorySequence trajToIntake;
    private TrajectorySequence trajToScore;
    private  TrajectorySequence trajToIntakeAfterPreload;

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
    private Servo odometryServo;

    private SlideThread slideThread;
    private IntakeThread intakeThread;
    private ScoreThread scoreThread;

    private Consumer<Double> intakeThreadExecutor;
    private Consumer<Double> scoreThreadExecutor;

    private Timing.Timer timer;

    private IMU imu;

    @Override
    public void runOpMode() {
        PhotonCore.experimental.setSinglethreadedOptimized(false);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

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

        odometryServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ODOMETRY_SERVO);
        odometryServo.setPosition(Constants.ODOMETRY_SERVO_INIT_POSITION);

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true);
        scoreSubsystem = new ScoreSubsystem(clawServo, pivotServoLeft, pivotServoRight, flipServo, alignServo, true);
        slideThread = new SlideThread(slideSubsystem);

        intakeThread = new IntakeThread(slideThread, scoreSubsystem, true);
        scoreThread = new ScoreThread(slideThread, scoreSubsystem);

        intakeThreadExecutor = (Double levelForSlides) -> {
            intakeThread.levelForSlides = levelForSlides;
            intakeThread.interrupt();
            intakeThread.start();
        };

        scoreThreadExecutor = (Double levelForSlides) -> {
            scoreThread.levelForSlides = levelForSlides;
            scoreThread.interrupt();
            scoreThread.start();
        };

        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-35.5, -63.5, Math.toRadians(270)));

        trajPreload = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-35, -26, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToSplineHeading(new Pose2d(-28, -7, Math.toRadians(225)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        trajToIntakeAfterPreload = drive.trajectorySequenceBuilder(trajPreload.end())
                .setTangent(Math.toRadians(225))
                .splineToSplineHeading(new Pose2d(-64, -11.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        trajToScore = drive.trajectorySequenceBuilder(new Pose2d(-64, -11.5, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-26, -21, Math.toRadians(135)), Math.toRadians(-30),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        trajToIntake = drive.trajectorySequenceBuilder(trajToScore.end())
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-64, -11.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        parkSpot1 = drive.trajectorySequenceBuilder(trajToScore.end())
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-64, -11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();

        parkSpot2 = drive.trajectorySequenceBuilder(trajToScore.end())
                .lineToLinearHeading(new Pose2d(-36.5, -13, Math.toRadians(0)))
                .build();

        parkSpot3 = drive.trajectorySequenceBuilder(trajToScore.end())
                .lineToLinearHeading(new Pose2d(-35.5, -12, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-10, -12, Math.toRadians(180)))
                .build();

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

        new Thread(() -> camera.closeCameraDevice()).start();

        if(!isStopRequested()){
            intakeRoutine(drive, trajPreload);

            scoreRoutine(drive, trajToIntakeAfterPreload, 0.215);

            for(double slideLevel : slidePositions) {
                intakeRoutineMid(drive, trajToScore);

                scoreRoutine(drive, trajToIntake, slideLevel);
            }

            sleep(30000);
        }
    }

    public void intakeRoutine(SampleMecanumDrive drive, TrajectorySequence traj){
        intakeThreadExecutor.accept(Constants.SLIDE_HIGH_JUNCTION_AUTO);

        while(intakeThread.isAlive() && slideThread.isAlive()){

        }

        drive.followTrajectorySequence(traj);
    }

    public void intakeRoutineMid(SampleMecanumDrive drive, TrajectorySequence traj){

        intakeThreadExecutor.accept(Constants.SLIDE_MID_JUNCTION);

        while(intakeThread.isAlive() && slideThread.isAlive()){

        }

        drive.followTrajectorySequence(traj);

    }

    public void scoreRoutine(SampleMecanumDrive drive, TrajectorySequence traj, Double levelForSlides){
        scoreThreadExecutor.accept(levelForSlides);

        while(scoreThread.isAlive() && slideThread.isAlive()){

        }

        if(levelForSlides == 0.0){

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
        else {
            drive.followTrajectorySequence(traj);
        }
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
