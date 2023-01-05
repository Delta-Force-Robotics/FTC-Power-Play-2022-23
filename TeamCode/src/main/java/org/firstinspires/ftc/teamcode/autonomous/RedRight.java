/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.LUT;
import com.arcrobotics.ftclib.util.Timing;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MathUtils.MathUtils;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.threads.AutoContTurretThread;
import org.firstinspires.ftc.teamcode.threads.AutoSlideThread;
import org.firstinspires.ftc.teamcode.threads.AutoTurretThread;
import org.firstinspires.ftc.teamcode.threads.TurretTurnThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.function.IntConsumer;

@Autonomous
public class RedRight extends LinearOpMode {
    private IMU imu;

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

    int[] slidePositions = {-145, -110, -65, -35, -0};
    public int turnAngle;
    public int slideLevel;
    public Timing.Timer scoreTimer;

    private Vector2d firstConeStack = new Vector2d(68.5, -11.5);
    private Vector2d secondConeStack = new Vector2d(-68.5,-11.5);

    private Vector2d firstJunction = new Vector2d(23.5, 0);
    private Vector2d midJunction = new Vector2d(0, -23.5);
    private Vector2d secondJunction = new Vector2d(-23.5,0);

    private Motor leftSlideMotor;
    private Motor rightSlideMotor;
    private Motor turretMotor;

    private ServoImplEx leftLinkageServo;
    private ServoImplEx rightLinkageServo;
    private Servo rightClawServo;
    private Servo leftClawServo;

    private SampleMecanumDrive drive;
    private TrajectorySequence traj1;
    private TrajectorySequence traj2;
    private TrajectorySequence parkSpot1;
    private TrajectorySequence parkSpot2;
    private TrajectorySequence parkSpot3;

    private AutoSlideThread autoSlideThread;
    private AutoTurretThread autoTurretThread;
    private TurretTurnThread turretTurnThread;
    private AutoContTurretThread autoContTurretThread;

    private ClawSubsystem clawSubsystem;
    private LinkageSubsystem linkageSubsystem;
    private SlideSubsystem slideSubsystem;
    private TurretSubsystem turretSubsystem;

    public IntConsumer slideLevelThread;
    public IntConsumer scoreThread;
    public IntConsumer intakeThread;

    public TurretTurnThread turretThread;
    public AutoSlideThread slideThread;

    ModernRoboticsI2cRangeSensor rangeSensor;

    private Timing.Timer timer;

    @Override
    public void runOpMode() {
        PhotonCore.experimental.setSinglethreadedOptimized(false);
        PhotonCore.experimental.setMaximumParallelCommands(14);
        PhotonCore.enable();

        double[] realValues = Constants.REAL_VALUE;
        double[] sensorValues = Constants.DISPLAYED_VALUE;
        LUT<Double, Double> correctedSensorValues = new LUT<Double, Double>();

        for(int i = 0; i < realValues.length; i++) {
            correctedSensorValues.add(sensorValues[i], realValues[i]);
        }

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        BigDecimal sensorValue = BigDecimal.valueOf(rangeSensor.getDistance(DistanceUnit.INCH)).setScale(2, RoundingMode.HALF_EVEN);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(70.5 - 8 - correctedSensorValues.getClosest(sensorValue.doubleValue()), -63, Math.toRadians(90)));

        telemetry.addData("sensorValue", 70.5 - 8 - correctedSensorValues.getClosest(sensorValue.doubleValue()));
        telemetry.update();

        Constants.turretTurnState = Constants.TurretTurnState.ROBOT_CENTRIC;

        leftSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        rightSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);
        turretMotor = new Motor(hardwareMap, HardwareConstants.ID_TURRET_MOTOR);

        leftLinkageServo = hardwareMap.get(ServoImplEx.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_LEFT);
        rightLinkageServo = hardwareMap.get(ServoImplEx.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_RIGHT);
        rightClawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_RIGHT);
        leftClawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_LEFT);

        leftSlideMotor.setInverted(true);
        rightClawServo.setDirection(Servo.Direction.REVERSE);
        rightLinkageServo.setDirection(ServoImplEx.Direction.REVERSE);

        leftClawServo.setPosition(Constants.OPEN_CLAW);
        rightClawServo.setPosition(Constants.OPEN_CLAW);

        leftLinkageServo.setPosition(Constants.INTAKE_SLIDE_INIT_POSITION);
        rightLinkageServo.setPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

        turretMotor.resetEncoder();
        leftSlideMotor.resetEncoder();
        rightSlideMotor.resetEncoder();

        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);
        imu.resetYaw();

        clawSubsystem = new ClawSubsystem(leftClawServo, rightClawServo);
        slideSubsystem = new SlideSubsystem(leftSlideMotor, rightSlideMotor, telemetry);
        turretSubsystem = new TurretSubsystem(turretMotor, imu, hardwareMap, telemetry);
        linkageSubsystem = new LinkageSubsystem(leftLinkageServo, rightLinkageServo);

        slideSubsystem.isInterrupted = this::isStopRequested;
        turretSubsystem.isInterrupted = this::isStopRequested;

        autoContTurretThread = new AutoContTurretThread(drive::getPoseEstimate, firstJunction, turretSubsystem, linkageSubsystem, clawSubsystem, new AutoSlideThread(slideSubsystem), telemetry);
        turretTurnThread = new TurretTurnThread(turretSubsystem,turnAngle,true);
        slideThread = new AutoSlideThread(slideSubsystem);

        autoContTurretThread.setDaemon(true);
        autoContTurretThread.setPriority(Thread.MIN_PRIORITY);

        traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(35.5, -35.5, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(new Pose2d(46, -13, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-46, -13),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        parkSpot1 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-10, -13))
                .build();

        parkSpot2 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-35, -13))
                .build();

        parkSpot3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-59, -13))
                .build();

        telemetry.addData("imu angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        telemetry.setMsTransmissionInterval(100);
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

            sleep(100);
        }

        FtcDashboard ftcDashboard = FtcDashboard.getInstance();
        telemetry = ftcDashboard.getTelemetry();

        Thread closeCamera = new Thread(() -> {
            camera.closeCameraDevice();
            camera.pauseViewport();
        });
        closeCamera.start();

        waitForStart();

        if(!isStopRequested()){
            clawSubsystem.useClaw(Constants.CLOSE_CLAW);
            sleep(200);
            slideThread.slideLevel = Constants.SLIDE_HIGH_JUNCTION - 20;
            slideThread.start();

            if( tagID == Constants.APRIL_TAG_PARK_ZONE_1 ){
                caseA(drive);
            }
            else if( tagID == Constants.APRIL_TAG_PARK_ZONE_2){
                caseB(drive);
            }
            else {
                caseC(drive);
            }

            linkageSubsystem.disablePwm();
        }
    }

    public void caseA(SampleMecanumDrive drive){
        drive.followTrajectorySequence(traj1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        double robotAngle = Math.toDegrees(poseEstimate.getHeading());
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;

        double angle = robotAngle - Math.toDegrees(Math.atan2(firstJunction.getY() - poseEstimate.getY(), firstJunction.getX() - poseEstimate.getX()));
        int ticks = (int)(angle / 360.0 * (double)Constants.TURRET_FULL_ROTATION);

        double angle2 = robotAngle - Math.toDegrees(Math.atan2(firstConeStack.getY() - poseEstimate.getY(), firstConeStack.getX() - poseEstimate.getX()));
        int ticks2 = (int) (angle2 / 360.0 * (double)Constants.TURRET_FULL_ROTATION);

        telemetry.addData("angle", angle);
        telemetry.addData("angle2", angle2);
        telemetry.addData("ticks", ticks);
        telemetry.addData("ticks2", ticks2);
        telemetry.addData("robotAngle", robotAngle);
        telemetry.addData("imuAngle", imuAngle);


        drive.followTrajectorySequence(parkSpot1);
    }

    public void caseB(SampleMecanumDrive drive){
        drive.followTrajectorySequence(traj1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        double robotAngle = Math.toDegrees(poseEstimate.getHeading());
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;

        double angle = robotAngle - Math.toDegrees(Math.atan2(firstJunction.getY() - poseEstimate.getY(), firstJunction.getX() - poseEstimate.getX()));
        int ticks = (int)(angle / 360.0 * (double)Constants.TURRET_FULL_ROTATION);

        double angle2 = robotAngle - Math.toDegrees(Math.atan2(firstConeStack.getY() - poseEstimate.getY(), firstConeStack.getX() - poseEstimate.getX()));
        int ticks2 = (int) (angle2 / 360.0 * (double)Constants.TURRET_FULL_ROTATION);

        telemetry.addData("angle", angle);
        telemetry.addData("angle2", angle2);
        telemetry.addData("ticks", ticks);
        telemetry.addData("ticks2", ticks2);
        telemetry.addData("robotAngle", robotAngle);
        telemetry.addData("imuAngle", imuAngle);


        drive.followTrajectorySequence(parkSpot2);
    }

    public void caseC(SampleMecanumDrive drive){
        autoContTurretThread.turretHomePosition = 40;
        autoContTurretThread.nextSlideLevel = slidePositions[0];
        autoContTurretThread.start();

        drive.followTrajectorySequence(traj1);

        autoContTurretThread.interrupt();

        Pose2d poseEstimate = drive.getPoseEstimate();
        double robotAngle = Math.toDegrees(poseEstimate.getHeading());
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;

        double extensionToFirstJunction = getExtensionToObject(firstJunction, poseEstimate);
        double extensionToFirstConeStack = getExtensionToObject(firstConeStack, poseEstimate);

        int ticks = getTicksToObject(firstJunction, poseEstimate);
        int ticks2 = getTicksToObject(firstConeStack, poseEstimate);

        telemetry.addData("ticks", ticks);
        telemetry.addData("ticks2", ticks2);
        telemetry.addData("robotAngle", robotAngle);
        telemetry.addData("imuAngle", imuAngle);

        for(int i = 1; i < slidePositions.length; i++) {
            intakeRoutine(ticks, extensionToFirstConeStack);
            scoreRoutine(slidePositions[i], ticks2, extensionToFirstJunction);
        }

        intakeRoutine(ticks - 120, 1);

        autoContTurretThread.turretHomePosition = Constants.TURRET_TURN_180;
        autoContTurretThread.nextSlideLevel = slidePositions[0];
        autoContTurretThread.junctionLocation = secondJunction;
        autoContTurretThread.start();

        drive.followTrajectorySequence(traj2);
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

    public void intakeRoutinePreload(int turnAngle) {
        slideThread.slideLevel = Constants.SLIDE_HIGH_JUNCTION;
        slideThread.start();

        turretTurnThread.turnAngle = turnAngle;
        turretTurnThread.start();

        while(turretTurnThread.isAlive() || slideThread.isAlive());
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void intakeRoutine(int turnAngle, double extensionToConeStack) {
        linkageSubsystem.setExtensionPosition(extensionToConeStack);

        sleep(350);

        clawSubsystem.useClaw(Constants.CLOSE_CLAW);

        sleep(200);

        slideThread.slideLevel = Constants.SLIDE_HIGH_JUNCTION;
        slideThread.start();

        sleep(200);

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

        turretTurnThread.turnAngle = turnAngle;
        turretTurnThread.start();

        while(turretTurnThread.isAlive() || slideThread.isAlive());
    }

    public void scoreRoutine(int slideLevel, int turnAngle, double extensionToJunction) {
        linkageSubsystem.setExtensionPosition(extensionToJunction);

        sleep(350);

        clawSubsystem.useClaw(Constants.OPEN_CLAW);

        sleep(50);

        linkageSubsystem.setExtensionPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

        turretTurnThread.turnAngle = turnAngle;
        turretTurnThread.start();

        sleep(200);

        slideThread.slideLevel = slideLevel;
        slideThread.start();


        while(slideThread.isAlive() || turretTurnThread.isAlive());
    }


    public double getExtensionToObject(Vector2d object, Pose2d poseEstimate) {
        double robotDistanceToObject = MathUtils.hypot(poseEstimate.getX() - object.getX(), poseEstimate.getY() - object.getY()) * 2.54;
        robotDistanceToObject = Range.clip(robotDistanceToObject, 0, Constants.INTAKE_SLIDE_FULL_EXTENDED_LENGTH_CM);

        return robotDistanceToObject / Constants.INTAKE_SLIDE_FULL_EXTENDED_LENGTH_CM * Constants.INTAKE_SLIDE_EXTENDED_SLIDE;
    }

    public int getTicksToObject(Vector2d object, Pose2d poseEstimate) {
        double robotAngle = Math.toDegrees(poseEstimate.getHeading());
        double angle = robotAngle - Math.toDegrees(MathUtils.atan2(object.getY() - poseEstimate.getY(), object.getX() - poseEstimate.getX()));

        return (int)(angle / 360.0 * (double)Constants.TURRET_FULL_ROTATION);
    }
}