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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.threads.AutoIntakeThread;
import org.firstinspires.ftc.teamcode.threads.AutoScoreThread;
import org.firstinspires.ftc.teamcode.threads.AutoSlideThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.function.IntConsumer;

@Autonomous
public class BlueRight extends LinearOpMode
{
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

    int[] slidePositions = {-450, -305, -215, -157, -10};
    public int turnAngle;
    public int slideLevel;

    private Vector2d firstConeStack = new Vector2d(-68.5, 11.5);
    private Vector2d secondConeStack = new Vector2d(68.5,11.5);

    private Vector2d firstJunction = new Vector2d(-23.5, 0);
    private Vector2d secondJunction = new Vector2d(23.5,0);

    private Motor leftSlideMotor;
    private Motor rightSlideMotor;
    private Motor turretMotor;

    private Servo leftLinkageServo;
    private Servo rightLinkageServo;
    private Servo rightClawServo;
    private Servo leftClawServo;

    private SampleMecanumDrive drive;
    private TrajectorySequence trajectoryIntakeLeftSide;
    private TrajectorySequence trajectoryIntakeRightSide;
    private TrajectorySequence traj1;
    private TrajectorySequence parkSpot1;
    private TrajectorySequence traj2;
    private TrajectorySequence parkSpot3;

    private AutoScoreThread autoScoreThread;
    private AutoIntakeThread autoIntakeThread;
    private AutoSlideThread autoSlideThread;

    public IntConsumer turretTurnThread;
    public IntConsumer slideLevelThread;
    public IntConsumer scoreThread;
    public IntConsumer intakeThread;

    public Thread turretThread;
    public Thread slideThread;

    private BNO055IMU imu;

    private Timing.Timer timer;

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

        Constants.turretTurnState = Constants.TurretTurnState.ROBOT_CENTRIC;

        leftSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        rightSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);
        turretMotor = new Motor(hardwareMap, HardwareConstants.ID_TURRET_MOTOR);

        leftLinkageServo = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_LEFT);
        rightLinkageServo = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_RIGHT);
        rightClawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_RIGHT);
        leftClawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_LEFT);

        leftSlideMotor.setInverted(true);
        rightClawServo.setDirection(Servo.Direction.REVERSE);
        rightLinkageServo.setDirection(Servo.Direction.REVERSE);

        rightClawServo.setPosition(0);
        leftClawServo.setPosition(0);
        leftLinkageServo.setPosition(0);
        rightLinkageServo.setPosition(0);

        turretMotor.resetEncoder();
        leftSlideMotor.resetEncoder();
        rightSlideMotor.resetEncoder();

        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-35.5, 62.5, Math.toRadians(-90)));

        traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-35.5, 12, Math.toRadians(-90)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-46.5, 12, Math.toRadians(-180)))
                .build();

        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(35.5, 12))
                .build();

        parkSpot1 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(10, 12))
                .build();

        parkSpot3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(59, 12))
                .build();

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

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

        waitForStart();

        if(!isStopRequested()){
            setClawPosition(Constants.CLOSE_CLAW);
            sleep(200);
            setSlideLevel(-99);

            if( tagID == Constants.APRIL_TAG_PARK_ZONE_1 ){
                caseA(drive);
            }
            else if( tagID == Constants.APRIL_TAG_PARK_ZONE_2){
                caseB(drive);
            }
            else {
                caseC(drive);
            }
        }

    }
    public void caseA(SampleMecanumDrive drive){
        drive.followTrajectorySequence(traj1);

        double imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90;
        Pose2d poseEstimate = drive.getPoseEstimate();
        double angle = imuAngle - Math.atan2(firstJunction.getY() - poseEstimate.getY(), firstJunction.getX() - poseEstimate.getX()) * 180.0 / Math.PI;
        int ticks = (int)(angle / 360.0 * 2268.0);

        double angle2 = - imuAngle + Math.atan2(firstConeStack.getY() - poseEstimate.getY(), firstConeStack.getX() - poseEstimate.getX()) * 180.0 / Math.PI;
        int ticks2 = (int)(angle2 / 360.0 * 2268.0);

        for(int slideLevel : slidePositions) {
            intakeRoutine(ticks);

            sleep(500);

            scoreRoutine(slideLevel, ticks2);

            sleep(300);
        }

        Pose2d pose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + Math.toRadians(90)));

        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(parkSpot1);
    }

    public void caseB(SampleMecanumDrive drive){
        drive.followTrajectorySequence(traj1);

        double imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90;
        Pose2d poseEstimate = drive.getPoseEstimate();
        double angle = imuAngle - Math.atan2(firstJunction.getY() - poseEstimate.getY(), firstJunction.getX() - poseEstimate.getX()) * 180.0 / Math.PI;
        int ticks = (int)(angle / 360.0 * 2268.0);

        double angle2 = - imuAngle + Math.atan2(firstConeStack.getY() - poseEstimate.getY(), firstConeStack.getX() - poseEstimate.getX()) * 180.0 / Math.PI;
        int ticks2 = (int)(angle2 / 360.0 * 2268.0);

        for(int slideLevel : slidePositions) {
            intakeRoutine(ticks);

            sleep(500);

            scoreRoutine(slideLevel, ticks2);

            sleep(300);
        }

        Pose2d pose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + Math.toRadians(90)));

        drive.followTrajectorySequence(traj2);
    }

    public void caseC(SampleMecanumDrive drive){
        drive.followTrajectorySequence(traj1);

        double imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90;
        Pose2d poseEstimate = drive.getPoseEstimate();
        double angle = imuAngle - Math.atan2(firstJunction.getY() - poseEstimate.getY(), firstJunction.getX() - poseEstimate.getX()) * 180.0 / Math.PI;
        int ticks = (int)(angle / 360.0 * 2268.0);

        double angle2 = - imuAngle + Math.atan2(firstConeStack.getY() - poseEstimate.getY(), firstConeStack.getX() - poseEstimate.getX()) * 180.0 / Math.PI;
        int ticks2 = (int)(angle2 / 360.0 * 2268.0);

        for(int slideLevel : slidePositions) {
            intakeRoutine(ticks);

            sleep(500);

            scoreRoutine(slideLevel, ticks2);

            sleep(300);
        }

        Pose2d pose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + Math.toRadians(90)));

        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(parkSpot3);
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

    public void intakeRoutine(int turnAngle) {
        setClawPosition(Constants.CLOSE_CLAW);

        sleep(100);

        setSlideLevel(Constants.SLIDE_HIGH_JUNCTION);

        setLinkageExtension(Constants.INTAKE_SLIDE_INIT_POSITION);

        turretTurn(turnAngle);

        sleep(200);

        setLinkageExtension(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
    }

    public void scoreRoutine(int slideLevel, int turnAngle) {
        setClawPosition(Constants.OPEN_CLAW);

        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while(!timer.done()) {

        }
        timer.pause();

        setLinkageExtension(Constants.INTAKE_SLIDE_INIT_POSITION);

        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        timer.start();
        while(!timer.done()) {

        }
        timer.pause();

        turretTurn(turnAngle);
        setSlideLevel(slideLevel);

        setLinkageExtension(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
    }

    public void setClawPosition(double clawPosition) {
        rightClawServo.setPosition(clawPosition);
        leftClawServo.setPosition(clawPosition);
    }

    public void setLinkageExtension(double linkageExtension) {
        leftLinkageServo.setPosition(linkageExtension);
        rightLinkageServo.setPosition(linkageExtension);
    }

    public void setSlideLevel(int slideLevel) {
        leftSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        rightSlideMotor.setRunMode(Motor.RunMode.PositionControl);

        // set and get the position coefficient
        //leftSlideMotor.setPositionCoefficient(Constants.SLIDE_P);
        //rightSlideMotor.setPositionCoefficient(Constants.SLIDE_P);

        // set the target position
        leftSlideMotor.setTargetPosition(slideLevel);      // an integer representing
        rightSlideMotor.setTargetPosition(slideLevel);      // desired tick count

        // set the tolerance
        leftSlideMotor.setPositionTolerance(Constants.SLIDE_ALLOWED_ERROR);   // allowed maximum error
        rightSlideMotor.setPositionTolerance(Constants.SLIDE_ALLOWED_ERROR);   // allowed maximum error


        if (slideLevel == Constants.SLIDE_INTAKE) {
            while (!leftSlideMotor.atTargetPosition() && !rightSlideMotor.atTargetPosition()) {
                while(leftSlideMotor.getCurrentPosition() <= -600) {
                    leftSlideMotor.set(0.0000006);
                    rightSlideMotor.set(0.0000006);
                }
                leftSlideMotor.set(0.000005);
                rightSlideMotor.set(0.000005);

                telemetry.addData("Left Motor Ticks", leftSlideMotor.getCurrentPosition());
                telemetry.addData("Right Motor Ticks", rightSlideMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        else if ((slideLevel == Constants.SLIDE_MID_JUNCTION && leftSlideMotor.getCurrentPosition() < Constants.SLIDE_MID_JUNCTION) ||
                (slideLevel == Constants.SLIDE_LOW_JUNCTION && leftSlideMotor.getCurrentPosition() < Constants.SLIDE_LOW_JUNCTION))
        {
            while (!leftSlideMotor.atTargetPosition() && !rightSlideMotor.atTargetPosition()) {
                leftSlideMotor.set(0.000006);
                rightSlideMotor.set(0.000006);
            }
        }
        else {
            while (!leftSlideMotor.atTargetPosition() && !rightSlideMotor.atTargetPosition()) {
                leftSlideMotor.set(1);
                rightSlideMotor.set(1);

                telemetry.addData("Left Motor Ticks", leftSlideMotor.getCurrentPosition());
                telemetry.addData("Right Motor Ticks", rightSlideMotor.getCurrentPosition());
                telemetry.update();
            }
        }

        if (slideLevel <= -15) {
            leftSlideMotor.setRunMode(Motor.RunMode.RawPower);
            rightSlideMotor.setRunMode(Motor.RunMode.RawPower);
            leftSlideMotor.set(-0.35);
            rightSlideMotor.set(-0.35);
        } else {
            leftSlideMotor.setRunMode(Motor.RunMode.RawPower);
            rightSlideMotor.setRunMode(Motor.RunMode.RawPower);
            leftSlideMotor.set(0);
            rightSlideMotor.set(0);
        }
    }

    public void turretTurn(int turnAngle) {
        /*turretMotor.setRunMode(Motor.RunMode.RawPower);
        PIDFController pidfController = new PIDFController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D, Constants.TURRET_F);
        pidfController.setSetPoint(turnAngle);
        pidfController.setTolerance(Constants.TURRET_ALLOWED_ERROR);


        while(!pidfController.atSetPoint() && opModeIsActive()) {
            telemetry.addData("turretError", pidfController.getPositionError());
            telemetry.update();

            turretMotor.set(
                    pidfController.calculate(
                            turretMotor.getCurrentPosition()
                    )
            );
        }*/
    }

    public int getTurnAngle() {
        return turnAngle;
    }

    public int getSlideLevel() {
        return slideLevel;
    }
}