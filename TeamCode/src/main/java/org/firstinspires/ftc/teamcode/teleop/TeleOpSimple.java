package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class TeleOpSimple extends LinearOpMode {
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;

    private Servo clawServo;
    private Servo flipServo;
    private Servo pivotServoLeft;
    private Servo pivotServoRight;
    private Servo alignServo;
    private Servo odometryServoLeft;
    private Servo odometryServoRight;
    private Servo odometryServoStrafe;

    private Encoder leftFwEncoder;
    private Encoder rightFwEncoder;
    private Encoder strafeEncoder;

    double lfPower;
    double rfPower;
    double lbPower;
    double rbPower;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        rf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        lb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_BACK_MOTOR);
        leftSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        rightSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LATERAL_ENCODER));
        rightFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_ENCODER));
        leftFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_ENCODER));

        leftFwEncoder.setDirection(Encoder.Direction.REVERSE);
        strafeEncoder.setDirection(Encoder.Direction.REVERSE);

        clawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO);
        flipServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        pivotServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_LEFT);
        pivotServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_RIGHT);
        alignServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ALIGN_SERVO);
        odometryServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_ODOMETRY_SERVO_LEFT);
        odometryServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_ODOMETRY_SERVO_RIGHT);
        odometryServoStrafe = hardwareMap.get(Servo.class, HardwareConstants.ID_ODOMETRY_SERVO_STRAFE);

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setDirection(Servo.Direction.REVERSE);

        clawServo.setPosition(Constants.OPEN_CLAW);
        pivotServoLeft.setPosition(Constants.PIVOT_SERVO_INIT_POSITION);
        pivotServoRight.setPosition(Constants.PIVOT_SERVO_INIT_POSITION);
        flipServo.setPosition(Constants.FLIP_SERVO_INIT_POSITION);
        alignServo.setPosition(Constants.ALIGN_SERVO_INIT_POSITION);

        odometryServoLeft.setDirection(Servo.Direction.REVERSE);
        odometryServoRight.setDirection(Servo.Direction.REVERSE);
        odometryServoStrafe.setDirection(Servo.Direction.REVERSE);

        odometryServoLeft.setPosition(Constants.ODOMETRY_SERVO_INIT_POSITION);
        odometryServoRight.setPosition(Constants.ODOMETRY_SERVO_INIT_POSITION);
        odometryServoStrafe.setPosition(Constants.ODOMETRY_SERVO_INIT_POSITION);

        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftSlideMotor.setInverted(true);

        waitForStart();

        while(opModeIsActive()) {
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            // Send calculated power to wheels
            lfPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            rfPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            lbPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rbPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

            lf.setPower(lfPower);
            rf.setPower(rfPower);
            lb.setPower(lbPower);
            rb.setPower(rbPower);

            //Position for flipServo
            if (gamepad1.b) {
                flipServo.setPosition(flipServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.x) {
                flipServo.setPosition(flipServo.getPosition() - 0.01);
                sleep(200);
            }

            //Position for pivotServoLeft and pivotServoRight
            if (gamepad1.dpad_up) {
                pivotServoLeft.setPosition(pivotServoLeft.getPosition() + 0.01);
                pivotServoRight.setPosition(pivotServoRight.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.dpad_down){
                pivotServoLeft.setPosition(pivotServoLeft.getPosition() - 0.01);
                pivotServoRight.setPosition(pivotServoRight.getPosition() - 0.01);
                sleep(200);
            }

            //Position for alignServo
            if(gamepad1.dpad_right) {
                alignServo.setPosition(alignServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.dpad_left) {
                alignServo.setPosition(alignServo.getPosition() - 0.01);
                sleep(200);
            }

            //Position for clawServo
            if (gamepad1.a) {
                clawServo.setPosition(clawServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.y) {
                clawServo.setPosition(clawServo.getPosition() - 0.01);
                sleep(200);
            }

            if(gamepad1.left_bumper) {
                odometryServoStrafe.setPosition(odometryServoStrafe.getPosition() + 0.01);
                sleep(200);
            }
            else if(gamepad1.right_bumper) {
                odometryServoStrafe.setPosition(odometryServoStrafe.getPosition() - 0.01);
                sleep(200);
            }

            leftSlideMotor.set(gamepad1.right_trigger - gamepad1.left_trigger);
            rightSlideMotor.set(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Left Slide Motor", leftSlideMotor.getCurrentPosition());
            telemetry.addData("Right Slide Motor", rightSlideMotor.getCurrentPosition());
            telemetry.addData("Slide Motor Power", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Claw Servo", clawServo.getPosition());
            telemetry.addData("Flip Servo", flipServo.getPosition());
            telemetry.addData("Pivot Servo Left", pivotServoLeft.getPosition());
            telemetry.addData("Pivot Servo Right", pivotServoRight.getPosition());
            telemetry.addData("Align Servo", alignServo.getPosition());
            telemetry.addData("Strafe Encoder", strafeEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightFwEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", leftFwEncoder.getCurrentPosition());
            telemetry.addData("Odometry Servo Left", odometryServoLeft.getPosition());
            telemetry.addData("Odometry Servo Right", odometryServoRight.getPosition());
            telemetry.addData("Odometry Servo Strafe", odometryServoStrafe.getPosition());
            telemetry.update();
        }
    }
}
