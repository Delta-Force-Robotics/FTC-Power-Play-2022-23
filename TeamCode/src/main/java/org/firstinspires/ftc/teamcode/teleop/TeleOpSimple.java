package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp
public class TeleOpSimple extends LinearOpMode {
    private DcMotor turretMotor;
    private Motor leftSlideMotor;
    private Motor rightSlideMotor;
    private BNO055IMU imu;
    private Servo clawServoR;
    private Servo clawServoL;
    private Servo leftIntakeSlideServo;
    private Servo rightIntakeSlideServo;

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        clawServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_RIGHT);
        clawServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_CLAW_SERVO_LEFT);
        leftIntakeSlideServo = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_LEFT);
        rightIntakeSlideServo = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDE_LINKAGE_SERVO_RIGHT);
        turretMotor = hardwareMap.get(DcMotor.class, HardwareConstants.ID_TURRET_MOTOR);
        leftSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        rightSlideMotor = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        clawServoR.setDirection(Servo.Direction.REVERSE);
        rightIntakeSlideServo.setDirection(Servo.Direction.REVERSE);


        clawServoR.setPosition(0);
        clawServoL.setPosition(0);
        leftIntakeSlideServo.setPosition(0);
        rightIntakeSlideServo.setPosition(0);

        rightSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftSlideMotor.setInverted(true);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);

        waitForStart();

        while(opModeIsActive()) {
            Orientation angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (gamepad1.a) {
                clawServoR.setPosition(clawServoR.getPosition() + 0.1);
                clawServoL.setPosition(clawServoL.getPosition() + 0.1);
                sleep(200);
            }
            else if (gamepad1.y) {
                clawServoR.setPosition(clawServoR.getPosition() - 0.1);
                clawServoL.setPosition(clawServoL.getPosition() - 0.1);
                sleep(200);
            }

            if(gamepad1.b) {
                leftIntakeSlideServo.setPosition(leftIntakeSlideServo.getPosition() + 0.01);
                //leftIntakeSlideServo.setPosition(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
                rightIntakeSlideServo.setPosition(rightIntakeSlideServo.getPosition() + 0.01);
                //rightIntakeSlideServo.setPosition(Constants.INTAKE_SLIDE_EXTENDED_SLIDE);
                sleep(200);
            }
            if(gamepad1.x) {
                leftIntakeSlideServo.setPosition(leftIntakeSlideServo.getPosition() - 0.01);
                //leftIntakeSlideServo.setPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

                rightIntakeSlideServo.setPosition(rightIntakeSlideServo.getPosition() - 0.01);
                //rightIntakeSlideServo.setPosition(Constants.INTAKE_SLIDE_INIT_POSITION);

                sleep(200);
            }

            turretMotor.setPower(-gamepad1.left_stick_y);

            leftSlideMotor.set(gamepad1.right_trigger - gamepad1.left_trigger + 0.1);
            rightSlideMotor.set(gamepad1.right_trigger - gamepad1.left_trigger + 0.1);


            telemetry.addData("Gyroscope Z ", angles.firstAngle);
            telemetry.addData("Gyroscope Y ", angles.secondAngle);
            telemetry.addData("Gyroscope X ", angles.thirdAngle);
            telemetry.addData("Linkage Servo Left", leftIntakeSlideServo.getPosition());
            telemetry.addData("Linkage Servo Right", rightIntakeSlideServo.getPosition());
            telemetry.addData("Turret motor", turretMotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", leftSlideMotor.getCurrentPosition());
            telemetry.addData("Right Slide Motor", rightSlideMotor.getCurrentPosition());
            telemetry.addData("Slide Motor Power", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Claw Servo Left", clawServoL.getPosition());
            telemetry.addData("Claw Servo Right", clawServoR.getPosition());
            telemetry.update();

        }
    }
}
