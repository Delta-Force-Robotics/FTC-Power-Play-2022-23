package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp
public class TeleOpSimple extends LinearOpMode {
    private DcMotor turretMotor;
    private BNO055IMU imu;
    private Servo rightS;
    private Servo leftS;
    private Servo leftIntakeSlideServo;
    private Servo rightIntakeSlideServo;

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        rightS = hardwareMap.get(Servo.class, "right");
        leftS = hardwareMap.get(Servo.class, "left");
        leftIntakeSlideServo = hardwareMap.get(Servo.class, "leftis");
        rightIntakeSlideServo = hardwareMap.get(Servo.class, "rightis");

        rightS.setDirection(Servo.Direction.REVERSE);
        leftIntakeSlideServo.setDirection(Servo.Direction.REVERSE);


        rightS.setPosition(0);
        leftS.setPosition(0);
        leftIntakeSlideServo.setPosition(0);
        rightIntakeSlideServo.setPosition(0);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);

        waitForStart();

        while(opModeIsActive()) {
            Orientation angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Gyroscope X ", angles.firstAngle);
            telemetry.addData("Gyroscope Y ", angles.secondAngle);
            telemetry.addData("Gyroscope Z ", angles.thirdAngle);
            telemetry.addData("Servo Left", leftS.getPosition());
            telemetry.addData("Servo Right", rightS.getPosition());

            telemetry.update();

            if (gamepad1.a) {
                rightS.setPosition(rightS.getPosition() + 0.1);
                leftS.setPosition(leftS.getPosition() + 0.1);
                sleep(200);
            }
            else if (gamepad1.y) {
                rightS.setPosition(rightS.getPosition() - 0.1);
                leftS.setPosition(leftS.getPosition() - 0.1);
                sleep(200);
            }

            if(gamepad1.b) {
                leftIntakeSlideServo.setPosition(leftIntakeSlideServo.getPosition() + 0.01);
                rightIntakeSlideServo.setPosition(rightIntakeSlideServo.getPosition() + 0.01);
                sleep(200);
            }
            if(gamepad1.x) {
                leftIntakeSlideServo.setPosition(leftIntakeSlideServo.getPosition() - 0.01);
                rightIntakeSlideServo.setPosition(rightIntakeSlideServo.getPosition() - 0.01);
                sleep(200);
            }
        }
    }
}
