package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleOpSimple extends LinearOpMode {
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;

    @Override
    public void runOpMode() {

        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("lefSlideMOtor Encoder ", leftSlideMotor.getCurrentPosition());
            telemetry.addData("rightSlideMOtor Encoder ", rightSlideMotor.getCurrentPosition());
            telemetry.update();
        }

    }
}
