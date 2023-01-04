package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.MathUtils.MathUtils;

@TeleOp(name = "Atan2 BenchMark")
public class Atan2Benchmark extends LinearOpMode {
    float size = 10000000;
    double result;

    @Override
    public void runOpMode() {
        long startTime = System.nanoTime();
        long endTime = System.nanoTime();
        telemetry.addData("time precalculate IceCore.atan2", (endTime - startTime));

        waitForStart();

        while(true) {
            if(gamepad1.x) {
                startTime = System.nanoTime();
                for(float i = 0; i <= size; i++) {
                    result = FastMath.atan2(Math.random() * 100, 1);
                }
                endTime = System.nanoTime();

                telemetry.addData("time FastMath.atan2", (endTime - startTime)/size);
                break;
            }
            else if (gamepad1.b) {
                startTime = System.nanoTime();
                for(float i = 0; i <= size; i++) {
                    result = MathUtils.atan2(Math.random() * 100, 1);
                }
                endTime = System.nanoTime();

                telemetry.addData("time IceCore atan2", (endTime - startTime)/size);
                break;
            }
            else if(gamepad1.y) {
                startTime = System.nanoTime();
                for(float i = 0; i <= size; i++) {
                    result = Math.atan2(Math.random() * 100, 1);
                }
                endTime = System.nanoTime();

                telemetry.addData("time Default atan2", (endTime - startTime)/size);
                break;
            }
        }

        telemetry.update();
        sleep(10000);
    }
}
