package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.MathUtils.MathUtils.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.MathUtils.MathUtils;

import java.util.Random;

@TeleOp(name = "Hypot Benchmark")
public class HypotBenchmark extends LinearOpMode {
    float size = 1000000;
    double result;
    long startTime, endTime;

    @Override
    public void runOpMode() {
        waitForStart();

        while(true) {
            if(gamepad1.x) {
                double x = 0, y = 0;
                startTime = System.nanoTime();
                for(float i = 0; i <= size; i++) {
                    x = Math.random()*100;
                    y = Math.random()*100;
                    result = MathUtils.hypot(x, y);
                }
                endTime = System.nanoTime();

                telemetry.addData("time MathUtils.hypot", (endTime - startTime)/size);
                telemetry.addData("result", result);
                telemetry.addData("value 1", x);
                telemetry.addData("value 2", y);
                telemetry.update();
                break;
            }
            else if (gamepad1.b) {
                startTime = System.nanoTime();
                for(float i = 0; i <= size; i++) {
                    result = MathUtils.hypot(Math.random()*100, Math.random()*100);
                }
                endTime = System.nanoTime();

                telemetry.addData("time IceCore hypot", (endTime - startTime)/size);
                break;
            }
            else if(gamepad1.y) {
                startTime = System.nanoTime();
                for(float i = 0; i <= size; i++) {
                    result = Math.hypot(Math.random()*100, Math.random()*100);
                }
                endTime = System.nanoTime();

                telemetry.addData("time Default hypot", (endTime - startTime)/size);
                break;
            }
        }

        sleep(10000);
    }
}
