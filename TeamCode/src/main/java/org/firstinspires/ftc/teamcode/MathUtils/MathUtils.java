package org.firstinspires.ftc.teamcode.MathUtils;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class MathUtils {
    private static final float PI = (float) Math.PI;
    private static final float PI_2 = PI / 2;
    private static final float PI_NEG_2 = -PI_2;
    private static final int SIZE = 4096;
    private static InterpLUT ATAN2;

    public static float atanScalarApproximation(float x) {
        float a1  =  0.99997726f;
        float a3  = -0.33262347f;
        float a5  =  0.19354346f;
        float a7  = -0.11643287f;
        float a9  =  0.05265332f;
        float a11 = -0.01172120f;

        float x_sq = x * x;
        return x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
    }

    public static float atan2(double y, double x) {
        boolean swap = Math.abs(x) < Math.abs(y);
        float atanInput = Float.parseFloat(String.valueOf((swap ? x : y) / (swap ? y : x)));

        float res = atanScalarApproximation(atanInput);
        res = swap ? (Math.signum(atanInput) * PI_2) - res : res;

        if (x < 0.0f) {
            res = (y >= 0.0f ? PI : -PI) + res;
        }

        return res;
    }

    public static double hypot(double x, double y) {
        return sqrt(x * x + y * y);
    }

    public static double sqrt(double n) {
        double x = Double.longBitsToDouble(0x5fe6ec85e7de30daL - (Double.doubleToLongBits(n) >> 1));

        return n * x * (1.5 - 0.5 * n * x * x);
    }

    public static double turretTicksToDegrees(int ticks) {
        return (double) ticks / (double) Constants.TURRET_FULL_ROTATION * 360;
    }

    public static double turretTicksToRadians(int ticks) {
        return (double) ticks / (double) Constants.TURRET_FULL_ROTATION * 2 * Math.PI;
    }
}
