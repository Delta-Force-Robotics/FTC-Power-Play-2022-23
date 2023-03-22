package org.firstinspires.ftc.teamcode.constants;

/**
 * A class containing all constants for motors' and servos' titles. <br><br>
 * This allows us to share constants across OP modes, and makes it easier to change them.
 */

public class HardwareConstants {
    // Drive motors
    public static final String ID_LEFT_FRONT_MOTOR = "lf";
    public static final String ID_LEFT_BACK_MOTOR = "lb";
    public static final String ID_RIGHT_FRONT_MOTOR = "rf";
    public static final String ID_RIGHT_BACK_MOTOR = "rb";

    // Slide motors
    public static final String ID_SLIDE_MOTOR_LEFT = "slideMotorL";
    public static final String ID_SLIDE_MOTOR_RIGHT = "slideMotorR";

    // Claw servos
    public static final String ID_INTAKE_CLAW_SERVO = "clawServo";
    public static final String ID_FLIP_SERVO = "flipServo";
    public static final String ID_PIVOT_SERVO_LEFT = "pivotServoLeft";
    public static final String ID_PIVOT_SERVO_RIGHT = "pivotServoRight";
    public static final String ID_ALIGN_SERVO = "alignServo";

    //dead wheels
    public static final String ID_LEFT_ENCODER = "lf"; // primu port
    public static final String ID_RIGHT_ENCODER = "rb"; // port 4
    public static final String ID_LATERAL_ENCODER = "strafeEncoder"; // exp port 4

    public static final String ID_ODOMETRY_SERVO_LEFT = "odometryServoLeft";

    public static final String ID_ODOMETRY_SERVO_RIGHT = "odometryServoRight";
    public static final String ID_ODOMETRY_SERVO_STRAFE = "odometryServoStrafe";
}
