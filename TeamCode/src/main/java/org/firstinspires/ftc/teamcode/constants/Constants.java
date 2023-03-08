package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A class containing all constants for subsystems. <br><br>
 * This allows us to share constants across OP modes, and makes it easier to change them.
 */

@Config
public class Constants {
    public static boolean ROBOT_STOPPED = false;
    public enum InputState {
        MANUAL_CONTROL,
        PRESET_POSITIONS;

    }

    public static InputState SLIDE_INPUT_STATE = InputState.MANUAL_CONTROL;


    //Slide Subsystem/
    public static final double SLIDE_INTAKE = 0.0;
    public static final double SLIDE_GR_JUNCTION = 0.0;
    public static final double SLIDE_LOW_JUNCTION = 0.19;
    public static final double SLIDE_MID_JUNCTION = 0.431;
    public static final double SLIDE_MID_JUNCTION_TELEOP = 0.435;
    public static final double SLIDE_HIGH_JUNCTION = 0.65;
    public static double SLIDE_HIGH_JUNCTION_AUTO = 0.68;
    public static double SLIDE_MANUAL_CONTROL_MAX = 0.72;
    public static double SLIDE_MAX_EXTENSION_TICKS = 1048;
    public static double SLIDE_MAX_EXTENSION_METERS = 0.822;
    public static double SLIDE_GRAVITY_COMPENSATOR = 0.1;
    public static double SLIDE_MOTOR_PASSIVE_POWER = 0.25;
    public static double SLIDE_ALLOWED_ERROR = 0.01;
    public static double SLIDE_ALLOWED_VELOCITY_ERROR = 0.07;
    public static double SLIDE_MAX_ERROR_INTEGRATION = 0.1;
    public static double[] SLIDE_POSITIONS_CONESTACK = {0.15, 0.13, 0.09, 0.05, 0.0};
    public static PIDFCoefficients SLIDE_RETRACT_PIDF_COEFF = new PIDFCoefficients(3, 0, 0.078, 0.09);
    public static PIDFCoefficients SLIDE_EXTEND_PIDF_COEFF = new PIDFCoefficients(8, 0.1, 0.3, 0.04);
    public static PIDFCoefficients SLIDE_RETRACT_PIDF_COEFF_TELEOP = new PIDFCoefficients(3, 0, 0.078, 0.09);
    public static PIDFCoefficients SLIDE_EXTEND_PIDF_COEFF_TELEOP = new PIDFCoefficients(10, 0.13, 0, 0);

    //Claw Subsystem
    public static final double OPEN_CLAW = 0.01;
    public static final double CLOSE_CLAW_AUTO = 0.2;
    public static final double CLOSE_CLAW_TELEOP = 0.18;

    public static final double PIVOT_SERVO_PIVOT_POSITION_TELEOP = 0.82;
    public static final double PIVOT_SERVO_INIT_POSITION = 0.05;
    public static final double PIVOT_SERVO_PIVOT_POSITION = 0.76;
    public static final double PIVOT_SERVO_INIT_AUTO_POSITION = 0.5;
    public static final double PIVOT_SERVO_UP_POSSITION = 0.22;
    public static final double PIVOT_SERVO_DOWN_POSSITION = 0.14;

    public static final double FLIP_SERVO_INIT_POSITION = 0;
    public static final double FLIP_SERVO_FLIP_POSITION = 0.56;
    public static final double FLIP_SERVO_FLIP_GR_POSITION = 0;

    public static final double ALIGN_SERVO_INIT_POSITION = 0;
    public static final double ALIGN_SERVO_ALIGN_POSITION = 0.28;
    public static final double ALIGN_SERVO_ALIGN_GR_POSITION = 0;

    public static final double ODOMETRY_SERVO_INIT_POSITION = 0;
    public static final double ODOMETRY_SERVO_RETRACTED_POSITION = 1;

    //April Tag IDs
    public static final int APRIL_TAG_PARK_ZONE_1 = 13;
    public static final int APRIL_TAG_PARK_ZONE_2 = 23;
    public static final int APRIL_TAG_PARK_ZONE_3 = 31;
}
