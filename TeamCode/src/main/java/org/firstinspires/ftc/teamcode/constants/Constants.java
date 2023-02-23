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

    //Slide Subsystem/
    public static final int SLIDE_INTAKE = 0;
    public static final int SLIDE_INTERMEDIARY = 65;
    public static final int SLIDE_GR_JUNCTION = 65;
    public static final int SLIDE_LOW_JUNCTION = 339;
    public static final int SLIDE_MID_JUNCTION = 593;
    public static int SLIDE_HIGH_JUNCTION = 662;
    public static int SLIDE_MAX_EXTENSION_TICKS = 710;
    public static double SLIDE_MAX_EXTENSION_MM = 822.2;
    public static double SLIDE_GRAVITY_COMPENSATOR = 0.10;
    public static double SLIDE_MOTOR_PASSIVE_POWER = 0.3;
    public static double SLIDE_ALLOWED_ERROR =  5;
    public static PIDFCoefficients SLIDE_RETRACT_PIDF_COEFF = new PIDFCoefficients(0.0003, 0.004, 0.00005, 0);
    public static PIDFCoefficients SLIDE_EXTEND_PIDF_COEFF = new PIDFCoefficients(1, 0.2, 0, 0);

    //Claw Subsystem
    public static final double OPEN_CLAW = 0;
    public static final double CLOSE_CLAW = 0.2;

    public static final double PIVOT_SERVO_INIT_POSITION = 0.05;
    public static final double PIVOT_SERVO_PIVOT_POSITION = 0.76;

    public static final double FLIP_SERVO_INIT_POSITION = 0;
    public static final double FLIP_SERVO_FLIP_POSITION = 0.55;
    public static final double FLIP_SERVO_FLIP_GR_POSITION = 0;

    public static final double ALIGN_SERVO_INIT_POSITION = 0;
    public static final double ALIGN_SERVO_ALIGN_POSITION = 0.28;
    public static final double ALIGN_SERVO_ALIGN_GR_POSITION = 0;

    //April Tag IDs
    public static final int APRIL_TAG_PARK_ZONE_1 = 13;
    public static final int APRIL_TAG_PARK_ZONE_2 = 23;
    public static final int APRIL_TAG_PARK_ZONE_3 = 31;
}
