package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * A class containing all constants for subsystems. <br><br>
 * This allows us to share constants across OP modes, and makes it easier to change them.
 */

@Config
public class Constants {

    public static boolean ROBOT_STOPPED = false;

    public enum InputState {
        MANUAL_CONTROL,
        PRESET_POSITIONS
    }

    //turret state
    public enum TurretTurnState {
        FIELD_CENTRIC,
        CONTINUOUS_FIELD_CENTRIC,
        ROBOT_CENTRIC,
    }

    public static InputState inputState = InputState.PRESET_POSITIONS;
    public static TurretTurnState turretTurnState = TurretTurnState.CONTINUOUS_FIELD_CENTRIC;

    public static final int gearRatio = 84;

    //Slide Subsystem
    public static final int SLIDE_INTAKE = -8;
    public static final int SLIDE_GR_JUNCTION = -12;
    public static final int SLIDE_INTERMEDIARY = -15;
    public static final int SLIDE_LOW_JUNCTION = -1108;
    public static final int SLIDE_MID_JUNCTION = -1920;
    public static final int SLIDE_HIGH_JUNCTION = -2900;

    public static final double SLIDE_P = 0.03;
    public static final double SLIDE_ALLOWED_ERROR = 40;

    //Slide Intake Subsystem
    public static final double INTAKE_SLIDE_INIT_POSITION = 0;
    public static final double INTAKE_SLIDE_INTERMEDIARY_POSITION = 0.025;
    public static final double INTAKE_SLIDE_EXTENDED_SLIDE = 0.24;
    public static final double INTAKE_SLIDE_FULL_EXTENDED_LENGTH_CM = 34;

    //Turret Subsystem
    public static final int TURRET_TURN_30 = 189;
    public static final int TURRET_TURN_60 = 378;
    public static final int TURRET_TURN_90 = 567;
    public static final int TURRET_TURN_180 = -1134;
    public static final int TURRET_CONSTRAINT_MAX = 1185;
    public static final int TURRET_CONSTRAINT_MIN = -1707;
    public static final int TURRET_FULL_ROTATION = 2268;

    public static double OP_MODE_START_GYRO_ZERO = 0;

    public static double TURRET_P = 0.005;
    public static double TURRET_I = 0.15;
    public static double TURRET_D = 0;
    public static double TURRED_F = 0;
    public static double TURRET_ALLOWED_ERROR = 6;
    //public static double TURRET_KS = 0; disabled for now
    //public static double TURRET_KV = 0; disabled for now
    //public static double TURRET_KA = 0; disabled for now

    //Claw Subsystem
    public static final double OPEN_CLAW = 0.1;
    public static final double CLOSE_CLAW = 0.70;

    //April Tag IDs
    public static final int APRIL_TAG_PARK_ZONE_1 = 131;
    public static final int APRIL_TAG_PARK_ZONE_2 = 563;
    public static final int APRIL_TAG_PARK_ZONE_3 = 797;
}
