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


    //Turret State
    public enum TurretTurnState {
        FIELD_CENTRIC,
        CONTINUOUS_FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }

    public static InputState inputState = InputState.PRESET_POSITIONS;
    public static TurretTurnState turretTurnState = TurretTurnState.FIELD_CENTRIC;

    public static final int gearRatio = 84;

    //Slide Subsystem/
    public static final int SLIDE_INTAKE = 0;
    public static final int SLIDE_GR_JUNCTION = -65;
    public static final int SLIDE_INTERMEDIARY = -65;
    public static final int SLIDE_LOW_JUNCTION = -390;
    public static final int SLIDE_MID_JUNCTION = -750;
    public static final int SLIDE_HIGH_JUNCTION = -1090;

    public static double MOTOR_PASSIVE_POWER = -0.39;

    public static PIDFCoefficients SLIDE_RETRACT_PIDF_COEFF = new PIDFCoefficients(0.002, 0.18, 0.0002, 0);
    public static double SLIDE_GRAVITY_COMPENSATOR = 0.10;

    public static PIDFCoefficients SLIDE_EXTEND_PIDF_COEFF = new PIDFCoefficients(5, 0.00005, 0.00002, 0);
    public static double SLIDE_ALLOWED_ERROR =  5;

    //Linkage Subsystem
    public static final double INTAKE_SLIDE_INIT_POSITION = 0;
    public static final double INTAKE_RETURN_POSITION = 0.0;
    public static final double INTAKE_SLIDE_INTERMEDIARY_POSITION = 0.014;
    public static final double INTAKE_SLIDE_EXTENDED_SLIDE = 0.24;
    public static final double INTAKE_SLIDE_FULL_EXTENDED_LENGTH_CM = 34;

    //Turret Subsystem
    public static final int TURRET_TURN_30 = 189;
    public static final int TURRET_TURN_60 = 378;
    public static final int TURRET_TURN_90 = 567;
    public static final int TURRET_TURN_180 = -1134;
    public static final int TURRET_TURN_135 = 850;
    public static final int TURRET_TURN_225 = -1417;
    public static final int TURRET_CONSTRAINT_MAX = 1185;
    public static final int TURRET_CONSTRAINT_MIN = -1303;
    public static final int TURRET_FULL_ROTATION = 2268;

    public static double TURRET_ALLOWED_ERROR = 2;
    public static PIDFCoefficients TURRET_PIDF_COEFF = new PIDFCoefficients(0.065, 0.0001, 0.001, 0);
    //public static double TURRET_KS = 0; disabled for now
    //public static double TURRET_KV = 0; disabled for now
    //public static double TURRET_KA = 0; disabled for now

    //Claw Subsystem
    public static final double OPEN_CLAW = 0.2;
    public static final double CLOSE_CLAW = 0.71;

    //April Tag IDs
    public static final int APRIL_TAG_PARK_ZONE_1 = 13;
    public static final int APRIL_TAG_PARK_ZONE_2 = 23;
    public static final int APRIL_TAG_PARK_ZONE_3 = 31;
}
