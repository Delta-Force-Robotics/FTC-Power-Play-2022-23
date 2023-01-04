package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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


    //Slide Subsystem/
    public static final int SLIDE_INTAKE = 0;
    public static final int SLIDE_GR_JUNCTION = -65;
    public static final int SLIDE_INTERMEDIARY = -65;
    public static final int SLIDE_LOW_JUNCTION = -490;
    public static final int SLIDE_MID_JUNCTION = -790;
    public static final int SLIDE_HIGH_JUNCTION = -1120;

    public static double MOTOR_PASSIVE_POWER = -0.43;

    public static PIDFCoefficients SLIDE_RETRACT_PIDF_COEFF = new PIDFCoefficients(0.003, 0.4, 0.0004, 0);
    public static double SLIDE_GRAVITY_COMPENSATOR = 0.10;

    public static PIDFCoefficients SLIDE_EXTEND_PIDF_COEFF = new PIDFCoefficients(0.01, 0.2, 0.0002, 0);
    public static double SLIDE_ALLOWED_ERROR =  15;

    //Linkage Subsystem
    public static final double INTAKE_SLIDE_INIT_POSITION = 0.19;
    public static final double INTAKE_RETURN_POSITION = 0.19;
    public static final double INTAKE_SLIDE_INTERMEDIARY_POSITION = 0.014;
    public static final double INTAKE_SLIDE_EXTENDED_SLIDE = 0.6;
    public static final double INTAKE_SLIDE_EXTENDED_AUTONOMOUS = 0.21;
    public static double INTAKE_SLIDE_FULL_EXTENDED_LENGTH_CM = 62;

    //Turret Subsystem
    public static final int TURRET_TURN_30 = 189;
    public static final int TURRET_GEAR_RATIO = 84;
    public static final int TURRET_TURN_60 = 378;
    public static final int TURRET_TURN_90 = 567;
    public static final int TURRET_TURN_180 = -1180;
    public static final int TURRET_TURN_135 = 850;
    public static final int TURRET_TURN_225 = -1417;
    public static final int TURRET_CONSTRAINT_MAX = 1185;
    public static final int TURRET_CONSTRAINT_MIN = -1770;
    public static final int TURRET_FULL_ROTATION = 2360;
    public static final double TURRET_AUTO_FF_TERM = 0.6259780907668232;

    public static double TURRET_ALLOWED_ERROR = 6;
    public static PIDFCoefficients TURRET_PIDF_COEFF = new PIDFCoefficients(0.065, 0.0002, 0.001, 0);
    public static PIDFCoefficients TURRET_PIDF_COEFF_CONTINUOUS = new PIDFCoefficients(0, 0, 0, 0);
    public static double TURRET_FF_KS = 0;
    public static double TURRET_FF_KV = 0;
    public static double TURRET_FF_KA = 0;

    //Claw Subsystem
    public static final double OPEN_CLAW = 0.2;
    public static final double CLOSE_CLAW = 0.70;

    //April Tag IDs
    public static final int APRIL_TAG_PARK_ZONE_1 = 13;
    public static final int APRIL_TAG_PARK_ZONE_2 = 23;
    public static final int APRIL_TAG_PARK_ZONE_3 = 31;

    //Claw PwmRange
    //public static int CLAW_RANGE_UPPER = 2500;
    //public static int CLAW_RANGE_LOWER = 500;

    public static double[] REAL_VALUE = {
            23.622, 24.0157, 24.2126, 24.8031, 25.1969,
            25.5906, 25.9843, 26.378, 26.9685, 27.3622,
            27.75591, 28.14961, 28.54331, 28.93701, 29.33071,
            29.9213, 30.315, 30.7087, 31.1024, 31.29921,
            32.08661, 32.48031, 35.4331
    };

    public static double[] DISPLAYED_VALUE = {
            22.05, 22.44, 22.83, 23.23, 23.62, 24.02, 24.41,
            24.80, 25.20, 25.59, 25.98, 26.38, 26.77, 27.17,
            27.56, 27.95, 28.35, 28.74, 29.13, 29.53, 29.92,
            30.31, 33.07
    };
}
