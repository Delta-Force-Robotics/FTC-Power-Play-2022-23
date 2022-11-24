package org.firstinspires.ftc.teamcode.constants;

/**
 * A class containing all constants for subsystems. <br><br>
 * This allows us to share constants across OP modes, and makes it easier to change them.
 */

public class Constants {

    //turret state
    public enum TurretTurnState {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    public static TurretTurnState turretTurnState= TurretTurnState.ROBOT_CENTRIC;

    //Slide Subsystem
    public static final int SLIDE_GR_JUNCTION = 0;
    public static final int SLIDE_MID_JUNCTION = 0;
    public static final int SLIDE_LOW_JUNCTION = 0;
    public static final int SLIDE_HIGH_JUNCTION = 0;
    public static final int SLIDE_INTERMEDIARY = 0;
    public static final int SLIDE_INTAKE = 0;

    //Slide Intake Subsystem
    public static final double INTAKE_SLIDE_INIT_POSITION = 0;
    public static final double INTAKE_SLIDE_EXTENDED_SLIDE = 0;
    public static final double INTAKE_SLIDE_EXTEND_10 = 0;

    //Turret Subsystem
    public static final int TURRET_TURN_30 = 63;
    public static final int TURRET_TURN_60 = 126;
    public static final int TURRET_TURN_90 = 189;
    public static final int TURRET_TURN_180 = -378;
    public static final int TURRET_CONSTRAINT_MAX = 189;
    public static final int TURRET_CONSTRAINT_MIN = -378;

    //Claw Subsystem
    public static final double OPEN_CLAW = 0;
    public static final double CLOSE_CLAW = 0;

}
