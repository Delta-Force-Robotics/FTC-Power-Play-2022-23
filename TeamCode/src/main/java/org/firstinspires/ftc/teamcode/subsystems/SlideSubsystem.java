package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeamUtils.PIDFController;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.function.BooleanSupplier;

public class SlideSubsystem extends SubsystemBase {
    public Motor slideMotorLeft;
    public Motor slideMotorRight;
    public BooleanSupplier isInterrupted;
    private PIDFController pidfLeftSlideMotor;
    private PIDFController pidfRightSlideMotor;
    private double[] pidfCoefficientsExtend;
    private double[] pidfCoefficientsRetract;
    public double calculateLeft;
    public double calculateRight;
    private Telemetry telemetry;

    public enum SlideState {
        HIGH(Constants.SLIDE_HIGH_JUNCTION),
        MID(Constants.SLIDE_MID_JUNCTION),
        LOW(Constants.SLIDE_LOW_JUNCTION),
        INTAKE(Constants.SLIDE_INTAKE);

        private double id;
        SlideState(double slideLevel) {
            id = slideLevel;
        }

        public double getId() { return id; }
        public void setId(double slideLevel) {
            id = slideLevel;
        }
        public boolean isSameSlideLevel(double slideLevel) { return id == slideLevel; }
    }

    public SlideState slideState = SlideState.INTAKE;

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, Telemetry telemetry, boolean resetEncoders) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        this.slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        this.slideMotorRight.setInverted(true);

        if(resetEncoders) {
            this.slideMotorLeft.resetEncoder();
            this.slideMotorRight.resetEncoder();
        }

        this.telemetry = telemetry;
    }

    /**
     * Sets the extension level for the slides using a PID.
     * @param level Intended level for slide extension, in millimeters.
     */
    public void setLevel(double level) {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.PRESET_POSITIONS;
        slideState.setId(level);

        try {
            sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        pidfCoefficientsExtend = new double[]{Constants.SLIDE_EXTEND_PIDF_COEFF.p, Constants.SLIDE_EXTEND_PIDF_COEFF.i, Constants.SLIDE_EXTEND_PIDF_COEFF.d, Constants.SLIDE_EXTEND_PIDF_COEFF.f};
        pidfCoefficientsRetract = new double[]{Constants.SLIDE_RETRACT_PIDF_COEFF.p, Constants.SLIDE_RETRACT_PIDF_COEFF.i, Constants.SLIDE_RETRACT_PIDF_COEFF.d, Constants.SLIDE_RETRACT_PIDF_COEFF.f};

        if(Math.signum(Constants.SLIDE_HIGH_JUNCTION)*(ticksToMeters(slideMotorLeft.getCurrentPosition()) - level) < 0) {
            telemetry.addData("1", ticksToMeters(slideMotorLeft.getCurrentPosition()) - level);
            pidfRightSlideMotor = new PIDFController(pidfCoefficientsExtend[0], pidfCoefficientsExtend[1], pidfCoefficientsExtend[2], pidfCoefficientsExtend[3]);
            pidfLeftSlideMotor  = new PIDFController(pidfCoefficientsExtend[0], pidfCoefficientsExtend[1], pidfCoefficientsExtend[2], pidfCoefficientsExtend[3]);
        }
        else {
            telemetry.addData("2", ticksToMeters(slideMotorLeft.getCurrentPosition()) - level);
            pidfRightSlideMotor = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
            pidfLeftSlideMotor  = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
        }

        pidfRightSlideMotor.setSetPoint(level);
        pidfLeftSlideMotor.setSetPoint(level);

        pidfRightSlideMotor.setTolerance(Constants.SLIDE_ALLOWED_ERROR, Constants.SLIDE_ALLOWED_VELOCITY_ERROR);
        pidfLeftSlideMotor.setTolerance(Constants.SLIDE_ALLOWED_ERROR, Constants.SLIDE_ALLOWED_VELOCITY_ERROR);

        pidfRightSlideMotor.setMaxErrorIntegration(Constants.SLIDE_MAX_ERROR_INTEGRATION);
        pidfLeftSlideMotor.setMaxErrorIntegration(Constants.SLIDE_MAX_ERROR_INTEGRATION);

        while(!pidfLeftSlideMotor.atSetPoint() && !pidfRightSlideMotor.atSetPoint() && !isInterrupted.getAsBoolean()) {
            calculateRight = pidfRightSlideMotor.calculate(ticksToMeters(slideMotorRight.getCurrentPosition()));
            calculateLeft = pidfLeftSlideMotor.calculate(ticksToMeters(slideMotorLeft.getCurrentPosition()));

            slideMotorRight.set(calculateRight);
            slideMotorLeft.set(calculateLeft);

            telemetry.addData("slideMotorLeft", ticksToMeters(slideMotorLeft.getCurrentPosition()));
            telemetry.addData("slideMotorRight", ticksToMeters(slideMotorRight.getCurrentPosition()));
            telemetry.update();

            try {
                sleep(25);
            } catch (InterruptedException e) {
                Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
                slideMotorLeft.set(Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_MAX_EXTENSION_METERS + 0.1);
                slideMotorRight.set(Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_MAX_EXTENSION_METERS + 0.1);

                e.printStackTrace();
            }
        }

        if (level >= 0.05) {
            slideMotorLeft.set(Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_MAX_EXTENSION_METERS + 0.1);
            slideMotorRight.set(Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_MAX_EXTENSION_METERS + 0.1);
        } else {
            slideMotorLeft.set(0);
            slideMotorRight.set(0);
        }

        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
    }

    public double getPassivePower() {
        if(slideMotorLeft.getCurrentPosition() <= 0.05) {
            return 0;
        }

       return Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)ticksToMeters(slideMotorLeft.getCurrentPosition())/(double)Constants.SLIDE_MAX_EXTENSION_METERS + 0.1;
    }

    public void setMotorPower( double power ) {
        slideMotorLeft.set(power);
        slideMotorRight.set(power);
    }

    public SlideState getSlideState() {
        return slideState;
    }

    public void setSlideState(SlideState slideState) {
        this.slideState = slideState;
    }

    public double ticksToMeters(int ticks) {
        return (double) ticks / Constants.SLIDE_MAX_EXTENSION_TICKS * Constants.SLIDE_MAX_EXTENSION_METERS;
    }

    public double metersToTicks(double meters) {
        return Math.round(meters / Constants.SLIDE_MAX_EXTENSION_METERS * Constants.SLIDE_MAX_EXTENSION_TICKS);
    }

    public int getMotorTicks() {
        return slideMotorLeft.getCurrentPosition();
    }

    public double getSlideExtensionMeters() {
        return ticksToMeters(getMotorTicks());
    }
}