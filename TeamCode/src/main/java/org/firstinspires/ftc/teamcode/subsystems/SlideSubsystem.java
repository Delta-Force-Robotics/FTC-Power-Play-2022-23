package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

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

    public enum SlideState {
        HIGH(Constants.SLIDE_HIGH_JUNCTION),
        MID(Constants.SLIDE_MID_JUNCTION),
        LOW(Constants.SLIDE_LOW_JUNCTION),
        INTAKE(Constants.SLIDE_INTAKE);

        private int id;
        SlideState(int slideLevel) {
            id = slideLevel;
        }

        int getId() { return id; }
        void setId(int slideLevel) {
            id = slideLevel;
        }
        boolean isSameSlideLevel(int slideLevel) { return id == slideLevel; }
    }

    private SlideState slideState = SlideState.INTAKE;

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, boolean resetEncoders) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.slideMotorRight.setInverted(true);

        if(resetEncoders) {
            this.slideMotorLeft.resetEncoder();
            this.slideMotorRight.resetEncoder();
        }
    }

    /**
     * Sets the extension level for the slides using a PID.
     * @param level Intended level for slide extension, in millimeters.
     */
    public void setLevel(int level) {
        if(slideState.isSameSlideLevel(level)) {
            return;
        }

        slideState.setId(level);

        pidfCoefficientsExtend  = new double[]{Constants.SLIDE_EXTEND_PIDF_COEFF.p, Constants.SLIDE_EXTEND_PIDF_COEFF.i, Constants.SLIDE_EXTEND_PIDF_COEFF.d, Constants.SLIDE_EXTEND_PIDF_COEFF.f};
        pidfCoefficientsRetract = new double[]{Constants.SLIDE_RETRACT_PIDF_COEFF.p, Constants.SLIDE_RETRACT_PIDF_COEFF.i, Constants.SLIDE_RETRACT_PIDF_COEFF.d, Constants.SLIDE_RETRACT_PIDF_COEFF.f};

        slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        if(Math.signum(Constants.SLIDE_GR_JUNCTION)*(slideMotorLeft.getCurrentPosition() - level) < 0) {
            pidfRightSlideMotor = new PIDFController(pidfCoefficientsExtend[0], pidfCoefficientsExtend[1], pidfCoefficientsExtend[2], pidfCoefficientsExtend[3]);
            pidfLeftSlideMotor  = new PIDFController(pidfCoefficientsExtend[0], pidfCoefficientsExtend[1], pidfCoefficientsExtend[2], pidfCoefficientsExtend[3]);
        }
        else {
            pidfRightSlideMotor = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
            pidfLeftSlideMotor  = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
        }

        pidfRightSlideMotor.setSetPoint(level);
        pidfLeftSlideMotor.setSetPoint(level);

        pidfRightSlideMotor.setTolerance(Constants.SLIDE_ALLOWED_ERROR);
        pidfLeftSlideMotor.setTolerance(Constants.SLIDE_ALLOWED_ERROR);

        pidfRightSlideMotor.setMaxErrorIntegration(20);
        pidfLeftSlideMotor.setMaxErrorIntegration(20);

        while(!pidfLeftSlideMotor.atSetPoint() && !pidfRightSlideMotor.atSetPoint() && !isInterrupted.getAsBoolean()) {
            calculateRight = pidfRightSlideMotor.calculate(ticksToMm(slideMotorRight.getCurrentPosition()));
            calculateLeft = pidfLeftSlideMotor.calculate(ticksToMm(slideMotorLeft.getCurrentPosition()));

            slideMotorRight.set(calculateRight);
            slideMotorLeft.set(calculateLeft);

            if(isInterrupted.getAsBoolean()) {
                slideMotorRight.set(0);
                slideMotorLeft.set(0);
            }

            try {
                Thread.sleep(12);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (level >= 5) {
            slideMotorLeft.set(Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_MAX_EXTENSION_MM);
            slideMotorRight.set(Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_MAX_EXTENSION_MM);
        } else {
            slideMotorLeft.set(0);
            slideMotorRight.set(0);
        }
    }

    public SlideState getSlideState() {
        return slideState;
    }

    public void setSlideState(SlideState slideState) {
        this.slideState = slideState;
    }

    public double ticksToMm(int ticks) {
        return (double) ticks / Constants.SLIDE_MAX_EXTENSION_TICKS * Constants.SLIDE_MAX_EXTENSION_MM;
    }

    public double mmToTicks(double mills) {
        return Math.round(mills / Constants.SLIDE_MAX_EXTENSION_MM * Constants.SLIDE_MAX_EXTENSION_TICKS);
    }
}