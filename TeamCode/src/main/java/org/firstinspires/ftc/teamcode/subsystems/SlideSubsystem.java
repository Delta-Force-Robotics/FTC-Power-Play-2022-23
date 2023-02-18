package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.TeamUtils.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.SlideInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.Timer;
import java.util.function.BooleanSupplier;

public class SlideSubsystem extends SubsystemBase implements SlideInterface {
    public Motor slideMotorLeft;
    public Motor slideMotorRight;
    public BooleanSupplier isInterrupted;
    private PIDFController pidfLeftSlideMotor;
    private PIDFController pidfRightSlideMotor;
    private double[] pidfCoefficientsExtend;
    private double[] pidfCoefficientsRetract;
    private Telemetry telemetry;

    //test
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

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, Telemetry telemetry) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;
        this.telemetry = telemetry;
    }

    /**
     * Sets the extension level for the slides using a PID.
     * @param level Intended level for slide extension.
     */
    @Override
    public void setLevel(int level, boolean auto) {
        if(slideState.isSameSlideLevel(level)) {
            return;
        }

        slideState.setId(level);

        double timerAverage = 0;
        int timerCount = 0;

        pidfCoefficientsExtend = new double[]{Constants.SLIDE_EXTEND_PIDF_COEFF.p, Constants.SLIDE_EXTEND_PIDF_COEFF.i, Constants.SLIDE_EXTEND_PIDF_COEFF.d, Constants.SLIDE_EXTEND_PIDF_COEFF.f};
        pidfCoefficientsRetract = new double[]{Constants.SLIDE_RETRACT_PIDF_COEFF.p, Constants.SLIDE_RETRACT_PIDF_COEFF.i, Constants.SLIDE_RETRACT_PIDF_COEFF.d, Constants.SLIDE_RETRACT_PIDF_COEFF.f};

        slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        if(Math.signum(Constants.SLIDE_GR_JUNCTION)*(slideMotorLeft.getCurrentPosition() + level) > 0) {
            pidfRightSlideMotor = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
            pidfLeftSlideMotor = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
        }
        else {
            pidfRightSlideMotor = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
            pidfLeftSlideMotor = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
        }

        pidfRightSlideMotor.setSetPoint(level);
        pidfLeftSlideMotor.setSetPoint(level);

        pidfRightSlideMotor.setTolerance(Constants.SLIDE_ALLOWED_ERROR);
        pidfLeftSlideMotor.setTolerance(Constants.SLIDE_ALLOWED_ERROR);

        pidfRightSlideMotor.setMaxErrorIntegration(5);
        pidfLeftSlideMotor.setMaxErrorIntegration(5);

        while(!pidfLeftSlideMotor.atSetPoint() && !pidfRightSlideMotor.atSetPoint() && !isInterrupted.getAsBoolean()) {
            double timeStart = System.nanoTime();

            calculateRight = pidfRightSlideMotor.calculate(slideMotorRight.getCurrentPosition(), level);
            calculateLeft = pidfLeftSlideMotor.calculate(slideMotorLeft.getCurrentPosition(), level);

            slideMotorRight.set(
                    pidfRightSlideMotor.calculate(
                            slideMotorRight.getCurrentPosition(),
                            level
                    )
            );

            slideMotorLeft.set(
                    pidfLeftSlideMotor.calculate(
                            slideMotorLeft.getCurrentPosition(),
                            level
                    )
            );

            timerAverage += System.nanoTime() - timeStart;
            timerCount++;

            telemetry.addData("CalculateRight", calculateRight);
            telemetry.addData("CalculateLeft", calculateLeft);

            telemetry.addData("Slide Error", pidfLeftSlideMotor.getPositionError());

            telemetry.update();
        }

        if (level <= -5) {
            slideMotorLeft.set(Constants.MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_HIGH_JUNCTION);
            slideMotorRight.set(Constants.MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_HIGH_JUNCTION);
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
}