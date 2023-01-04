package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    private PIDFController pidfExtendLeft;
    private PIDFController pidfExtendRight;
    private PIDFController pidfRetractLeft;
    private PIDFController pidfRetractRight;
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

        if(!auto) {
            pidfCoefficientsExtend = new double[]{Constants.SLIDE_EXTEND_PIDF_COEFF.p, Constants.SLIDE_EXTEND_PIDF_COEFF.i, Constants.SLIDE_EXTEND_PIDF_COEFF.d, Constants.SLIDE_EXTEND_PIDF_COEFF.f};
            pidfCoefficientsRetract = new double[]{Constants.SLIDE_RETRACT_PIDF_COEFF.p, Constants.SLIDE_RETRACT_PIDF_COEFF.i, Constants.SLIDE_RETRACT_PIDF_COEFF.d, Constants.SLIDE_RETRACT_PIDF_COEFF.f};
        }
        else {
            pidfCoefficientsExtend = new double[]{Constants.SLIDE_EXTEND_PIDF_COEFF.p+0.02, Constants.SLIDE_EXTEND_PIDF_COEFF.i, Constants.SLIDE_EXTEND_PIDF_COEFF.d, Constants.SLIDE_EXTEND_PIDF_COEFF.f};
            pidfCoefficientsRetract = new double[]{Constants.SLIDE_RETRACT_PIDF_COEFF.p, Constants.SLIDE_RETRACT_PIDF_COEFF.i, Constants.SLIDE_RETRACT_PIDF_COEFF.d, Constants.SLIDE_RETRACT_PIDF_COEFF.f};
        }

        pidfExtendLeft = new PIDFController(pidfCoefficientsExtend[0], pidfCoefficientsExtend[1], pidfCoefficientsExtend[2], pidfCoefficientsExtend[3]);
        pidfExtendRight = new PIDFController(pidfCoefficientsExtend[0], pidfCoefficientsExtend[1], pidfCoefficientsExtend[2], pidfCoefficientsExtend[3]);
        pidfRetractLeft = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);
        pidfRetractRight = new PIDFController(pidfCoefficientsRetract[0], pidfCoefficientsRetract[1], pidfCoefficientsRetract[2], pidfCoefficientsRetract[3]);

        slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        if(slideMotorLeft.getCurrentPosition() - level < 0) {
            pidfRetractRight.setSetPoint(level);
            pidfRetractLeft.setSetPoint(level);

            pidfRetractRight.setTolerance(Constants.SLIDE_ALLOWED_ERROR);
            pidfRetractLeft.setTolerance(Constants.SLIDE_ALLOWED_ERROR);

            while(!pidfRetractRight.atSetPoint() && !pidfRetractLeft.atSetPoint() && !isInterrupted.getAsBoolean()) {
                double timeStart = System.nanoTime();

                calculateRight = pidfRetractRight.calculate(slideMotorRight.getCurrentPosition(), level);
                calculateLeft = pidfRetractLeft.calculate(slideMotorLeft.getCurrentPosition(), level);

                slideMotorRight.set(
                        pidfRetractRight.calculate(
                                slideMotorRight.getCurrentPosition(),
                                level
                        ) - Constants.SLIDE_GRAVITY_COMPENSATOR
                );

                slideMotorLeft.set(
                        pidfRetractLeft.calculate(
                                slideMotorLeft.getCurrentPosition(),
                                level
                        ) - Constants.SLIDE_GRAVITY_COMPENSATOR
                );

                timerAverage += System.nanoTime() - timeStart;
                timerCount++;

                telemetry.addData("CalculateRight", calculateRight);
                telemetry.addData("CalculateLeft", calculateLeft);

                telemetry.addData("Slide Error", pidfRetractRight.getPositionError());

                telemetry.update();
            }

            telemetry.addData("Loop Time Average", timerAverage / timerCount);
            telemetry.addData("Slide Error", pidfRetractRight.getPositionError());
            telemetry.update();
        }
        else {
            pidfExtendRight.setSetPoint(level);
            pidfExtendLeft.setSetPoint(level);

            pidfExtendRight.setTolerance(Constants.SLIDE_ALLOWED_ERROR);
            pidfExtendLeft.setTolerance(Constants.SLIDE_ALLOWED_ERROR);

            while(!pidfExtendRight.atSetPoint() && !pidfExtendLeft.atSetPoint() && !isInterrupted.getAsBoolean()) {
                double timeStart = System.nanoTime();

                slideMotorRight.set(
                        pidfExtendRight.calculate(
                                slideMotorRight.getCurrentPosition(),
                                level
                        )
                );

                slideMotorLeft.set(
                        pidfExtendLeft.calculate(
                                slideMotorLeft.getCurrentPosition(),
                                level
                        )
                );

                timerAverage += System.nanoTime() - timeStart;
                timerCount++;

                telemetry.addData("Slide Error", pidfExtendRight.getPositionError());
                telemetry.addData("Loop time", System.nanoTime() - timeStart);
                telemetry.update();
            }

            telemetry.addData("Loop Time Average", timerAverage / timerCount);
            telemetry.addData("Slide Error", pidfExtendRight.getPositionError());
            telemetry.update();
        }

        if (level <= -15) {
            slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
            slideMotorRight.setRunMode(Motor.RunMode.RawPower);
            slideMotorLeft.set(Constants.MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_HIGH_JUNCTION);
            slideMotorRight.set(Constants.MOTOR_PASSIVE_POWER * (double)level/(double)Constants.SLIDE_HIGH_JUNCTION);
        } else {
            slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
            slideMotorRight.setRunMode(Motor.RunMode.RawPower);
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