package org.firstinspires.ftc.teamcode.threads;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MathUtils.MathUtils;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.Supplier;

public class AutoContTurretThread extends Thread{
    public Supplier<Pose2d> poseEstimate;
    public Vector2d junctionLocation;
    public TurretSubsystem turretSubsystem;
    public LinkageSubsystem linkageSubsystem;
    public AutoSlideThread slideThread;
    public ClawSubsystem clawSubsystem;
    public Telemetry telemetry;
    public int turretHomePosition;
    public int nextSlideLevel;

    private PIDFController pidfController;
    private double[] pidfCoefficients;

    public AutoContTurretThread(Supplier<Pose2d> poseEstimate, Vector2d junctionLocation, TurretSubsystem turretSubsystem, LinkageSubsystem linkageSubsystem, ClawSubsystem clawSubsystem, AutoSlideThread slideThread, Telemetry telemetry) {
        this.poseEstimate = poseEstimate;
        this.junctionLocation = junctionLocation;
        this.turretSubsystem = turretSubsystem;
        this.linkageSubsystem = linkageSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.slideThread = slideThread;
        this.telemetry = telemetry;
    }

    @Override
    public void run() {
        turretSubsystem.turretMotor.setRunMode(Motor.RunMode.RawPower);

        pidfCoefficients = new double[]{Constants.TURRET_PIDF_COEFF.p, Constants.TURRET_PIDF_COEFF.i, Constants.TURRET_PIDF_COEFF.d, 0};
        pidfController = new PIDFController(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);

        Pose2d pose = poseEstimate.get();
        int ticks = getTicksToJunction(pose);
        double linkageExtension = getExtensionToJunction(pose);

        pidfController.setSetPoint(ticks);

        while(!isInterrupted() && linkageExtension > Constants.INTAKE_SLIDE_EXTENDED_SLIDE) {
            pose = poseEstimate.get();
            ticks = getTicksToJunction(pose);

            turretSubsystem.turretMotor.set(
                    pidfController.calculate(
                            turretSubsystem.turretMotor.getCurrentPosition(),
                            ticks
                    )
            );

            linkageExtension = getExtensionToJunction(pose);
            linkageSubsystem.setExtensionPosition(linkageExtension);

            telemetry.addData("Ticks ContTurretTurn", ticks);
            telemetry.addData("Linkage Extension", linkageExtension);
            telemetry.update();

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        clawSubsystem.useClaw(Constants.OPEN_CLAW);

        linkageSubsystem.setExtensionPosition(0);

        slideThread.slideLevel = nextSlideLevel;
        slideThread.start();

        turretSubsystem.rotateTurret(turretHomePosition, false);
    }

    public int getTicksToJunction(Pose2d poseEstimate) {
        double robotAngle = Math.toDegrees(poseEstimate.getHeading());
        double angle = robotAngle - Math.toDegrees(MathUtils.atan2(junctionLocation.getY() - poseEstimate.getY(), junctionLocation.getX() - poseEstimate.getX()));

        return (int)(angle / 360.0 * (double)Constants.TURRET_FULL_ROTATION);
    }

    public double getExtensionToJunction(Pose2d poseEstimate) {
        double robotDistanceToJunction = MathUtils.hypot(poseEstimate.getX() - junctionLocation.getX(), poseEstimate.getY() - junctionLocation.getY()) * 2.54;

        return robotDistanceToJunction / (Constants.INTAKE_SLIDE_FULL_EXTENDED_LENGTH_CM + 4.75) * Constants.INTAKE_SLIDE_EXTENDED_SLIDE;
    }
}
