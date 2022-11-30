package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Interfaces.TurretInterface;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.threads.AutoTurretTurnThread;
import org.firstinspires.ftc.teamcode.threads.TurretTurnThread;

import java.util.function.DoubleSupplier;

public class TurretManualControlCommand extends CommandBase {
    private LinkageSubsystem linkageSubsystem;
    private AutoTurretTurnThread autoTurretTurnThread;
    private TurretTurnThread turretTurnThread;
    private DoubleSupplier controllerX;
    private DoubleSupplier controllerY;
    private BNO055IMU imu;
    private TurretInterface turretInterface;
    private Motor turretMotor;

    public TurretManualControlCommand(LinkageSubsystem linkageSubsystem, AutoTurretTurnThread autoTurretTurnThread, TurretTurnThread turretTurnThread, DoubleSupplier controllerX, DoubleSupplier controllerY, BNO055IMU imu) {
        this.linkageSubsystem = linkageSubsystem;
        this.autoTurretTurnThread = autoTurretTurnThread;
        this.turretTurnThread = turretTurnThread;
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.imu = imu;
    }

    @Override
    public void execute() {
        if (Constants.inputState == Constants.InputState.MANUAL_CONTROL) {
            double theta        = Math.atan2(controllerY.getAsDouble(), controllerX.getAsDouble()) / (2 * Math.PI) * 756;
            double extension    = Math.sqrt((controllerX.getAsDouble() * controllerX.getAsDouble()) + (controllerY.getAsDouble() * controllerY.getAsDouble()));

            if (Constants.turretTurnState == Constants.TurretTurnState.ROBOT_CENTRIC || Constants.turretTurnState == Constants.TurretTurnState.FIELD_CENTRIC) {
                turretTurnThread.turnAngle = (int) theta;
                if (!turretTurnThread.isAlive()) {
                    turretTurnThread.run();
                }
            } else {
                autoTurretTurnThread.turnAngle = (int) theta;
                if (!autoTurretTurnThread.isAlive()) {
                    autoTurretTurnThread.run();
                }
            }

            linkageSubsystem.setExtensionPosition(extension);
        }
    }
}
