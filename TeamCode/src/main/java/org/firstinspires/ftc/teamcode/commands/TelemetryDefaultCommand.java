package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Device;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.List;

public class TelemetryDefaultCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;
    private SlideSubsystem slideSubsystem;
    private LinkageSubsystem linkageSubsystem;
    private DriveSubsystem driveSubsystem;
    private ClawSubsystem clawSubsystem;
    private IMU turretIMU;
    private IMU chassisIMU;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private List<LynxModule> lynxModuleArraylist;
    private VoltageSensor voltageSensorTurretMotor;
    private VoltageSensor voltageSensorSlideMotorLeft;
    private VoltageSensor voltageSensorSlideMotorRight;

    public TelemetryDefaultCommand(HardwareMap hardwareMap, TurretSubsystem turretSubsystem, SlideSubsystem slideSubsystem, LinkageSubsystem linkageSubsystem, DriveSubsystem driveSubsystem, ClawSubsystem clawSubsystem, IMU chassisIMU, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.turretSubsystem = turretSubsystem;
        this.slideSubsystem = slideSubsystem;
        this.linkageSubsystem = linkageSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.chassisIMU = chassisIMU;
        this.telemetry = telemetry;

        //voltageSensorTurretMotor = hardwareMap.voltageSensor.get(turretSubsystem.);
        //voltageSensorSlideMotorLeft = hardwareMap.voltageSensor.get(HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        //voltageSensorSlideMotorRight = hardwareMap.voltageSensor.get(HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        lynxModuleArraylist = hardwareMap.getAll(LynxModule.class);
        addRequirements(linkageSubsystem);
    }

    @Override
    public void execute() {
        Orientation angles1 = chassisIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Control Hub ", lynxModuleArraylist.get(0).getModuleSerialNumber());
        telemetry.addData("Expansion Hub ", lynxModuleArraylist.get(1).getModuleSerialNumber());

        for(LynxModule lynxModule : lynxModuleArraylist) {
            telemetry.addData("Input Voltage " + lynxModule.getModuleAddress(), lynxModule.getInputVoltage(VoltageUnit.VOLTS));
            telemetry.addData("Input Current " + lynxModule.getModuleAddress(), lynxModule.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Temperature " + lynxModule.getModuleAddress(), lynxModule.getTemperature(TempUnit.CELSIUS));
            telemetry.addData("Connection Info " + lynxModule.getModuleAddress(), lynxModule.getConnectionInfo());
        }

        telemetry.addData("Turret Turn State", Constants.turretTurnState);
        telemetry.addData("Gyroscope 1 Z ", angles1.firstAngle);
        telemetry.addData("Gyroscope 1 Y ", angles1.secondAngle);
        telemetry.addData("Gyroscope 1 X ", angles1.thirdAngle);
        telemetry.addData("Turret Motor", turretSubsystem.turretMotor.getCurrentPosition());
        telemetry.addData("Right Turn", turretSubsystem.rightTurn);
        telemetry.addData("Left Turn", turretSubsystem.leftTurn);
        telemetry.addData("Left Slide Motor", slideSubsystem.slideMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Motor", slideSubsystem.slideMotorRight.getCurrentPosition());
        telemetry.addData("Left relative", turretSubsystem.leftTurnTurretRelative);
        telemetry.addData("Right relative", turretSubsystem.rightTurnTurretRelative);
        telemetry.addData("Linkage Extension Position", linkageSubsystem.getExtensionPosition());
        telemetry.addData("Left Encoder", driveSubsystem.LeftFront.getCurrentPosition());
        telemetry.addData("Right Encoder", driveSubsystem.rightFront.getCurrentPosition());
        telemetry.update();

        for(LynxModule lynxModule : lynxModuleArraylist) {
            Log.d("ROBOT_DBMSG Input Voltage " + lynxModule.getModuleAddress(), "" + lynxModule.getInputVoltage(VoltageUnit.VOLTS));
            Log.d("ROBOT_DBMSG Input Current " + lynxModule.getModuleAddress(), "" + lynxModule.getCurrent(CurrentUnit.AMPS));
            Log.d("ROBOT_DBMSG Temperature " + lynxModule.getModuleAddress(), "" + lynxModule.getTemperature(TempUnit.CELSIUS));
            Log.d("ROBOT_DBMSG Connection Info " + lynxModule.getModuleAddress(), lynxModule.getConnectionInfo());
        }

        Log.d("ROBOT_DBMSG ", "================END CYCLE================\n");
    }
}
