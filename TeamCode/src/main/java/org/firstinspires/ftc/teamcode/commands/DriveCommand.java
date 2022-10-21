package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    DoubleSupplier str;
    DoubleSupplier fwd;
    DoubleSupplier rot;

    MecanumSubsystem mecanumSubsystem;

    public DriveCommand(MecanumSubsystem mecanumSubsystem, DoubleSupplier str, DoubleSupplier fwd, DoubleSupplier rot){
        this.str =str;
        this.fwd =fwd;
        this.rot =rot;
        this.mecanumSubsystem = mecanumSubsystem;
    }

    @Override
    public void execute() {
        mecanumSubsystem.drive(str.getAsDouble(), -fwd.getAsDouble(), rot.getAsDouble());

    }
}
