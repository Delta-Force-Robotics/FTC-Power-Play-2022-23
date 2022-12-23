package org.firstinspires.ftc.teamcode.commands;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.SlideThread;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class SlideCommand extends CommandBase {

    SlideSubsystem slideSubsystem;
    SlideThread slideThread;
    Timing.Timer timer;
    DoubleSupplier rightTriggerPower;
    DoubleSupplier leftTriggerPower;
    private int sign;

    public SlideCommand(SlideSubsystem slideSubsystem, DoubleSupplier leftTriggerPower, DoubleSupplier rightTriggerPower) {
        this.slideSubsystem = slideSubsystem;
        this.leftTriggerPower = leftTriggerPower;
        this.rightTriggerPower = rightTriggerPower;

        addRequirements(slideSubsystem);
    }

    @Override
    public void execute() {
    }
}
