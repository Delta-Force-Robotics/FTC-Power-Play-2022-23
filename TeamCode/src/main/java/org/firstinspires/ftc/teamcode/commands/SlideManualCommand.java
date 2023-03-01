package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

import java.util.function.DoubleSupplier;

public class SlideManualCommand extends CommandBase {

    private SlideSubsystem slideSubsystem;
    private DoubleSupplier rightTrigger;
    private DoubleSupplier leftTrigger;

    public SlideManualCommand(SlideSubsystem slideSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        this.slideSubsystem = slideSubsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;

        addRequirements(slideSubsystem);
    }

    @Override
    public void execute() {
        if(Constants.SLIDE_INPUT_STATE == Constants.InputState.MANUAL_CONTROL) {
            double rightTriggerOutput = (slideSubsystem.getSlideExtensionMeters() > Constants.SLIDE_MANUAL_CONTROL_MAX) ? slideSubsystem.getPassivePower() : Math.max(slideSubsystem.getPassivePower(), rightTrigger.getAsDouble());
            double leftTriggerOutput = leftTrigger.getAsDouble() * (slideSubsystem.getPassivePower() + 0.05);
            double slidePower = rightTriggerOutput - leftTriggerOutput;

            slideSubsystem.setMotorPower(slidePower);
            slideSubsystem.slideState.setId(slideSubsystem.getSlideExtensionMeters());
        }
    }
}
