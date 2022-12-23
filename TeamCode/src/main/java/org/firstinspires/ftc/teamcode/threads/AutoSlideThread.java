package org.firstinspires.ftc.teamcode.threads;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class AutoSlideThread extends Thread {
    public SlideSubsystem slideSubsystem;
    public int slideLevel;

    public AutoSlideThread(SlideSubsystem slideSubsystem) {
        this.slideSubsystem = slideSubsystem;
    }

    @Override
    public void run() {
        slideSubsystem.setLevel(slideLevel, false);
    }
}
