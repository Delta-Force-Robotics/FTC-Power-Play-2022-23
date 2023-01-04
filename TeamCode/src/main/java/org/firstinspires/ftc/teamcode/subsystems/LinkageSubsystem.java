package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Interfaces.LinkageInterface;

public class LinkageSubsystem extends SubsystemBase implements LinkageInterface {

    private ServoImplEx slideServoL;
    private ServoImplEx slideServoR;

    public enum ExtensionState {
        EXTENDED,
        RETRACTED
    }

    private ExtensionState extensionState = ExtensionState.RETRACTED;

    public LinkageSubsystem(ServoImplEx slideServoL, ServoImplEx slideServoR) {
        this.slideServoL = slideServoL;
        this.slideServoR = slideServoR;
    }

    /**
     * Sets the servos to the intake position
     * @param slideIntakePosition gets to desired position
     */
    public void setExtensionPosition(double slideIntakePosition) {
        slideServoL.setPosition(slideIntakePosition);
        slideServoR.setPosition(slideIntakePosition);
    }

    public ExtensionState getExtensionState() {
        return extensionState;
    }

    public void setExtensionState(ExtensionState extensionState) {
        this.extensionState = extensionState;
    }

    public void disablePwm() {
        slideServoL.setPwmDisable();
        slideServoR.setPwmDisable();
    }
}


