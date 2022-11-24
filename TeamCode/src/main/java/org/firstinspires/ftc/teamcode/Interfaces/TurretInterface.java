package org.firstinspires.ftc.teamcode.Interfaces;

public interface TurretInterface {
   void rotateTurretRobotCentric(int rotateTicks);
   void rotateTurretFieldCentric(int rotateTicks);
   int clipTicksToConstraints(int ticks);
}
