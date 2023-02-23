package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //drive.trajectorySequenceBuilder(new Pose2d(-35.5, -62.5, Math.toRadians(90)))
                                //.splineToSplineHeading(new Pose2d(-35.5, -23.5, Math.toRadians(90)), Math.toRadians(90))
                                //.splineToSplineHeading(new Pose2d(-55.5, -12.5, Math.toRadians(180)), Math.toRadians(180))
                                //.build());
                        //drive.trajectorySequenceBuilder(new Pose2d(-55, -12, Math.toRadians(-180)))
                                //.lineToLinearHeading(new Pose2d(-35.5, -12, Math.toRadians(-135)))
                                //.lineToLinearHeading(new Pose2d(-33, -9, Math.toRadians(-135)))
                                    //.splineToLinearHeading(new Pose2d(-33, -9, Math.toRadians(-135)), Math.toRadians(30))
                                //.build());
                        //drive.trajectorySequenceBuilder(new Pose2d(-33, -9, Math.toRadians(-135)))   ///park spot 3
                                //.lineToLinearHeading(new Pose2d(-33, -12, Math.toRadians(-180)))
                                //.lineToLinearHeading(new Pose2d(-10, -12, Math.toRadians(180)))
                                //.build());
                        //drive.trajectorySequenceBuilder(new Pose2d(-33, -9, Math.toRadians(-135)))  ///park spot 2
                                //.lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(-180)))
                                //.build());
                        drive.trajectorySequenceBuilder(new Pose2d(-33, -9, Math.toRadians(-135)))  ///park spot 1
                                .lineToLinearHeading(new Pose2d(-33, -12.5, Math.toRadians(-180)))
                                .lineToLinearHeading(new Pose2d(-58, -12.5, Math.toRadians(180)))
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}