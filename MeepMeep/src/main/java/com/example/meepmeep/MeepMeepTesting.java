package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(950);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15)
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 14)
                .followTrajectorySequence(drive ->
                        // TRAJECTORY SEQUENCE RED BOTTOM
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35.5, -11.5, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(35.5, -11.5)) // score & parking spot 2
                                //.lineToConstantHeading(new Vector2d(11.5, -11.5)) //parking spot 1
                                //.lineToConstantHeading(new Vector2d(57.5, -11.5)) //parking spot 3
                                .build()

                        // TRAJECTORY SEQUENCE RED TOP
                       /*drive.trajectorySequenceBuilder(new Pose2d(35.5, -60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(35.5, -11.5, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-35.5, -11.5)) // parking position 2
                                .lineToConstantHeading(new Vector2d(-11.5, -11.5)) // parking position 3
                                .lineToConstantHeading(new Vector2d(-57.5, -11.5)) // parking position 1
                                .build()*/

                        // TRAJECTORY SEQUENCE BLUE BOTTOM
                        /*drive.trajectorySequenceBuilder(new Pose2d(-35.5, 60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35.5, 11.5, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(35.5, 11.5)) // parking position 2
                                .lineToConstantHeading(new Vector2d(11.5, 11.5)) // parking position 3
                                .lineToConstantHeading(new Vector2d(57.5, 11.5)) // parking position 1
                                .build()*/

                        // TRAJECTORY SEQUENCE BLUE TOP
                       /* drive.trajectorySequenceBuilder(new Pose2d(35.5, 60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(35.5, 11.5, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-35.5, 11.5)) // parking position 2
                               // .lineToConstantHeading(new Vector2d(-11.5, 11.5)) // parking position 3
                               // .lineToConstantHeading(new Vector2d(-57.5, 11.5)) // parking position 1
                                .build()*/

                        // test lmao
                        /*drive.trajectorySequenceBuilder(new Pose2d(-57.5, -35.5, Math.toRadians(90)))

                                .lineToConstantHeading(new Vector2d(11.5, -35.5))
                                .splineTo(new Vector2d(11.5, 11.5), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(-35.5, 57.5, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(-35.5, -35.5))
                                .lineToLinearHeading(new Pose2d(-57.5, -35.5, Math.toRadians(0)))
                                .build()*/
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}