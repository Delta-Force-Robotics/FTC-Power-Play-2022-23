package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(950);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15)
                .setConstraints(45, 45, Math.toRadians(360), Math.toRadians(360), 14)
                .followTrajectorySequence(drive ->
                        // TRAJECTORY SEQUENCE RED LEFT
                       drive.trajectorySequenceBuilder(new Pose2d(35.5, -62.5, Math.toRadians(90)))
                               .splineToSplineHeading(new Pose2d(35.5, -35.5, Math.toRadians(90)), Math.toRadians(90))
                               .splineToSplineHeading(new Pose2d(46, -13, Math.toRadians(0)), Math.toRadians(0))
                               .lineToLinearHeading(new Pose2d(-46, -13, Math.toRadians(0)))
                               //.lineToLinearHeading(new Pose2d(35.5, -11.5, Math.toRadians(90)))
                               //.turn(Math.toRadians(90))
                               //.lineToLinearHeading(new Pose2d(45.5, -11.5, Math.toRadians(180)))
                               //.lineToLinearHeading(new Pose2d(-45.5, -11.5, Math.toRadians(180)))
                               //.lineToLinearHeading(new Pose2d(-35.5, -11.5, Math.toRadians(180)))
                               //.turn(Math.toRadians(90))
                               // score & parking spot 2
                               // .lineToConstantHeading(new Vector2d(11.5, -11.5)) //parking spot 1
                               // .lineToConstantHeading(new Vector2d(57.5, -11.5)) //parking spot 3
                                .build()

                        // TRAJECTORY SEQUENCE RED RIGHT
                       /*drive.trajectorySequenceBuilder(new Pose2d(35.5, -62.5, Math.toRadians(90)))
                               .lineToLinearHeading(new Pose2d(35.5, -9, Math.toRadians(90)))
                               .turn(Math.toRadians(-90))
                               .lineToConstantHeading(new Vector2d(11.5, -9))
                                .build()*/

                        // TRAJECTORY SEQUENCE BLUE RIGHT
                        /*drive.trajectorySequenceBuilder(new Pose2d(-35.5, 62.5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-35.5, 11.5, Math.toRadians(-90)))
                                .turn(Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(35.5, 11.5)) // parking position 2
                                .lineToConstantHeading(new Vector2d(11.5, 11.5)) // parking position 3
                                .lineToConstantHeading(new Vector2d(57.5, 11.5)) // parking position 1
                                .build()*/

                        // TRAJECTORY SEQUENCE BLUE LEFT
                      /*drive.trajectorySequenceBuilder(new Pose2d(35.5, 62.5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(35.5, 11.5, Math.toRadians(-90)))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(-35.5, 11.5)) // parking position 2
                                .lineToConstantHeading(new Vector2d(-11.5, 11.5)) // parking position 3
                                .lineToConstantHeading(new Vector2d(-57.5, 11.5)) // parking position 1
                                .build()*/

                        // TEST LMAO
                        /*drive.trajectorySequenceBuilder(new Pose2d(-57.5, -35.5, Math.toRadians(90)))

                                .lineToConstantHeading(new Vector2d(11.5, -35.5))
                                .splineTo(new Vector2d(11.5, 11.5), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(-35.5, 57.5, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(-35.5, -35.5))
                                .lineToLinearHeading(new Pose2d(-57.5, -35.5, Math.toRadians(0)))
                                .build()*/
                        /*drive = new SampleMecanumDrive(hardwareMap);
                        drive.setPoseEstimate(new Pose2d(-35.5, -58, Math.toRadians(90)));

                        traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(-35.5,-9))
                        .build();*/

        /*traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(35.5, -9))

                .build();*/

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}