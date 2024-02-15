package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCyclingAuton {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(
                                new Pose2d(12.8,71.2, Math.toRadians(-90))).setTangent(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(22.8,29.6, Math.toRadians(-148)))
                                .lineToLinearHeading(new Pose2d(50.8,34, Math.toRadians(-6)))
                                .lineToLinearHeading(new Pose2d(42.4,12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-62,12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36.4,12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(50,26, Math.toRadians(9)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}