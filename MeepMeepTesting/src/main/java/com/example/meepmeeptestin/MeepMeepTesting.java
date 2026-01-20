package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 38, 0))
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(136)), Math.toRadians(0))
                .waitSeconds(2)

                .splineToLinearHeading(
                        new Pose2d(-13, 12, Math.toRadians(91)),
                        Math.toRadians(-90)
                )
                .lineToY(47)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(136)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(10, 12, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(47)
                .lineToY(20)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(136)), Math.toRadians(0))
                        .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(136)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(36, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(47)
                .lineToY(20)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(136)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(90)), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
        }