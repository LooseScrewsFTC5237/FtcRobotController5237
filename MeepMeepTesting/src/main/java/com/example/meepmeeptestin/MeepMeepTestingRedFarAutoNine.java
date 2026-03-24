package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingRedFarAutoNine {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 30, Math.toRadians(180)))
                // 1st Shot
                .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(150.5))
                // 1st Intake
                .strafeToLinearHeading(new Vector2d(35, 20), Math.toRadians(90))
                .waitSeconds(0)
                .lineToY(50 /* , new TranslationalVelConstraint(15), null */)
                // 2nd Shot
                .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(150.5))
                .waitSeconds(0.6)
                // 2nd Intake
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(new Pose2d(60, 40,Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60,67),90)
                .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(150.5))
                .waitSeconds(0.6)
                .turnTo(Math.toRadians(-280))
                .strafeToLinearHeading(new Vector2d(60, 34), Math.toRadians(-280))
                .lineToX(72 /*, new TranslationalVelConstraint(21), null */)
                .lineToX(67 /*, new TranslationalVelConstraint(21), null */)
                .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(166))
                .strafeToLinearHeading(new Vector2d(40, 20), Math.toRadians(75))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}