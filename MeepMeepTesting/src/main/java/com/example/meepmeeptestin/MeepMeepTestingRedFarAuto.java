package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingRedFarAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 14.5, Math.toRadians(180)))

                // 1st Shot
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(154.5))
                .waitSeconds(1)

                // 1st Intake
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(31, 35,Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(31, 53, Math.toRadians(95)), Math.toRadians(90))

                // 2nd Shot
                .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(158.5)), Math.toRadians(270))
                .waitSeconds(0.6)

               // 2nd Intake
                .strafeToLinearHeading(new Vector2d(48, 63), Math.toRadians(355))
                .strafeToLinearHeading(new Vector2d(60, 64), Math.toRadians(355))
                .waitSeconds(0.6)

                // 3rd Shot
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(158.5))
                .waitSeconds(0.6)

                // 3rd Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(70, 47, Math.toRadians(100)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(70, 65, Math.toRadians(100)), Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(50, 70, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.6)

                // 4th Shot
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(158.5))
                .waitSeconds(0.6)

                // 4th Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(70, 47, Math.toRadians(100)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(70, 65, Math.toRadians(100)), Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(50, 70, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.6)

                // 5th Shot
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(158.5))
                .waitSeconds(0.6)

                // 5th Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(70, 47, Math.toRadians(100)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(70, 65, Math.toRadians(100)), Math.toRadians(100))
                .waitSeconds(0.6)

//                // 6th Shot
//                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(158.5))
//                .waitSeconds(0.6)
//
//                // 6th Intake
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(48, 40,Math.toRadians(135)), Math.toRadians(135))
//                .lineToY(55)
//                .waitSeconds(0.6)

                // Park
                .strafeToLinearHeading(new Vector2d(60, 40), Math.toRadians(90))


                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}