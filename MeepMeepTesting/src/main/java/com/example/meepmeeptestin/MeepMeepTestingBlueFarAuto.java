package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBlueFarAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -14.5, Math.toRadians(180)))

                // 1st Shot
                .strafeToLinearHeading(new Vector2d(56, -23), Math.toRadians(199.5))
                .waitSeconds(1)

                // 1st Intake
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(35, -35,Math.toRadians(270)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(35,-55),Math.toRadians(270))

                // 2nd Shot
                .splineToSplineHeading(new Pose2d(56, -23, Math.toRadians(199.5)), Math.toRadians(90))
                .waitSeconds(0.6)

               // 2nd Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(60, -40,Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-58)
                .waitSeconds(0.6)

                // 3rd Shot
                .strafeToLinearHeading(new Vector2d(56, -23), Math.toRadians(209.5))
                .waitSeconds(0.6)

                // 3rd Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(60, -40,Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-58)
                .waitSeconds(0.6)

                // 4th Shot
                .strafeToLinearHeading(new Vector2d(56, -23), Math.toRadians(194))
                .waitSeconds(0.6)

                // 4th Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, -40,Math.toRadians(225)), Math.toRadians(225))
                .lineToY(-58)
                .waitSeconds(0.6)

                // 5th Shot
                .strafeToLinearHeading(new Vector2d(56, -23), Math.toRadians(194))
                .waitSeconds(0.6)

                // 5th Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, -40,Math.toRadians(225)), Math.toRadians(225))
                .lineToY(-58)
                .waitSeconds(0.6)

                // 6th Shot
                .strafeToLinearHeading(new Vector2d(56, -23), Math.toRadians(194))
                .waitSeconds(0.6)

                // 6th Intake
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, -40,Math.toRadians(255)), Math.toRadians(225))
                .lineToY(-58)
                .waitSeconds(0.6)

                // Park
                .strafeToLinearHeading(new Vector2d(60, -40), Math.toRadians(270))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}