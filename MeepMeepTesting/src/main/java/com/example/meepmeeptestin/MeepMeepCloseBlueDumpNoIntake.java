package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseBlueDumpNoIntake {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(850);

        Pose2d dumpPose1 = new Pose2d(-5, -60, Math.toRadians(165));
        double dumpTangent1 = Math.toRadians(270);
        Pose2d shootPose = new Pose2d(-16, -16, Math.toRadians(217));
        Pose2d dumpPose2 = new Pose2d(5, -60, Math.toRadians(0));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -37, Math.toRadians(0)))

                //First Shot
                .strafeToLinearHeading(new Vector2d(-16, -16),Math.toRadians(217))
                .waitSeconds(2)

                //Intake Goal Side Line
                .setTangent(Math.toRadians(260))
                .splineToSplineHeading(new Pose2d(-10, -33, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-10, -53, Math.toRadians(270)), Math.toRadians(270))

                //Dump 1
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dumpPose1,dumpTangent1)

                //Second Shot
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-16,-16), Math.toRadians(217))
                .waitSeconds(1)

                //Intake Middle Line
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(17, -33, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(17, -52, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0)

                //Dump 2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dumpPose2,dumpTangent1)

                //Third Shot
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-16,-16), Math.toRadians(217))
                .waitSeconds(1)

                //Dump 3
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(dumpPose2,dumpTangent1)
                .waitSeconds(1)

                // Intake Dumped Artifacts
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20 ,-65 ,Math.toRadians(300)), Math.toRadians(0))

                //Fourth Shot
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-16,-16), Math.toRadians(217))
                .waitSeconds(1)

                //Park
                .strafeToLinearHeading(new Vector2d(-16, -37), Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
