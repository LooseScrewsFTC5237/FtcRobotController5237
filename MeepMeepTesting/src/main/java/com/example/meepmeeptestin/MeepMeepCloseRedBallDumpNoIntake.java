package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseRedBallDumpNoIntake {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d dumpPose1 = new Pose2d(0, 55, Math.toRadians(190));
        double dumpTangent1 = Math.toRadians(90);
        Pose2d shootPose = new Pose2d(-16, 16, Math.toRadians(135));
        Pose2d dumpPose2 = new Pose2d(5, 55, Math.toRadians(350));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 37, Math.toRadians(0)))

                //First Shot
                .strafeToLinearHeading(new Vector2d(-16, 16),Math.toRadians(135))
                .waitSeconds(2)

                //Intake Goal Side Line
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(-10, 27, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10, 53, Math.toRadians(90)), Math.toRadians(90))

                //Dump 1
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(dumpPose1,dumpTangent1)

                //Second Shot
                .setTangent(Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-16,16), Math.toRadians(135))
                .waitSeconds(1)

                //Intake Middle Line
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, 27, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(22, 52, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0)

                //Dump 2
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(dumpPose2,dumpTangent1)

                //Third Shot
                .setTangent(Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-16,16), Math.toRadians(135))
                .waitSeconds(1)

                //Dump'N Intake
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(new Pose2d(5, 58, Math.toRadians(100)), dumpTangent1)
                .splineToLinearHeading(new Pose2d(25,60 ,Math.toRadians(135)), Math.toRadians(0))
                .waitSeconds(1)


                //Fourth Shot
                .setTangent(Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-16,16), Math.toRadians(135))
                .waitSeconds(1)

                //Park
                .strafeToLinearHeading(new Vector2d(-16, 37), Math.toRadians(90))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
