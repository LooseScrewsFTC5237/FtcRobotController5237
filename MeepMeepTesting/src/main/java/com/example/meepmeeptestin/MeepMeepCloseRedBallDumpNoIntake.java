package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseRedBallDumpNoIntake {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d dumpPose1 = new Pose2d(7, 57, Math.toRadians(135));
        double dumpTangent1 = Math.toRadians(0);
        Pose2d shootPose = new Pose2d(-16, 16, Math.toRadians(135));
        Pose2d dumpPose2 = new Pose2d(25, 60, Math.toRadians(135));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 37, Math.toRadians(0)))

                //1st Shot
                .strafeToLinearHeading(new Vector2d(-16,16),Math.toRadians(135))
                .waitSeconds(0.75)


                //Intake Goal Side Line
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(new Pose2d(-13, 33, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, 53, Math.toRadians(90)), Math.toRadians(90))

                //Dump No Intake
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-1, 53, Math.toRadians(180)), Math.toRadians(90))

                //2nd Shot
                .strafeToLinearHeading(new Vector2d(-16,16),Math.toRadians(135))
                .waitSeconds(0.75)

                //Intake Middle Line
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(14, 35,Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(14, 52,Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0)

                //3rd Shot
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(0.75)

                //Dump'N Intake
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(dumpPose1,dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //4th Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(shootPose, Math.toRadians(270))
                .waitSeconds(0.75)

                //Dump'N Intake2
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(dumpPose1, dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //5th Shot
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(0.75)

                //Dump'N Intake3
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(dumpPose1, dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //6th Shot
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(0.75)

                //Park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-16, 37, Math.toRadians(90)), Math.toRadians(90))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
