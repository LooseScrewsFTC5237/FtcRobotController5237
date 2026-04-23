package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepCloseBlueDump {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d dumpPose1 = new Pose2d(5, -68, Math.toRadians(225));
        double dumpTangent1 = Math.toRadians(270);
        Pose2d shootPose = new Pose2d(-16, -16, Math.toRadians(217));
        Pose2d dumpPose2 = new Pose2d(20, -63, Math.toRadians(225));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -37, Math.toRadians(0)))

                //First Shot
                .splineToLinearHeading(shootPose, Math.toRadians(0))
                //.strafeToLinearHeading(new Vector2d(-16, 16))
                .waitSeconds(2)

                //Intake Middle Line
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(17, -33, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(17, -52, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0)

                //Second Shot
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Dump'N Intake
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12, -20, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(dumpPose1,dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //Third Shot
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, -20,Math.toRadians(270)), Math.toRadians(180))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Intake Goal Side Line
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-10, -33, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-10, -53, Math.toRadians(270)), Math.toRadians(270))

                //Fourth Shot
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Dump'N Intake2
                .setTangent(Math.toRadians(280))
                .splineToSplineHeading(dumpPose1, dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //Fifth Shot
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, -20,Math.toRadians(270)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-16, -16, Math.toRadians(216)), Math.toRadians(180))
                .waitSeconds(1)

                //Park
                .splineToLinearHeading(new Pose2d(-16, -37, Math.toRadians(270)), Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
