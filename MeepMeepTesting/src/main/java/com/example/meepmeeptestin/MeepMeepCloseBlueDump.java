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

        Pose2d dumpPose1 = new Pose2d(7, -57, Math.toRadians(225));
        double dumpTangent1 = Math.toRadians(0);
        Pose2d shootPose = new Pose2d(-16, -16, Math.toRadians(225));
        Pose2d dumpPose2 = new Pose2d(25, -60, Math.toRadians(225));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -37, Math.toRadians(0)))

                //1st Shot
                .strafeToLinearHeading(new Vector2d(-16,-16),Math.toRadians(225))
                .waitSeconds(2)


                //Intake Middle Line
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(14, -35,Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(14, -52,Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0)

                //2nd Shot
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Dump'N Intake
                .setTangent(Math.toRadians(280))
                .splineToSplineHeading(dumpPose1,dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //3rd Shot
                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Intake Goal Side Line
                .setTangent(Math.toRadians(280))
                .splineToSplineHeading(new Pose2d(-13, -33, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-12, -53, Math.toRadians(270)), Math.toRadians(270))

                //4th Shot
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(90))
                .waitSeconds(1)

                //Dump'N Intake2
                .setTangent(Math.toRadians(280))
                .splineToSplineHeading(dumpPose1, dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //5th Shot
                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Dump'N Intake3
                .setTangent(Math.toRadians(280))
                .splineToSplineHeading(dumpPose1, dumpTangent1)
                .splineToLinearHeading(dumpPose2, dumpTangent2)
                .waitSeconds(1)

                //6th Shot
                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(1)

                //Park
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-16, -37, Math.toRadians(270)), Math.toRadians(270))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
