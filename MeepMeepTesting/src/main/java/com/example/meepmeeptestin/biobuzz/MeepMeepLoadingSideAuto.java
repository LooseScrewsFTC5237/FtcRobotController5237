package com.example.meepmeeptestin.biobuzz;

import static com.example.meepmeeptestin.Constants.GARDEN_BEGIN_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_FLOWER_POSE;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_APPROACH_HEADING;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_APPROACH_VECTOR;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_HEADING;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_VECTOR;
import static com.example.meepmeeptestin.Constants.INTAKE_LOADING_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_BEGIN_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_FLOWER_APPROACH_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_FLOWER_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_PARK_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_SHOOT_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_SHOOT_POSE_2;
import static com.example.meepmeeptestin.Constants.PARK_POSE;
import static com.example.meepmeeptestin.Constants.SLOW_SLOW_VEL;
import static com.example.meepmeeptestin.Constants.SLOW_VEL;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM_LEFT;
import static com.example.meepmeeptestin.Constants.TO_LEFT;
import static com.example.meepmeeptestin.Constants.TO_RIGHT;
import static com.example.meepmeeptestin.Constants.TO_TOP;
import static com.example.meepmeeptestin.Constants.TO_TOP_LEFT;
import static com.example.meepmeeptestin.Constants.TO_TOP_RIGHT;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepLoadingSideAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(LOADING_BEGIN_POSE)

                // region block 1

                .waitSeconds(10) // this is a placeholder for camera
                .waitSeconds(2) // Shoot
                .setTangent(TO_TOP)
                .splineToLinearHeading(LOADING_FLOWER_APPROACH_POSE, TO_BOTTOM)
                .splineToLinearHeading(LOADING_FLOWER_POSE, TO_BOTTOM) // Depollinater down
                .waitSeconds(0.5)

                // endregion


                // region block 3

                .setTangent(TO_TOP)
                .splineToLinearHeading(LOADING_FLOWER_APPROACH_POSE, TO_BOTTOM)

                // endregion

                .setTangent(TO_LEFT)
                .splineToLinearHeading(LOADING_SHOOT_POSE_2, TO_BOTTOM)
                .waitSeconds(2) // shoot

                // region block 4


                // region block 5

                .setTangent(TO_TOP)
                .splineToLinearHeading(LOADING_PARK_POSE,TO_RIGHT)

                // endregion



                .build());

        Image img = null;
        try {
            // Provide the absolute path to your image file
            img = ImageIO.read(new File("MeepMeepTesting\\biobuzz.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}