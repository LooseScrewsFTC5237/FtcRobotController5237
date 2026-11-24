package com.example.meepmeeptestin.biobuzz;

import static com.example.meepmeeptestin.Constants.GARDEN_BEGIN_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_FIRST_SHOOT_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_FLOWER_APPROACH_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_FLOWER_POSE;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_APPROACH_VECTOR_2;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_HEADING;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_VECTOR;
import static com.example.meepmeeptestin.Constants.PARK_POSE;
import static com.example.meepmeeptestin.Constants.SLOW_VEL;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM_LEFT;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM_RIGHT;
import static com.example.meepmeeptestin.Constants.TO_LEFT;
import static com.example.meepmeeptestin.Constants.TO_RIGHT;
import static com.example.meepmeeptestin.Constants.TO_TOP;
import static com.example.meepmeeptestin.Constants.TO_TOP_RIGHT;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepGardenDoubleShoot {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(GARDEN_BEGIN_POSE)

                //region block 1
                .waitSeconds(2)
                .splineToLinearHeading(GARDEN_FIRST_SHOOT_POSE, TO_TOP)
                .waitSeconds(4)// shoot
                .strafeToSplineHeading(INTAKE_GARDEN_APPROACH_VECTOR_2, INTAKE_GARDEN_HEADING) // intake
                .strafeToSplineHeading(INTAKE_GARDEN_VECTOR, INTAKE_GARDEN_HEADING, SLOW_VEL) // intake
                // endregion

                // region block 2
                .setTangent(TO_LEFT)
                .splineToLinearHeading(GARDEN_FIRST_SHOOT_POSE, TO_LEFT)
                .waitSeconds(4)// shoot
                //endregion

                // region block 3
                .setTangent(TO_RIGHT)
                .splineToLinearHeading(GARDEN_FLOWER_APPROACH_POSE, TO_BOTTOM)
                .setTangent(TO_RIGHT)
                .splineToLinearHeading(GARDEN_FLOWER_POSE, TO_RIGHT, SLOW_VEL)
                .waitSeconds(.5)
                // endregion

                // region block park
                .setTangent(TO_BOTTOM_LEFT)
                .splineToLinearHeading(PARK_POSE, TO_BOTTOM_RIGHT) //park
                .waitSeconds(1)
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