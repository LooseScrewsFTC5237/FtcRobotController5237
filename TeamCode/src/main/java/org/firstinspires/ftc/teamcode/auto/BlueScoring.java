package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;



@Config


@Autonomous
public class BlueScoring extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    PoseMap poseMap;


    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(14, 60.5, Math.toRadians(270));


        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap).stopAndAdd(() -> drive.setDrivePowers()).build()) // TODO:call instances of subsystems


                )



        );

    }


}
