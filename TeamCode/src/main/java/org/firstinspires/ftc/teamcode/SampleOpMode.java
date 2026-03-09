package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "SampleRedAuto")
public class SampleOpMode extends LinearOpMode {





    @Override
    public void runOpMode()   throws InterruptedException {
        // create starting pose
        Pose2d  beginPose = new Pose2d(new Vector2d(-70,46 ), Math.toRadians(0));

        // create RR drive obj
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        //Creating Autonamous path
        Action path = drive.actionBuilder(beginPose)
                .build();

    }

}


