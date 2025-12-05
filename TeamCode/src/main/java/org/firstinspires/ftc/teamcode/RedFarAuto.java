package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//  DcMotorEx shooter;
// DcMotorEx shooter2;
@Autonomous()
public class RedFarAuto extends LinearOpMode {

    protected MecanumDrive drive;

    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    Hood hood = new Hood();

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(DcMotor.class, "Shooter Feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        hood.init(hardwareMap);
        hood.setServoPos(0.42);
        double shooterSpeed = 2000;
        double  currentShooterVelocity = shooter.getVelocity();



        Pose2d beginPose = new Pose2d(60, 30, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(() -> {
                            // These motors turn on the moment 'Start' is pressed
                            intake.setPower(1.0); // Assuming you want the intake on too
                            feeder.setPower(1.0);
                            shooter.setVelocity(shooterSpeed);
                            shooter2.setVelocity(shooterSpeed);
                        })

                        .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(162)), Math.toRadians(0))
                        /*
                        .stopAndAdd(() -> {

                            for ( ) {
                                if (currentShooterVelocity > 0.9 * shooterSpeed) &&
                                (currentShooterVelocity < 1.1 * shooterSpeed) {
                                    feeder.setPower(1.0);
                                } else{
                                    feeder.setPower(0);
                                }
                            }

                            }
                        })
                         */
                        .waitSeconds(0.15)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        .waitSeconds(0.25)
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.25)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.25)

                        .splineToLinearHeading(new Pose2d(35, 20, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(0)
                        .lineToY(50)
                        .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(162)), Math.toRadians(0))

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        .waitSeconds(0.25)
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.25)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.25)


                        .splineToLinearHeading(new Pose2d(40, 60, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(0)
                        .lineToX(58)
                        .waitSeconds(0)
                        .splineToLinearHeading(new Pose2d(38, 50, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(0)
                        .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(162)), Math.toRadians(0))

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        .waitSeconds(0.25)
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.25)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(0.15)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.25)


                        .splineToLinearHeading(new Pose2d(40, 20, Math.toRadians(90)), Math.toRadians(0))
                        .build());




        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
    }
}
//  //
//                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
//                        .waitSeconds(0.15)
//                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
//                        .waitSeconds(0.25)
//
//                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
//                        .waitSeconds(0.15)
//                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
//                        .waitSeconds(0.25)
//
//                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
//                        .waitSeconds(0.15)
//                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
//                        .waitSeconds(0.25)
//                        //

