package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class FarAuto extends LinearOpMode {

    @Autonomous()
    public static class RedFarAuto extends FarAuto {
        public RedFarAuto() {
            super(new Pose2d(60, 30, Math.toRadians(180)), new IdentityPoseMap());
        }
    }

    @Autonomous()
    public static class BlueFarAuto extends FarAuto {
        public BlueFarAuto() {
            super(
                    new Pose2d(60, -30, Math.toRadians(180)),
                    pose -> new Pose2dDual<>(
                           new Vector2dDual<>(
                                   pose.position.x,
                                   pose.position.y.unaryMinus()
                           ),
                            pose.heading.inverse()
                    )
            );
        }
    }

    protected MecanumDrive drive;

    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    Hood hood = new Hood();

    public static boolean UPDATE_FLYWHEEL_PID = false;
    public static double FLYWHEEL_P = 70;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 5;
    public static double FLYWHEEL_F = 21;
    PoseMap poseMap;
    Pose2d startingPose;

    public FarAuto(Pose2d startingPose, PoseMap poseMap) {
        this.poseMap = poseMap;
        this.startingPose = startingPose;
    }

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
        double shooterSpeed = 1125;
        double currentShooterVelocity = shooter.getVelocity();

        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!UPDATE_FLYWHEEL_PID) {
            FLYWHEEL_P = c.p;
            FLYWHEEL_I = c.i;
            FLYWHEEL_D = c.d;
            FLYWHEEL_F = c.f;
        }
        pidTuner();

        Pose2d beginPose = startingPose;// new Pose2d(60, 30, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        pidTuner();

        Actions.runBlocking(
                drive.actionBuilder(beginPose, poseMap)
                        .stopAndAdd(() -> {
                            // These motors turn on the moment 'Start' is pressed
                            intake.setPower(1.0); // Assuming you want the intake on too
                            feeder.setPower(0.0);
                            shooter.setVelocity(shooterSpeed);
                            shooter2.setVelocity(shooterSpeed);
                        })

                        .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(155.5)), Math.toRadians(0))

                        .waitSeconds(2)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(0.11)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        .waitSeconds(0.4)
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.4)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.4)

                        .splineToLinearHeading(new Pose2d(35, 20, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(0)
                        .lineToY(
                                55, // Target Y-coordinate
                                new TranslationalVelConstraint(15), // **New Max Velocity (e.g., 15 in/s)**
                                null // Use default acceleration constraint
                        )
                        .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(155.5)), Math.toRadians(0))

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        .waitSeconds(0.4)
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.4)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.4)


                        .splineToLinearHeading(new Pose2d(40, 60, Math.toRadians(20)), Math.toRadians(0))
                        .waitSeconds(0)
                        .lineToX(
                                62, // Target Y-coordinate
                                new TranslationalVelConstraint(15
                                ),
                                null // Use default acceleration constraint
                        )
                        .waitSeconds(0)
                        .splineToLinearHeading(new Pose2d(38, 50, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(0)
                        .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(155.5)), Math.toRadians(0))

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        .waitSeconds(0.4)
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.4)

                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(0.111)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .waitSeconds(0.4)


                        .splineToLinearHeading(new Pose2d(40, 20, Math.toRadians(75)), Math.toRadians(0))
                        .build());




        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);


    }
    private void pidTuner() {
        if (UPDATE_FLYWHEEL_PID) {
            PIDFCoefficients c = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            UPDATE_FLYWHEEL_PID = false;
        }
        telemetry.addData("Flywheel P", FLYWHEEL_P);
        telemetry.addData("Flywheel I", FLYWHEEL_I);
        telemetry.addData("Flywheel D", FLYWHEEL_D);
        telemetry.addData("Flywheel F", FLYWHEEL_F);
    }

}
