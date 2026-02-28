package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@Config
public class Close18BallDump extends LinearOpMode {

    boolean isRed = false;
    @Autonomous()
    public static class RedClose18BallDump extends Close18BallDump {
        public RedClose18BallDump() {
            super(new Pose2d(-60, 37, Math.toRadians(0)), new IdentityPoseMap());
            isRed = true;
        }
    }

    @Autonomous()
    public static class BlueClose18BallDump extends Close18BallDump {
        public BlueClose18BallDump() {
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
    private Limelight3A limelight;
    private double shooterSpeed = 1170;

    public static boolean
            UPDATE_FLYWHEEL_PID = true;

    public static double Redoffset = 0;
    public static double Blueoffset = -2.5;
    public static double BEARING_THRESHOLD = 0.25; // Angled towards the tag (degrees)
    public static double TURN_GAIN   =  0.04  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN  = 0.3;

    public static double feederOnTime = 1;
    PoseMap poseMap;
    Pose2d startingPose;

    Action autoAimAction() {
        return new Action() {

            long startTimeMillis = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                // Do your auto aim code here
                double turn = 0;

                LLResult llResult = limelight.getLatestResult();
                double headingError = llResult.getTx();
                double headingOffset = isRed ? Redoffset : Blueoffset;
                double offsetError = headingError + headingOffset;
                if (Math.abs(offsetError) < BEARING_THRESHOLD) {
                    return false;
                }
                turn = Range.clip((offsetError + (Math.signum(offsetError) * TURN_STATIC)) * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn * -1));

                return (now - startTimeMillis < 2_000);
            }
        };
    }

    Action shooterCheckAction() {
        return new Action() {

            long startTimeMillis = 0;
            double currentShooterVelocity;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                currentShooterVelocity = shooter.getVelocity();
                telemetryPacket.put("Current Velocity", currentShooterVelocity);
                telemetryPacket.put("Target Velocity", shooterSpeed);
                if (Math.abs(currentShooterVelocity - shooterSpeed) < 50  ) {
                    return false;
                }

                return (now - startTimeMillis < 1_000);

            }
        };
    }

    public Close18BallDump(Pose2d startingPose, PoseMap poseMap) {
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
        hood.setServoPos(0.0675);
        double shooterSpeed = 1170;
        double currentShooterVelocity = shooter.getVelocity();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        Pose2d dumpPose = new Pose2d(12.5, 65, Math.toRadians(120));
        double dumpTangent = Math.toRadians(90);
        Pose2d shootPose = new Pose2d(-16, 16, Math.toRadians(135));

        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!UPDATE_FLYWHEEL_PID) {
            TwentyTwentyFiveJava.FLYWHEEL_P = c.p;
         TwentyTwentyFiveJava.FLYWHEEL_I = c.i;
            TwentyTwentyFiveJava.FLYWHEEL_D = c.d;
            TwentyTwentyFiveJava.FLYWHEEL_F = c.f;
        }
        pidTuner();

        Pose2d beginPose = startingPose;// new Pose2d(60, 30, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);
        limelight.start();
        telemetry.addData("Current Velocity", 0);
        telemetry.addData("Target Velocity", 0);
        telemetry.addData("Feeder Speed", 0);
        waitForStart();
        pidTuner();

        Actions.runBlocking(
                drive.actionBuilder(beginPose, poseMap)
                        // Turn on motors
                        .stopAndAdd(() -> {
                            intake.setPower(0.0);
                            feeder.setPower(0.0);
                            shooter.setVelocity(shooterSpeed);
                            shooter2.setVelocity(shooterSpeed);
                        })
                        //First Shot
                        .splineToLinearHeading(shootPose, Math.toRadians(0))
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        //Intake Middle Line
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(12, 16,Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(12, 49,Math.toRadians(90)), Math.toRadians(90))
                        .stopAndAdd(() -> intake.setPower(0))
                        //Second Shot
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(shootPose, Math.toRadians(180))
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        //Dump'N Intake
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(dumpPose, dumpTangent)
                        .waitSeconds(1)
                        .stopAndAdd(() -> intake.setPower(0))
                        //Third Shot
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(shootPose, Math.toRadians(180))
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        //Intake Goal Side Line
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-10, 16, Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-10, 49, Math.toRadians(90)), Math.toRadians(90))
                        .stopAndAdd(() -> intake.setPower(0))
                        //Fourth Shot
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(shootPose, Math.toRadians(90))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .stopAndAdd(() -> intake.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        //Dump'N Intake2
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(dumpPose, dumpTangent)
                        .waitSeconds(1)
                        .stopAndAdd(() -> intake.setPower(0))
                        //Fifth Shot
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(shootPose, Math.toRadians(180))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .stopAndAdd(() -> intake.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        //Dump'N Intake3
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(dumpPose, dumpTangent)
                        .waitSeconds(1)
                        .stopAndAdd(() -> intake.setPower(0))
                        //Sixth Shot
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(shootPose, Math.toRadians(180))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .stopAndAdd(() -> intake.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        .waitSeconds(1)
                        //Park
                        .stopAndAdd(() -> intake.setPower(0))
                        .splineToLinearHeading(new Pose2d(-16, 35, Math.toRadians(90)), Math.toRadians(90))
                        .build());

        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
    }
    private void pidTuner() {
        if (UPDATE_FLYWHEEL_PID) {
            PIDFCoefficients c = new PIDFCoefficients(TwentyTwentyFiveJava.FLYWHEEL_P,TwentyTwentyFiveJava.FLYWHEEL_I, TwentyTwentyFiveJava.FLYWHEEL_D,TwentyTwentyFiveJava.FLYWHEEL_F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            UPDATE_FLYWHEEL_PID = false;
        }
        telemetry.addData("Flywheel P", TwentyTwentyFiveJava.FLYWHEEL_P);
        telemetry.addData("Flywheel I",TwentyTwentyFiveJava.FLYWHEEL_I);
        telemetry.addData("Flywheel D", TwentyTwentyFiveJava.FLYWHEEL_D);
        telemetry.addData("Flywheel F", TwentyTwentyFiveJava.FLYWHEEL_F);
    }

}
