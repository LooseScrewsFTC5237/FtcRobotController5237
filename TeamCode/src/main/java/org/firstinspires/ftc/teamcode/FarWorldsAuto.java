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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@Config
public class FarWorldsAuto extends LinearOpMode {

    boolean isRed = false;
    @Autonomous()
    public static class RedFarWorldsAuto extends FarWorldsAuto {
        public RedFarWorldsAuto() {
            super(new Pose2d(60, 30, Math.toRadians(180)), new IdentityPoseMap());
            isRed = true;
        }
    }

    @Autonomous()
    public static class BlueFarWorldsAuto extends FarWorldsAuto {
        public BlueFarWorldsAuto() {
            super(
                    new Pose2d(60, -30, Math.toRadians(180)
                    ),
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
    private DigitalChannel laserInput;
    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    Hood hood = new Hood();
    private Limelight3A limelight;
    private double shooterSpeed = 1170;

    public static boolean
            UPDATE_FLYWHEEL_PID = true;
    public static boolean artifactPresent = false;

    public static int artifactCounter = 0;
    public static double Redoffset = 0;
    public static double Blueoffset = -2.5;
    public static double BEARING_THRESHOLD = 0.25; // Angled towards the tag (degrees)
    public static double TURN_GAIN   =  0.04  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN  = 0.3;

    public static double feederOnTime = 0.6;
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

    public FarWorldsAuto(Pose2d startingPose, PoseMap poseMap) {
        this.poseMap = poseMap;
        this.startingPose = startingPose;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        laserInput = hardwareMap.get(DigitalChannel.class, "LaserArtifactDetector");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
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
        Pose2d dumpPose1 = new Pose2d(7, 57, Math.toRadians(135));
        double dumpTangent1 = Math.toRadians(90);
        Pose2d shootPose = new Pose2d(-16, 16, Math.toRadians(135));
        Pose2d dumpPose2 = new Pose2d(25, 60, Math.toRadians(135));
        double dumpTangent2 = Math.toRadians(90);

//        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        if (!UPDATE_FLYWHEEL_PID) {
//            TwentyTwentyFiveJava.FLYWHEEL_P = c.p;
//         TwentyTwentyFiveJava.FLYWHEEL_I = c.i;
//            TwentyTwentyFiveJava.FLYWHEEL_D = c.d;
//            TwentyTwentyFiveJava.FLYWHEEL_F = c.f;
//        }
//        pidTuner();
        PIDFCoefficients pidfNew = new PIDFCoefficients (140, 0, 0, 12.86);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfNew);

        Pose2d beginPose = startingPose;// new Pose2d(60, 30, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);
        limelight.start();
        telemetry.addData("Current Velocity", 0);
        telemetry.addData("Target Velocity", 0);
        telemetry.addData("Feeder Speed", 0);
        waitForStart();
        PIDFCoefficients currentPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel P", currentPIDF.p);
        telemetry.addData("Flywheel F", currentPIDF.f);

        Actions.runBlocking(
                drive.actionBuilder(beginPose, poseMap)

                        // Turn on motors
                        .stopAndAdd(() -> {
                            intake.setPower(0.0);
                            feeder.setPower(0.0);
                            shooter.setVelocity(shooterSpeed);
                            shooter2.setVelocity(shooterSpeed);
                        })

                        // Prep First Three Shots
                        .strafeToLinearHeading(new Vector2d(60, 28), Math.toRadians(90))
                        .stopAndAdd(autoAimAction())
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))

                        // First Intake
                        .lineToY(60)
                        .stopAndAdd(() -> intake.setPower(0))
                        .waitSeconds(0.5)
                        .lineToY(28)
                        // Prep Second Three Shots
                        .strafeToLinearHeading(new Vector2d(60, 28), Math.toRadians(90))
                        .stopAndAdd(autoAimAction())
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))

                        // Second Intake
                        //I stopped here
                        .waitSeconds(0.6)
//                        .turnTo(Math.toRadians(-280))
//                        .strafeToLinearHeading(new Vector2d(60, 34), Math.toRadians(-280))
                        .splineToSplineHeading(new Pose2d(60, 40,Math.toRadians(90)), Math.toRadians(270))
//                        .lineToX(72 /*, new TranslationalVelConstraint(21), null */)
                        .strafeToLinearHeading(new Vector2d(60,67),90)
//                        .lineToX(67 /*, new TranslationalVelConstraint(21), null */)
                        .stopAndAdd(() -> intake.setPower(0)) // Turn off intake

                        // Pre Third Three Shots
                        .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(150.5))
                        .stopAndAdd(autoAimAction())
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        // Third Intake
                        .waitSeconds(0.6)
                        .turnTo(Math.toRadians(-280))
                        .strafeToLinearHeading(new Vector2d(60, 34), Math.toRadians(-280))
                        .lineToX(72 /*, new TranslationalVelConstraint(21), null */)
                        .lineToX(67 /*, new TranslationalVelConstraint(21), null */)
                        .stopAndAdd(() -> intake.setPower(0)) // Turn off intake
                        // Pre 4th shots
                        .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(166))
                        .stopAndAdd(autoAimAction())
                        .stopAndAdd(() -> intake.setPower(1))
                        .stopAndAdd(() -> feeder.setPower(1))
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0))
                        .stopAndAdd(() -> intake.setPower(0)) // Turn off intake
                        //.stopAndAdd(shooterCheckAction())
                        //.stopAndAdd(() -> intake.setPower(1)) // Turn intake back on

                        // Take Third Three Shots
                        //  .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        //  .waitSeconds(feederOnTime)
                        //  .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        //  .stopAndAdd(shooterCheckAction())
                        //  .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        //  .waitSeconds(feederOnTime)
                        // .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        //  .stopAndAdd(shooterCheckAction())
                        //  .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        //  .waitSeconds(feederOnTime)
                        //  .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        // Park
                        .strafeToLinearHeading(new Vector2d(40, 20), Math.toRadians(75))

                        .build());

        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
        // }
        // private void pidTuner() {
        //   if (UPDATE_FLYWHEEL_PID) {
        //     PIDFCoefficients c = new PIDFCoefficients(TwentyTwentyFiveJava.FLYWHEEL_P, TwentyTwentyFiveJava.FLYWHEEL_I, TwentyTwentyFiveJava.FLYWHEEL_D, TwentyTwentyFiveJava.FLYWHEEL_F);
        //    shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        //    shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        //  UPDATE_FLYWHEEL_PID = false;
        //  }
        //telemetry.addData("Flywheel P", TwentyTwentyFiveJava.FLYWHEEL_P);
        //  telemetry.addData("Flywheel I", TwentyTwentyFiveJava.FLYWHEEL_I);
        // telemetry.addData("Flywheel D", TwentyTwentyFiveJava.FLYWHEEL_D);
        // telemetry.addData("Flywheel F", TwentyTwentyFiveJava.FLYWHEEL_F);

    }
}
