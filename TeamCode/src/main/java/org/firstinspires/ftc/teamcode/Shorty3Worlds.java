/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@Config
@TeleOp(name = "Short3WorldsJava", group = "Robot")
public class Shorty3Worlds extends OpMode {

    //region Public Static Variables
    public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    // This declares the four motors needed
    int lift_height = 0;
    public static int fastShooterSpeed = 1500;
    public static int shooterSpeed = 1200;
    public static int slowShooterSpeed = 960;
    public static double shooterSpeedTolerance = 40;
    public static int targetVelocity = 0;
    public static double DRIVE_SPEED = 0.8;
    public static byte shooterMode = 0;
    public static double BEARING_THRESHOLD = 1; // Angled towards the tag (degrees)
    public static double TURN_GAIN   =  0.0025  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)//
    private AprilTagProcessor aprilTag;
    private Limelight3A limelight;
    private static final int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    public static int  DECIMATION = 3;
    public static float targetRPM;
    public static double rpmDistanceMultiplier = 7.61765;
    public static double axisOffsetRPM = 776.59769;
    public static double Blueoffset = -2.5;
    public static double Redoffset = 0;
    public static double hoodDistanceMultiplier = -0.00188;
    public static double getAxisOffsetHood = 0.535;
    public static boolean UPDATE_FLYWHEEL_PID = false;
    public static double FLYWHEEL_P = 130;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 13.55;
    public static double closeHoodAngle = 0;
    public static double mediumHoodAngle = 0.06;
    public static double farHoodAngle = 0.12;
    public static double FarRPMBump = 60;
    public static double FarHoodBump = -0.04;
    public static double headingOffset = 0;

    public static int artifactCounter = 0;
    public static boolean artifactPresent = false;

    public static boolean velocityCheck;

    public static boolean velocityCheck2;

    //endregion

    //region Hardware Variables
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    CRServo turret;
    DigitalChannel limitSwitchLeft;
    DigitalChannel limitSwitchRight;
    private DigitalChannel laserInput;
    // endregion

    private final int READ_PERIOD = 1;

    public static double p = 0.01, i = 0, d = 0.0001;

    Hood hood = new Hood();

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    // region VisionPortal Code
    private VisionPortal visionPortal;

    // Create the vision portal by using a builder.
    VisionPortal.Builder builder = new VisionPortal.Builder();
    //endregion

    @Override
    public void init() {
        // region HardwareMaps
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(DcMotor.class, "Shooter Feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        turret = hardwareMap.get(CRServo.class,"Turret");
        limitSwitchLeft = hardwareMap.get(DigitalChannel.class, "LimitLeft");
        limitSwitchRight = hardwareMap.get(DigitalChannel.class, "LimitRight");
        laserInput = hardwareMap.get(DigitalChannel.class, "LaserArtifactDetector");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        // endregion


        // Initialize the Apriltag Detection process
        //initAprilTag();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        //region Limelight
        //FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
        //endregion

        //region Reverse Motors
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        //endregion

        hood.init(hardwareMap);
        hood.setServoPos(0);

        //region Motor Encoder Settings
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //endregion

        //region PID Coefficients
        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!UPDATE_FLYWHEEL_PID) {
            FLYWHEEL_P = c.p;
            FLYWHEEL_I = c.i;
            FLYWHEEL_D = c.d;
            FLYWHEEL_F = c.f;
        }
        pidTuner();
       // PIDFCoefficients pidfNew = new PIDFCoefficients (140, 0, 0, 12.86);
       // shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
      //  shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfNew);
        //endregion

        //region IMU Orientations
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //endregion
    }

    @Override
    public void loop() {
        boolean artifactDetected = laserInput.getState();
    pidTuner();

        //region PIDFCoefficients
//        PIDFCoefficients currentPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("Flywheel P", currentPIDF.p);
//        telemetry.addData("Flywheel F", currentPIDF.f);
        //endregion

        double  currentShooterVelocity = shooter.getVelocity();
        // telemetry.addLine("Press X to reset Yaw");
        // telemetry.addLine("Hold left bumper to drive in robot relative");
        // telemetry.addLine("The left joystick sets the robot direction");
        // telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addData("Actual Shooter Speed: ", currentShooterVelocity);
        if (shooterMode == 1) {
            telemetry.addData("Target Shooter Speed", slowShooterSpeed);
        } else if (shooterMode == 2) {
            telemetry.addData("Target Shooter Speed", shooterSpeed);
        } else if (shooterMode == 3) {
            telemetry.addData("Target Shooter Speed", fastShooterSpeed);
        } else if (shooterMode == 4 && gamepad2.right_trigger > 0) {
            telemetry.addData("Target Shooter Speed", targetRPM);
        } else if (shooterMode == 4) {
            telemetry.addData("Target Shooter Speed", slowShooterSpeed);
        }
        // If you press the X button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.x) {
            imu.resetYaw();
        }

        //region Artifact Indication Code
        if (artifactCounter >= 3) {
            hood.setArtifactIndicatorPos(0.5);
        } else {
            hood.setArtifactIndicatorPos(.611);
        }
        //endregion

        // region Controls and Artifact Counter
        // Intake Motor
        if (gamepad2.right_bumper  && artifactCounter < 3 || gamepad2.right_trigger > 0){
            intake.setPower(1);
        } else if(gamepad2.left_bumper){
            intake.setPower(-1);
           artifactCounter = 0;
        } else {
            intake.setPower(0);
        }

        // Feeder Motor
        // velocityCheck =  (currentShooterVelocity >= targetRPM - shooterSpeedTolerance) && (currentShooterVelocity <= targetRPM + shooterSpeedTolerance);
        if (gamepad2.right_trigger > 0) {
            feeder.setPower(1);
            artifactCounter = 0;
        } else {
            feeder.setPower(0);
        }

        // Artifact Counter
        if (artifactDetected & !artifactPresent) {
            artifactCounter++;
            artifactPresent = true;
        }

        if (!artifactDetected) {
            artifactPresent = false;
        }
        telemetry.addData("Artifact counter", artifactCounter);
        // endregion

       // region April Tag Detection

        LLResult llResult = limelight.getLatestResult();

        //endregion

        //region Variables and Telemetry
        double driveSpeed, strafe, turn, turretTurn;
        boolean rightSwitchState, leftSwitchState;
        rightSwitchState = limitSwitchRight.getState();
        leftSwitchState = limitSwitchLeft.getState();
        // turretTurn = 0;
        driveSpeed = -gamepad1.left_stick_y * DRIVE_SPEED;
        strafe = gamepad1.left_stick_x  * DRIVE_SPEED;
       double headingError = llResult.getTx();
        telemetry.addData("heading error", headingError);
        telemetry.addData("tx", llResult.getTx());
        //endregion

        //region Limelight
        LLResultTypes.FiducialResult goalTag = null;

        for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
            telemetry.addLine(String.format("Found tag: %d - %s", fiducial.getFiducialId(), fiducial));
            if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                telemetry.addLine(String.format("Found a goal tag! %d", fiducial.getFiducialId()));
                goalTag = fiducial;
            }
        }

        Position p = null;
        if (goalTag != null) {
            p = goalTag.getTargetPoseCameraSpace().getPosition();
            headingOffset = goalTag.getFiducialId() == 20 ? Blueoffset : goalTag.getFiducialId() == 24 ? Redoffset : 0.0;
            telemetry.addData("heading offset", headingOffset);
        }
        //endregion

        //region Auto Targeting
        if ((gamepad2.a && p != null)) {
            double offsetError = headingError + headingOffset;
            double dist = Math.hypot(p.x, p.z) * 39.3701;
            hood.setServoPos(-1.57333E-8 * Math.pow(dist,4) + .00000477067 * Math.pow(dist,3) - .000504967 * Math.pow(dist,2) + .0228883 * dist - .3125);

            shooterMode = 4;
            if  (Math.abs(offsetError) < BEARING_THRESHOLD) {
                turretTurn = 0;
                telemetry.addData("Auto", "Robot aligned with AprilTag!");
                if (gamepad2.a) {
                    hood.setReadyPos(.5);
                }
                else {
                    hood.setReadyPos(.277);
                }
            } else {
               turretTurn = Range.clip(Math.signum(offsetError) * (Math.sqrt(Math.abs(offsetError) * TURN_GAIN)), -MAX_AUTO_TURN, MAX_AUTO_TURN);
               hood.setReadyPos(.277);
               telemetry.addData("heading Error + heading offset", headingError+headingOffset);
            }
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turretTurn);
        } else
        //endregion

        // region Manual Aiming
       if (!leftSwitchState && gamepad2.left_stick_x < 0) {
            turretTurn = 0;
        } else if (!rightSwitchState && gamepad2.left_stick_x > 0) {
             turretTurn = 0;
        } else {
            turretTurn = gamepad2.left_stick_x;
        }
        telemetry.addData("turretTurn: ", turretTurn);
        turret.setPower(turretTurn);
        turn = gamepad1.right_stick_x;
        // Shooter Motor
        if (gamepad2.dpad_down) {
            targetRPM = (slowShooterSpeed);
            hood.setServoPos(closeHoodAngle);
        } else if (gamepad2.dpad_right) {
            targetRPM = (shooterSpeed);
            hood.setServoPos(mediumHoodAngle);
        } else if (gamepad2.dpad_up) {
            targetRPM = (fastShooterSpeed);
            hood.setServoPos(farHoodAngle);
        }
        //endregion


        //region RPM
        else if (gamepad2.dpad_left) {
            targetRPM = 0;
        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", shooter.getVelocity());
        shooter.setVelocity(targetRPM);
        shooter2.setVelocity(targetRPM);
        driveFieldRelative(driveSpeed, strafe, turn);
        //endregion

        //region More Auto Targeting
        if (p != null) {
            double dist = Math.hypot(p.x, p.z) * 39.3701;
            telemetry.addData("Range", dist);
            if (gamepad2.a) {
                targetRPM = (float) (
                        (-0.0000267733 * Math.pow(dist, 4))
                                + (0.00762133 * Math.pow(dist, 3))
                                - (.728067 * Math.pow(dist, 2))
                                + (32.69667 * dist)
                                + 515);
            }
        }
        //endregion
    }


//region PID Tuner
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
//endregion
   //endregion

//region Field Relative
        // This routine drives the robot field relative
        private void driveFieldRelative(double forward, double right, double rotate){
            // First, convert direction being asked to drive to polar coordinates
            double theta = Math.atan2(forward, right);
            double r = Math.hypot(right, forward);

            // Second, rotate angle by the angle the robot is pointing
            theta = AngleUnit.normalizeRadians(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            // Third, convert back to cartesian
            double newForward = r * Math.sin(theta);
            double newRight = r * Math.cos(theta);

            // Finally, call the drive method with robot relative forward and right amounts
            drive(newForward, newRight, rotate);
        }
//endregion

        //region Dive Code
        // Thanks to FTC16072 for sharing this code!!
        public void drive ( double forward, double right, double rotate){
            // This calculates the power needed for each wheel based on the amount of forward,
            // strafe right, and rotate
            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backRightPower = forward + right - rotate;
            double backLeftPower = forward - right + rotate;

            double maxPower = 1.0;
            double maxSpeed = 1.0;  // make this slower for outreaches

            if (gamepad1.left_trigger > 0) {
                maxSpeed = 0.4;
            } else if (gamepad1.right_trigger > 0) {
                maxSpeed = 1.0;
            } else {
                maxSpeed = 0.85;
            }

            // This is needed to make sure we don't pass > 1.0 to any wheel
            // It allows us to keep all of the motors in proportion to what they should
            // be and not get clipped
            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));

            // We multiply by maxSpeed so that it can be set lower for outreaches
            // When a young child is driving the robot, we may not want to allow full
            // speed.
            frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
            frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
            backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
            backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));


        }
//endregion
    }

