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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

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
@TeleOp(name = "TwentyTwentyFiveJava", group = "Robot")
public class TwentyTwentyFiveJava extends OpMode {
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
    public static double TURN_GAIN   =  0.08  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    //private AprilTagProcessor aprilTag;
    private Limelight3A limelight;
    private static final int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    public static int  DECIMATION = 3;
    public static float targetRPM;
    public static double rpmDistanceMultiplier = 7.61765;
    public static double axisOffsetRPM = 776.59769;
    public static double Blueoffset = 1;
    public static double Redoffset = 5;
    public static double hoodDistanceMultiplier = -0.00188;
    public static double getAxisOffsetHood = 0.535;
    public static boolean UPDATE_FLYWHEEL_PID = false;
    public static double FLYWHEEL_P = 135;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 12.928;
    public static double closeHoodAngle = 0;
    public static double mediumHoodAngle = 0.06;
    public static double farHoodAngle = 0.12;
    public static double FarRPMBump = 60;
    public static double FarHoodBump = -0.04;
    public static double headingOffset = 0;
    public static int artifactCounter = 0;
    public static boolean artifactPresent = false;

    public static double testHoodAngle = 0;
    public static boolean velocityCheck;

    public static boolean velocityCheck2;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    private final int READ_PERIOD = 1;
    private DigitalChannel laserInput;
    Hood hood = new Hood();

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;


    //private VisionPortal visionPortal;

    // Create the vision portal by using a builder.
    //VisionPortal.Builder builder = new VisionPortal.Builder();

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(DcMotor.class, "Shooter Feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        laserInput = hardwareMap.get(DigitalChannel.class, "LaserArtifactDetector");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        // Initialize the Apriltag Detection process
        //initAprilTag();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        //FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.init(hardwareMap);
        hood.setServoPos(0);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!UPDATE_FLYWHEEL_PID) {
            FLYWHEEL_P = c.p;
            FLYWHEEL_I = c.i;
            FLYWHEEL_D = c.d;
            FLYWHEEL_F = c.f;
        }
        pidTuner();



        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    @Override
    public void loop() {
        boolean artifactDetected = laserInput.getState();
        pidTuner();
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
        } else if (shooterMode == 4 && gamepad2.right_bumper) {
            telemetry.addData("Target Shooter Speed", targetRPM);
        } else if (shooterMode == 4) {
            telemetry.addData("Target Shooter Speed", slowShooterSpeed);
        }
        // If you press the X button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.x) {
            imu.resetYaw();
        }

        // Artifact Indication Code
        if (artifactCounter >= 3) {
            hood.setArtifactIndicatorPos(0.5);
        } else {
            hood.setArtifactIndicatorPos(.611);
        }

        // Intake Motor
        if (gamepad2.right_trigger > 0 && artifactCounter < 3 || gamepad2.right_bumper){
            intake.setPower(1);
        } else if(gamepad2.left_trigger > 0){
            intake.setPower(-1);
            artifactCounter = 0;
        } else {
            intake.setPower(0);
        }

        // Feeder Motor
        // velocityCheck =  (currentShooterVelocity >= targetRPM - shooterSpeedTolerance) && (currentShooterVelocity <= targetRPM + shooterSpeedTolerance);
        if (gamepad2.right_bumper ) {
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


       /* List<AprilTagDetection> currentDetections = aprilTag.getDetections();
       telemetryAprilTag(currentDetections);
       AprilTagDetection goalTag = getGoalTag(currentDetections); */
        LLResult llResult = limelight.getLatestResult();

        /* Tell the driver what we see, and what to do.
        if (goalTag != null) {
            telemetry.addData("Found", "ID %d (%s)", goalTag.id, goalTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", goalTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", goalTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", goalTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }*/

        double driveSpeed, strafe, turn;
        driveSpeed = -gamepad1.left_stick_y * DRIVE_SPEED;
        strafe = gamepad1.left_stick_x  * DRIVE_SPEED;
        double headingError = llResult.getTx();
        telemetry.addData("heading error", headingError);
        telemetry.addData("tx", llResult.getTx());
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

        if ((gamepad1.a && p != null)) {
            double offsetError = headingError + headingOffset;
            double dist = Math.hypot(p.x, p.z) * 39.3701;
            hood.setServoPos(-4.42373E-8 * Math.pow(dist,4) + .0000120342 * Math.pow(dist,3) - .00114364 * Math.pow(dist,2) + .0455678 * dist - .595175);

            shooterMode = 4;
           // velocityCheck2 = currentShooterVelocity <= (targetRPM + shooterSpeedTolerance) && currentShooterVelocity >= (targetRPM - shooterSpeedTolerance);
            if  (Math.abs(offsetError) < BEARING_THRESHOLD) {
                turn = 0;
                telemetry.addData("Auto", "Robot aligned with AprilTag!");
                if (gamepad1.a) {
                    hood.setReadyPos(.5);
                }
                else {
                    hood.setReadyPos(.277);
                }
            } else {
                turn = Range.clip((offsetError + (Math.signum(offsetError) * TURN_STATIC)) * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                hood.setReadyPos(.277);
                telemetry.addData("heading Error + heading offset", headingError+headingOffset);
            }
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        } else {
            turn = gamepad1.right_stick_x;

        }
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
        } else if (gamepad1.a && p != null) {
            double dist = Math.hypot(p.x, p.z) * 39.3701;
            targetRPM = (float) (
                    (-0.0000215225 * Math.pow(dist,4))
                            + (0.00607313 * Math.pow(dist,3))
                            - (.570502 * Math.pow(dist,2))
                            + (26.36195 * dist)
                            + 571.02925);
            telemetry.addData("Range", dist);
        } else if (gamepad2.dpad_left) {
            targetRPM = 0;
        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", shooter.getVelocity());
        shooter.setVelocity(targetRPM);
        shooter2.setVelocity(targetRPM);
        driveFieldRelative(driveSpeed, strafe, turn);
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

//    private void initAprilTag() {
//
//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//
//                // The following default settings are available to un-comment and edit as needed.
//                //.setDrawAxes(false)
//                //.setDrawCubeProjection(false)
//                //.setDrawTagOutline(true)
//                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//
//                // == CAMERA CALIBRATION ==
//                // If you do not manually specify calibration parameters, the SDK will attempt
//                // to load a predefined calibration for your camera.
//                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                // ... these parameters are fx, fy, cx, cy.
//
//                .build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(DECIMATION);
//
//        // Create the vision portal by using a builder.
//        //VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera
//        //builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor and build the vision portal
//       // builder.addProcessor(aprilTag);
//       // visionPortal = builder.build();
//
//        // Disable or re-enable the aprilTag processor at any time.
//        //visionPortal.setProcessorEnabled(aprilTag, true);
//
//    }   // end method initAprilTag()


    //private void telemetryAprilTag(List<AprilTagDetection> currentDetections) {
        //telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        //for (AprilTagDetection detection : currentDetections) {
           // if (detection.metadata != null) {
                //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            //} else {
             //   telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
               // telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        //    }
        //}    end for() loop

        // Add "key" information to telemetry
        // telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        // telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        // telemetry.addLine("RBE = Range, Bearing & Elevation");

       // end method telemetryAprilTag()

    //private AprilTagDetection getGoalTag(List<AprilTagDetection> detections) {
       // for (AprilTagDetection detection : detections) {
            // Look to see if we have size info on this tag.
           // if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
              //  if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                  //  return detection;
              //  } else {
                    // This tag is in the library, but we do not want to track it right now.
                  //  telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
               // }
           // } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
               // telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);


        //return null;


    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
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


    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        if (gamepad1.left_bumper) {
            maxSpeed = 0.4;
        } else if (gamepad1.right_bumper) {
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

}

