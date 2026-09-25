/* Copyright (c) 2025 FIRST. All rights reserved.
 * ...
 */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
@TeleOp(name = "CameraTrakingRed", group = "Robot")
public class CameraTrakingRed extends OpMode {

    public static double DESIRED_DISTANCE = 12.0;
    public static double BEARING_THRESHOLD = 1;
    public static double TURN_GAIN = 0.08;
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN = 0.3;

    private Limelight3A limelight;

    public static int DECIMATION = 3;
    public static double rpmDistanceMultiplier = 7.61765;
    public static double axisOffsetRPM = 776.59769;
    public static double DRIVE_SPEED = 0.8;
    public static double headingOffset = 0;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();

        double driveSpeed = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = gamepad1.left_stick_x * DRIVE_SPEED;
        double turn = gamepad1.right_stick_x * DRIVE_SPEED; // Manual turn

        LLResultTypes.FiducialResult goalTag = null;

        if (llResult != null && llResult.isValid()) {
            double headingError = llResult.getTx();
            telemetry.addData("heading error", headingError);
            telemetry.addData("tx", llResult.getTx());

            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                telemetry.addLine(String.format("Found tag: %d", id));

                if (id >= 30 && id <= 37) {
                    telemetry.addLine(String.format("Found a goal tag! %d", id));
                    goalTag = fiducial;
                }
            }

            if (goalTag != null) {
                Position p = goalTag.getTargetPoseCameraSpace().getPosition();
                double dist = Math.hypot(p.x, p.z) * 39.3701;
                telemetry.addData("Target Distance", "%.1f inches", dist);
            }
        }
        else {
            telemetry.addData("Tags Found", "0");
        }

        // Mecanum drive power calculations
        double frontLeftPower  = driveSpeed + strafe + turn;
        double frontRightPower = driveSpeed - strafe - turn;
        double backLeftPower   = driveSpeed - strafe + turn;
        double backRightPower  = driveSpeed + strafe - turn;

        // Normalize wheel powers
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Apply power to motors
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        telemetry.update();
    } // <--- This closing bracket was missing in your original code
}