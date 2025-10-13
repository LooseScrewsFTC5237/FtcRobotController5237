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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
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
@TeleOp(name = "TwentyTwentyFourJava", group = "Robot")
public class TwentyTwentyFourJava extends OpMode {
    // This declares the four motors needed
    int lift_height = 0;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    // DcMotor leftlift;
    // DcMotor rightlift;
    // TouchSensor leftlift0;
    // TouchSensor rightlift0;
    // Servo left_claw;
    // Servo right_claw;
    // Servo dunker;
    // Servo intakepivot;
    // Servo rightgrapplehook;
    // Servo leftgrapplehook;

    private final int READ_PERIOD = 1;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL Drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR Drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL Drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR Drive");
        // leftlift = hardwareMap.get(DcMotor.class, "left lift");
        // rightlift = hardwareMap.get(DcMotor.class, "right lift");
        // rightlift0 = hardwareMap.get(TouchSensor.class, "right lift 0");
        // leftlift0 = hardwareMap.get(TouchSensor.class,"left lift 0");
        // left_claw = hardwareMap.get(Servo.class, "Left Clawimen");
        // right_claw = hardwareMap.get(Servo.class, "Right Specclaw");
        // dunker = hardwareMap.get(Servo.class, "DUNKER");
        // intakepivot = hardwareMap.get(Servo.class, "intake pivot");
        // rightgrapplehook = hardwareMap.get(Servo.class, "Right Rapple Grook");
        // leftgrapplehook = hardwareMap.get(Servo.class, "Left Rrapple Gook");


        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        // frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        // rightlift.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightlift.setTargetPosition(0);
        // leftlift.setTargetPosition(0);
        // rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftlift.setPower(0.9);
        // rightlift.setPower(0.9);
        // leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

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
        if (gamepad2.dpad_up) {
            lift_height = 750;
        } else if (gamepad2.dpad_down) {
            lift_height = 0;
        } else if (gamepad2.dpad_left) {
            lift_height = 350;
        } else if (gamepad2.dpad_right) {
            lift_height = 500;
        }
        /*
        if(leftlift0.isPressed()) {
            leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(rightlift0.isPressed()) {
            rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (rightlift.getTargetPosition() >= 25) {
            rightlift.setPower(0.9);
        } else if (rightlift.getTargetPosition() < 25 && rightlift.getCurrentPosition() >= 25) {
            rightlift.setPower(0.5);
        } else if (rightlift.getTargetPosition() < 25 && rightlift.getCurrentPosition() < 25) {
            rightlift.setPower(0);
        }
        if (leftlift.getTargetPosition() >= 25) {
            leftlift.setPower(0.9);
        } else if (leftlift.getTargetPosition() < 25 && leftlift.getCurrentPosition() >= 25) {
            leftlift.setPower(0.5);
        } else if (leftlift.getTargetPosition() < 25 && leftlift.getCurrentPosition() < 25) {
            leftlift.setPower(0);
        }
        */
        // rightlift.setTargetPosition(lift_height);
        // leftlift.setTargetPosition(lift_height);
        telemetry.addData("Lift Height", lift_height);
        // telemetry.addData("left lift",leftlift.getCurrentPosition());
        // telemetry.addData("right lift",rightlift.getCurrentPosition());
        // telemetry.addData("right lift power",rightlift.getPower());
        // telemetry.addData("left lift power",leftlift.getPower());
        if ((lift_height <= 750) && (gamepad2.right_stick_y < 0)) {
            lift_height -= gamepad2.right_stick_y * 50;
        } else if (gamepad2.right_stick_y > 0) {
            lift_height -= gamepad2.right_stick_y * 50;
        }
        /*
        if (gamepad2.x) {
            left_claw.setPosition(0.8);
            right_claw.setPosition(0.2);
        } else  {
            left_claw.setPosition(1);
            right_claw.setPosition(0);
        }
        if (gamepad2.b) {
            dunker.setPosition(0.23);
        } else {
            dunker.setPosition(0);
        }
        if (gamepad2.y) {
            intakepivot.setPosition(0.772);
        }
        if (gamepad2.a) {
            leftgrapplehook.setPosition(0.5);
            rightgrapplehook.setPosition(0.5);
        } else {
            leftgrapplehook.setPosition(1);
            rightgrapplehook.setPosition(0);
        }
        */


    }
}
