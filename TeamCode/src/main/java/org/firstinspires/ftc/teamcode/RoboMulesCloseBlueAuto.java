/*
 * Copyright (c) 2025 Base 10 Assets, LLC
 * All rights reserved.
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
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels," and two servos
 * which feed that launcher.
 *
 * This robot starts up against the goal and launches all three projectiles before driving away
 * off the starting line.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */
@Config
@Autonomous(name="Robo Mules Blue Close", group="StarterBot")
//@Disabled
public class RoboMulesCloseBlueAuto extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private ElapsedTime runtime = new ElapsedTime();

    private ElapsedTime feederOnTimer = new ElapsedTime();

    public static double feederOnTime = 0.185;
    private static boolean feederOnReset = true;
    private ElapsedTime betweenShotsTime = new ElapsedTime();
    private static boolean betweenShotsReset = true;
    private ElapsedTime turnTimer = new ElapsedTime();
    private static boolean turnReset = true;
    private ElapsedTime backwardsTimer = new ElapsedTime();
    private static double backwardsReset = 0;
    public static double LAUNCHER_TARGET_VELOCITY = 975;

    public static double shotsTaken = 0;

    public static double turnTime = 0.3;
    public static double backwardsTime = 0.7;
    public static double endTime = 20;
    private enum LaunchState {
        SPIN_UP,
        SHOT,
        WAIT_BETWEEN_SHOTS,
        TURN,
        BACKWARD,
        END
    }

    private RoboMulesCloseBlueAuto.LaunchState autoState;

    @Override
    public void init() {

        runtime.reset();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        // Inside init():
        PIDFCoefficients pidfNew = new PIDFCoefficients(250, 0, 0, 12.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void start() {
        runtime.reset();
        autoState = LaunchState.SPIN_UP;
        shotsTaken = 0;
        feederOnReset = true;
        betweenShotsReset = true;
        turnReset = true;
        backwardsReset = 0;
    }

    @Override
    public void loop() {

        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        if (runtime.seconds() >= endTime) {
            autoState = LaunchState.END;
        }

        switch (autoState) {
            case SPIN_UP:
                if (runtime.seconds() >= 3.0) {
                    autoState = LaunchState.SHOT;
                }
                break;
            case SHOT:
                
                if (feederOnReset) {
                    feederOnTimer.reset();
                    feederOnReset = false;
                }

                if (feederOnTimer.seconds() <= feederOnTime && shotsTaken <= 4 ) {
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                } else if (shotsTaken <= 4) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    betweenShotsReset = true;
                    autoState = LaunchState.WAIT_BETWEEN_SHOTS;
                    shotsTaken++;
                } else {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    autoState = LaunchState.BACKWARD;
                }

                break;
            case WAIT_BETWEEN_SHOTS:

                if (betweenShotsReset) {
                    betweenShotsTime.reset();
                    betweenShotsReset = false;
                }

                if (betweenShotsTime.seconds() >= 1) {
                    autoState = LaunchState.SHOT;
                    feederOnReset = true;
                }

                break;
            case TURN:

                if (turnReset) {
                    turnTimer.reset();
                    turnReset = false;
                }
                
                if (turnTimer.seconds() >= 0.5) {
                    leftBackDrive.setPower(0.5);
                    leftFrontDrive.setPower(0.5);
                    rightFrontDrive.setPower(-0.5);
                    rightBackDrive.setPower(-0.5);
                }

                if (turnTimer.seconds() >= 0.5 + turnTime) {
                    leftBackDrive.setPower(0);
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    
                    backwardsReset = 3;
                    autoState = LaunchState.BACKWARD;
                }
                break;
            case BACKWARD:
                if (backwardsReset == 0) {
                    backwardsTimer.reset();
                    backwardsReset = 1;
                }
                if (backwardsReset == 3) {
                    backwardsTimer.reset();
                    backwardsReset = 4;
                }
                if (backwardsTimer.seconds() >= 0.5) {
                    leftFrontDrive.setPower(-0.5);
                    leftBackDrive.setPower(-0.5);
                    rightFrontDrive.setPower(-0.5);
                    rightBackDrive.setPower(-0.5);
                }
                if (backwardsTimer.seconds() >= 0.5 + backwardsTime) {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    
                    if (backwardsReset == 1) {
                        backwardsReset = 2;
                    }
                    if (backwardsReset == 4) {
                        backwardsReset = 5;
                    }
                }
                if (backwardsReset == 2) {
                    autoState = LaunchState.TURN;
                }
                if (backwardsReset == 5) {
                    autoState = LaunchState.END;
                }
                break;
            case END:
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                launcher.setPower(0);
                leftFeeder.setPower(0);
                break;
        }

    }
}







