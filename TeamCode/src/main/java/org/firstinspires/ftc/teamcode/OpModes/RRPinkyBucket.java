package org.firstinspires.ftc.teamcode.OpModes;


/* Copyright (c) 2019 FIRST. All rights reserved.
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

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.MSMechOps;
import org.firstinspires.ftc.teamcode.PinpointDrive;

//@Disabled
@Autonomous(name = "Auto - Pinky Bucket", group = "Competition", preselectTeleOp = "RobotTeleOp")
public class RRPinkyBucket extends LinearOpMode{

    public static String TEAM_NAME = "Mouse Spit";
    public static int TEAM_NUMBER = 11572;

    public final static HWProfile2 robot = new HWProfile2();
    public final static MSParams params = new MSParams();

    public LinearOpMode opMode = this;
    public MSMechOps mechOps;

    //PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
    PinpointDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);
        mechOps = new MSMechOps(robot, opMode, params);


        robot.servoClaw.setPosition(params.CLAW_CLOSE);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_DOWN);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_DOWN);

        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.motorLift.setPower(1);
        robot.motorLiftRight.setPower(1);
        robot.motorLEFT.setPower(0.6);
        robot.motorRIGHT.setPower(0.6);

        while (!isStopRequested() && !opModeIsActive()) {
            // Wait for the DS start button to be touched.
            telemetry.addData(">", "Touch Play to start OpMode");

            telemetry.update();

        }
        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            runAutonoumousMode();
        }
    }

    //end runOpMode();

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d samplePreScoringPosition = new Pose2d(0, 0, 0);
        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d parkPrepPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        drive = new PinpointDrive(hardwareMap, initPose);

        /*****************
         * Set values for RoadRunner Pathing
         */

        samplePreScoringPosition = new Pose2d(5, -15, Math.toRadians(45));
        sampleScoringPosition = new Pose2d(20, -3, Math.toRadians(45));
        yellowSample1Position = new Pose2d(13, -22, Math.toRadians(-90));
        yellowSample2Position = new Pose2d(24, -20, Math.toRadians(-90));
        yellowSample3Position = new Pose2d(23, -28, Math.toRadians(-45));
        parkPrepPose = new Pose2d(-16,-50 , Math.toRadians(180));
        parkPose = new Pose2d(-11, -51, Math.toRadians(180));

        // Raise Arm to high bar scoring position

        // TODO: Add code to release the sample and lower the arm
        // if (opModeIsActive()) robot.servoWrist.setPosition(params.Wrist_Auto);
        if (opModeIsActive()) mechOps.AutoDumpUp();




        // Drive to specimen scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(samplePreScoringPosition.position, samplePreScoringPosition.heading)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        //score sample
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_OPEN);
        safeWaitSeconds(0.5);
        mechOps.liftPosition(params.LIFT_MIN_LOW);
        //pick sample 1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(yellowSample1Position.position,yellowSample1Position.heading)
                        .build());

        //Transfer
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        if (opModeIsActive()) mechOps.AutoDumpDown();
        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        //drive to scoring
        if (opModeIsActive()) mechOps.AutoDumpUp();

            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //   .strafeToLinearHeading(samplePreScoringPosition.position, samplePreScoringPosition.heading)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        //score sample
        if (opModeIsActive()) robot.servoTwist.setPosition(params.TWIST_VERTICAL);
        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_OPEN);
        safeWaitSeconds(0.5);

        mechOps.liftPosition(params.LIFT_MIN_LOW);
        //pick sample 1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(yellowSample2Position.position,yellowSample2Position.heading)
                        .build());

        //Transfer
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        if (opModeIsActive()) mechOps.AutoDumpDown();

        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        //drive to scoring
        if (opModeIsActive()) mechOps.AutoDumpUp();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //   .strafeToLinearHeading(samplePreScoringPosition.position, samplePreScoringPosition.heading)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());
        //pick sample 3
        //score sample
        if (opModeIsActive()) robot.servoTwist.setPosition(params.TWIST_VERTICAL);
        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_OPEN);
        safeWaitSeconds(0.5);

        mechOps.liftPosition(params.LIFT_MIN_LOW);
        //pick sample 3
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(yellowSample3Position.position,yellowSample3Position.heading)
                        .build());

        //Transfer
        if (opModeIsActive()) robot.servoTwist.setPosition(params.TWIST_HALF);
        if (opModeIsActive()) mechOps.AutoDumpDown();

        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        //drive to scoring
        if (opModeIsActive()) mechOps.AutoDumpUp();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //   .strafeToLinearHeading(samplePreScoringPosition.position, samplePreScoringPosition.heading)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        //parking
        if (opModeIsActive()) mechOps.AutoDumpDown();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPrepPose.position, parkPrepPose.heading)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

}   // end class
