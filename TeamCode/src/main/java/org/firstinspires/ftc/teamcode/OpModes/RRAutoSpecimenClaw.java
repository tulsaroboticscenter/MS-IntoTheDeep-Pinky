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
@Autonomous(name = "Auto - SPECIMENS CLAW", group = "Competition", preselectTeleOp = "RobotTeleOp")
public class RRAutoSpecimenClaw extends LinearOpMode{

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
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_DOWN);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_DOWN);
        robot.servoClaw.setPosition(params.CLAW_CLOSE);
        robot.servoBar.setPosition(params.Bar_Auto);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
      //  robot.servoWrist.setPosition(params.Wrist_Box);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoBucket.setPosition(params.Bucket_Catch);

        while (!isStopRequested() && !opModeIsActive()) {
            // Wait for the DS start button to be touched.
            telemetry.addData(">", "Touch Play to start OpMode");

            telemetry.update();
            robot.servoClaw.setPosition(params.CLAW_CLOSE);
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
        Pose2d specimenPrePreScoringPosition = new Pose2d(0, 0, 0);
        Pose2d specimenPreScoringPosition = new Pose2d(0, 0, 0);
        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
        Pose2d specimenScoringSlide = new Pose2d(0, 0, 0);
        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d midwayPose0 = new Pose2d(0, 0, 0);
        Pose2d midwayPose05 = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d Sweep3 = new Pose2d(0, 0, 0);
        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
        Pose2d Sweep2 = new Pose2d(0, 0, 0);
        Pose2d midwayPose3 = new Pose2d(0, 0, 0);
        Pose2d Sweep1 = new Pose2d(0, 0, 0);

        Pose2d parkPrepPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        drive = new PinpointDrive(hardwareMap, initPose);

        /*****************
         * Set values for RoadRunner Pathing
         */
        specimenPrePreScoringPosition= new Pose2d(10, 5, Math.toRadians(45));//old
        specimenPreScoringPosition = new Pose2d(15, 11, Math.toRadians(0));
        specimenScoringPosition = new Pose2d(25, 11, 0);
        specimenScoringSlide = new Pose2d(25, 15, 0);
        grabSpecimenPosition = new Pose2d(1, -24, Math.toRadians(0));
        coloredSample1Position = new Pose2d(5, 30, Math.toRadians(90));
        coloredSample2Position = new Pose2d(35, 58, 90);
        coloredSample3Position = new Pose2d(35, 60, Math.toRadians(90));
        midwayPose0 = new Pose2d(14, -21, Math.toRadians(0)); //Before first pick old
        midwayPose05 = new Pose2d(50, -21, 0);
        midwayPose1= new Pose2d(50, -30, Math.toRadians(0));//pick first 15
        Sweep1 = new Pose2d(9.5, -31, Math.toRadians(0));
        midwayPose2 = new Pose2d(50, -41.5, Math.toRadians(0)); //pick middle 25  .-27
        Sweep2 = new Pose2d(7.5, -41.5, Math.toRadians(0));
        midwayPose3 = new Pose2d(50, -49, Math.toRadians(0)); //pick close to wall 35 .-27
        Sweep3 = new Pose2d(7.7, -49, Math.toRadians(0));

        parkPose = new Pose2d(0, -35, Math.toRadians(-95));

        // Raise Arm to high bar scoring position

        // TODO: Add code to release the sample and lower the arm
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        if (opModeIsActive()) robot.servoClawRotation1.setPosition(params.CLAWROTATION1_DOWN);
        if (opModeIsActive()) robot.servoClawRotation2.setPosition(params.CLAWROTATION2_DOWN);
        if (opModeIsActive()) robot.servoExtend.setPosition(params.Extend_IN);
        if (opModeIsActive()) robot.servoExtendRight.setPosition(params.ExtendRight_IN);


        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        if (opModeIsActive()) mechOps.SpiceScore2();
        //sleep(1000);
        // Drive to specimen scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                        .build());

        // Score specimen
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
        if (opModeIsActive()) mechOps.openClaw();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                        .build());
        if (opModeIsActive()) mechOps.openClaw();
        //sleep(1000);
        // TODO: Add code to release the sample and lower the arm
        if (opModeIsActive())mechOps.armin();
        if (opModeIsActive()) mechOps.openClaw();
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
//                        .build());


        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        //          Lower Lift
        //if (opModeIsActive()) mechOps.raiseLift(params.LIFT_Floor);

        // Drive to color specimen Position
        // Push Color Sample1 into the Observation area
        // Drive to color sample1 Position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                       // .strafeToLinearHeading(specimenPreScoringPosition.position,specimenPreScoringPosition.heading)
                        .strafeToLinearHeading(midwayPose0.position, midwayPose0.heading)
                        .strafeToLinearHeading(midwayPose05.position,midwayPose05.heading)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .strafeToLinearHeading(Sweep1.position, Sweep1.heading)
                        .build());

//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                        .build());
        //if (opModeIsActive()) mechOps.Sweep();
        //Turn to Sample 1 Drop
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(Sweep1.position, Sweep1.heading)
//                        .build());
        //Turn to Sample Pick 2
      //  if (opModeIsActive()) mechOps.PreSweep();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .strafeToLinearHeading(Sweep2.position, Sweep2.heading)
                        .build());
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
//                        .build());
       // if (opModeIsActive()) mechOps.Sweep();
        //Turn to Sample 2 Drop
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(Sweep2.position, Sweep2.heading)
//                        .build());
       // if (opModeIsActive()) mechOps.PreSweep();

        /*Turn to Sample Pick 3
        *
        *
         */
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
//                        .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
//                        .strafeToLinearHeading(Sweep3.position, Sweep3.heading)
//
//                        .build());
       // if (opModeIsActive()) mechOps.Sweep();

        //Turn to Sample 3 Drop
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(Sweep3.position, Sweep3.heading)
//                        .build());

        if (opModeIsActive()) mechOps.WallGrab();
        // Grab the specimen 2
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());

        Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(0))
                            .lineToX(3)
                            .build()  );

        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        safeWaitSeconds(0.1);

        if (opModeIsActive()) mechOps.SpiceScore2();

        // Drive to specimen scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                        .build());

        // Score specimen

        // TODO: Add code to release the sample and lower the arm
        if (opModeIsActive()) mechOps.SpiceScore2();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
                        .build());
        if (opModeIsActive()) mechOps.openClaw();
        //          Lower Lift
        if (opModeIsActive()) mechOps.WallGrab();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(0))
                        .lineToX(3)
                        .build()  );
        // Grab the specimen 3
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        safeWaitSeconds(0.1);

        if (opModeIsActive()) mechOps.SpiceScore2();



        // Drive to specimen scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                        .build());

        // Score specimen

        // TODO: Add code to release the sample and lower the arm
        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_SCORE);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
                        .build());
        if (opModeIsActive()) mechOps.openClaw();

        if (opModeIsActive()) mechOps.WallGrab();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());

//here to stop 4th Spec
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(0))
                        .lineToX(3)
                        .build()  );
        // Grab the specimen 4
        if (opModeIsActive()) robot.servoClaw.setPosition(params.CLAW_CLOSE);
        safeWaitSeconds(0.1);

        if (opModeIsActive()) mechOps.SpiceScore2();


        // Drive to specimen scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                        .build());

        // Score specimen

        // TODO: Add code to release the sample and lower the arm
        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_SCORE);
        //      Actions.runBlocking(
 //               drive.actionBuilder(drive.pose)
 //                       .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
 //                       .build());

        if (opModeIsActive()) safeWaitSeconds(0.25);
        if (opModeIsActive()) mechOps.openClaw();

        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_MIN_LOW);
        if (opModeIsActive()) mechOps.armout();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());

        //          Lower Lift

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }

    /**
     *
     */

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

}   // end class
