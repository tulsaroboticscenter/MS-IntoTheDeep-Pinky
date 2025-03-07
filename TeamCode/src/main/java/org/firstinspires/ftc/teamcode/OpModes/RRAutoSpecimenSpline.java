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
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

//@Disabled
@Autonomous(name = "Auto - SPECIMENS Spline", group = "Competition", preselectTeleOp = "RobotTeleOp")
public class RRAutoSpecimenSpline extends LinearOpMode{

    public static String TEAM_NAME = "Mouse Spit";
    public static int TEAM_NUMBER = 11572;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_SAMPLES,
        BLUE_SPECIMENS,
        RED_SAMPLES,
        RED_SPECIMENS
    }

    public static START_POSITION startPosition;

    public final static HWProfile2 robot = new HWProfile2();
    public final static MSParams params = new MSParams();
    public LinearOpMode opMode = this;
    public MSMechOps mechOps;

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);
        mechOps = new MSMechOps(robot, opMode, params);

        robot.servoSpice.setPosition(params.SPICE_CLOSE);
        robot.servoClaw.setPosition(params.CLAW_CLOSE);
        robot.servoBar.setPosition(params.Bar_Auto);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoWrist.setPosition(params.Wrist_Box);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoBucket.setPosition(params.Bucket_Catch);

        while (!isStopRequested() && !opModeIsActive()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            //robot.servoSpice.setPosition(params.SPICE_CLOSE);
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
        Pose2d grabSpecimenPrePosition = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d midwayPose0 = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
        Pose2d midwayPose3 = new Pose2d(0, 0, 0);
        Pose2d midwayPose4 = new Pose2d(0, 0, 0);

        Pose2d parkPrepPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);


        drive = new PinpointDrive(hardwareMap, initPose);

        /*****************
         * Set values for RoadRunner Pathing
         */
        specimenPrePreScoringPosition= new Pose2d(-10, 5, 45);
        specimenPreScoringPosition = new Pose2d(-18, -9, 0);
        //specimenScoringX =
        specimenScoringPosition = new Pose2d(-33, -10, 0);
        specimenScoringSlide = new Pose2d(-33, -15, 0);
        grabSpecimenPrePosition = new Pose2d(-15, 0, Math.toRadians(-90));
        grabSpecimenPosition = new Pose2d(-7, 20, Math.toRadians(180));
        coloredSample1Position = new Pose2d(-5, 30, Math.toRadians(-90));
        coloredSample2Position = new Pose2d(-35, -58, 90);
        coloredSample3Position = new Pose2d(-35, -60, Math.toRadians(90));
        midwayPose0 = new Pose2d(-22, 9, Math.toRadians(103)); //Before first pick
        midwayPose1 = new Pose2d(-25, 30, Math.toRadians(110)); //pick close to wall 35 .-27
        midwayPose2 = new Pose2d(-25, 22, Math.toRadians(110)); //pick middle 25  .-27
        midwayPose3 = new Pose2d(-29.5, 13, Math.toRadians(110));//pick first 15
        midwayPose4 = new Pose2d(-11, 20, Math.toRadians(50)); // drop off

        parkPose = new Pose2d(0, 40, Math.toRadians(-180));


        // Raise Arm to high bar scoring position
        if (opModeIsActive()) {
            // TODO: Add code to release the sample and lower the arm
            robot.servoSpice.setPosition(params.SPICE_CLOSE);
            robot.servoWrist.setPosition(params.Wrist_Auto);
            mechOps.raiseLift(params.LIFT_CLIP_HIGH);


            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                           // .strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
                            .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            // Score specimen

            // TODO: Add code to release the sample and lower the arm
            mechOps.raiseLift(params.LIFT_CLIP_SCORE);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
                            .build());
            mechOps.openClaw();


            //          Lower Lift
            mechOps.raiseLift(params.LIFT_MIN_LOW);
            mechOps.armout();
            // Drive to color specimen Position
            // Push Color Sample1 into the Observation area
            // Drive to color sample3 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(specimenPreScoringPosition.position,specimenPreScoringPosition.heading),specimenPreScoringPosition.heading)
                            .splineToLinearHeading(new Pose2d(midwayPose0.position, midwayPose0.heading) ,midwayPose0.heading)
                            .splineToLinearHeading(new Pose2d(midwayPose3.position, midwayPose3.heading), midwayPose3.heading)
                            .build());

            robot.servoClaw.setPosition(params.CLAW_CLOSE);

            //Turn to Sample 3 Drop

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(midwayPose4.position, midwayPose4.heading), midwayPose4.heading)
                            .build());
            robot.servoClaw.setPosition(params.CLAW_OPEN);

            //Turn to Sample Pick 2

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(midwayPose2.position, midwayPose2.heading), midwayPose2.heading)
                            .build());
            robot.servoClaw.setPosition(params.CLAW_CLOSE);

            //Turn to Sample 2 Drop

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(midwayPose4.position, midwayPose4.heading), midwayPose4.heading)
                            .build());
            robot.servoClaw.setPosition(params.CLAW_OPEN);


            //Turn to Sample Pick 1
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(midwayPose1.position, midwayPose1.heading), midwayPose1.heading)
                            .build());
            robot.servoClaw.setPosition(params.CLAW_CLOSE);
            //Turn to Sample 1 Drop
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(midwayPose4.position, midwayPose4.heading), midwayPose4.heading)
                            .build());
            robot.servoClaw.setPosition(params.CLAW_OPEN);


            mechOps.armin();
            // Grab the specimen 2
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(grabSpecimenPrePosition.position, grabSpecimenPrePosition.heading),grabSpecimenPrePosition.heading)
                            .splineToLinearHeading(new Pose2d(grabSpecimenPosition.position, grabSpecimenPosition.heading),grabSpecimenPosition.heading)
                            .turnTo(Math.toRadians(-180))
                            .lineToX(2)
                            .build());
/**
            Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .turnTo(Math.toRadians(-180))
                                .lineToX(2)
                                .build()  );
**/
            robot.servoSpice.setPosition(params.SPICE_CLOSE);
            safeWaitSeconds(0.1);

            mechOps.raiseLift(params.LIFT_CLIP_HIGH);


            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading), specimenPrePreScoringPosition.heading)
                            .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            //.lineToX(-33)
                            .build());

            // Score specimen

            // TODO: Add code to release the sample and lower the arm
            mechOps.raiseLift(params.LIFT_CLIP_SCORE);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
                            .build());
            mechOps.openClaw();
            //          Lower Lift
            mechOps.raiseLift(params.LIFT_MIN_LOW);

            //Return to wall for 3
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(grabSpecimenPrePosition.position, grabSpecimenPrePosition.heading), grabSpecimenPrePosition.heading)
                            .splineToLinearHeading(new Pose2d(grabSpecimenPosition.position, grabSpecimenPosition.heading), grabSpecimenPosition.heading)
                            .build());

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(-180))
                            .lineToX(2)
                            .build()  );
            // Grab the specimen 3
            robot.servoSpice.setPosition(params.SPICE_CLOSE);
            safeWaitSeconds(0.1);

            mechOps.raiseLift(params.LIFT_CLIP_HIGH);


            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading), specimenPrePreScoringPosition.heading)
                            .splineToLinearHeading(new Pose2d(specimenPreScoringPosition.position, specimenPreScoringPosition.heading), specimenPreScoringPosition.heading)
                            //.splineToLinearHeading(new Pose2d(specimenScoringPosition.position, specimenScoringPosition.heading), specimenScoringPosition.heading)
                            .lineToX(-33)
                            .build());

            // Score specimen 3

            // TODO: Add code to release the sample and lower the arm
            mechOps.raiseLift(params.LIFT_CLIP_SCORE);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
                            .build());
            mechOps.openClaw();

            mechOps.raiseLift(params.LIFT_MIN_LOW);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(grabSpecimenPrePosition.position, grabSpecimenPrePosition.heading), grabSpecimenPrePosition.heading)
                            .splineToLinearHeading(new Pose2d(grabSpecimenPosition.position, grabSpecimenPosition.heading), grabSpecimenPosition.heading)
                            .build());

//here to stop 4th Spec
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(-180))
                            .lineToX(4)
                            .build()  );
            // Grab the specimen 4
            robot.servoSpice.setPosition(params.SPICE_CLOSE);
            safeWaitSeconds(0.1);

            mechOps.raiseLift(params.LIFT_CLIP_HIGH);


            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineToLinearHeading(new Pose2d(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading), specimenPrePreScoringPosition.heading)
                            .splineToLinearHeading(new Pose2d(specimenPreScoringPosition.position, specimenPreScoringPosition.heading), specimenPreScoringPosition.heading)
                            //.splineToLinearHeading(new Pose2d(specimenScoringPosition.position, specimenScoringPosition.heading), specimenScoringPosition.heading)
                            .lineToX(-33)
                            .build());

            // Score specimen

            // TODO: Add code to release the sample and lower the arm
            mechOps.raiseLift(params.LIFT_CLIP_SCORE);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
                            .build());
            mechOps.openClaw();

            mechOps.raiseLift(params.LIFT_MIN_LOW);
            mechOps.armout();

            /**
             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .splineToLinearHeading(new Pose2d(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading), specimenPrePreScoringPosition.heading)
             .splineToLinearHeading(new Pose2d(specimenPreScoringPosition.position, specimenPreScoringPosition.heading), specimenPreScoringPosition.heading)
             .splineToLinearHeading(new Pose2d(specimenScoringPosition.position, specimenScoringPosition.heading), specimenScoringPosition.heading)
             .build());

            //          Lower Lift







             // Grab the specimen
             if(opModeIsActive()) {
             safeWaitSeconds(1);
             }

             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
             .build());

             // Raise Arm to high basket scoring position
             if(opModeIsActive()) {
             safeWaitSeconds(1);
             // TODO: Add code to raise claw to specimen high bar
             }


             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
             .build());



             // Score the specimen on the high bar
             // Lower the arm
             if(opModeIsActive()) {
             // TODO: Add code to score the specimen
             }


             // Push Color Sample1 into the Observation area
             // Drive to color sample1 Position
             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
             .build());

             // Grab the specimen
             if(opModeIsActive()) {
             // TODO: Add code to grab the specimen from the observation area (from the floor)
             }

             // Raise Arm to high basket scoring position
             if(opModeIsActive()) {
             // TODO: Add code to raise claw to specimen high bar
             }

             // Drive to specimen scoring position
             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
             .build());

             // Score the specimen on the high bar
             // Lower the arm
             if(opModeIsActive()) {
             // TODO: Add code to score the specimen
             }

             // Drive to colored Sample3 Position
             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
             .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
             .strafeToLinearHeading(coloredSample3Position.position, coloredSample3Position.heading)
             .build());

             // Push Color Sample3 into the Observation area
             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
             .build());

             // Grab the specimen
             if(opModeIsActive()) {
             // TODO: Add code to grab the specimen from the observation area (from the floor)
             }

             // Raise Arm to high basket scoring position
             if(opModeIsActive()) {
             // TODO: Add code to raise claw to specimen high bar
             }

             // Drive to specimen scoring position
             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
             .build());

             // Score the specimen on the high bar
             // Lower the arm
             if(opModeIsActive()) {
             // TODO: Add code to score the specimen
             }

             // Park
             if(opModeIsActive()) {
             // TODO: Add code to park
             // set claw and motors into correct position
             }

             Actions.runBlocking(
             drive.actionBuilder(drive.pose)
             .strafeToLinearHeading(parkPose.position, parkPose.heading)
             .build());
             **/

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(new Pose2d(parkPose.position, parkPose.heading), parkPose.heading)
                            .build());
        }
    }

    /**
     *
     */

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        State setupConfig = State.START_POSITION;
        Boolean menuActive = true;

        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested() && menuActive){
            switch(setupConfig){
                case START_POSITION:
                    telemetry.addData("Initializing Autonomous:",
                            TEAM_NAME, " ", TEAM_NUMBER);
                    telemetry.addData("---------------------------------------","");
                    telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
                    telemetry.addData("    Blue Yellow Samples   ", "(X / ▢)");
                    telemetry.addData("    Blue Specimens ", "(Y / Δ)");
                    telemetry.addData("    Red Yellow Samples    ", "(B / O)");
                    telemetry.addData("    Red Specimens  ", "(A / X)");

                    if(gamepad1.dpad_left){
                        startPosition = START_POSITION.BLUE_SPECIMENS;
                        menuActive = false;
                    }

                    if(gamepad1.dpad_right){
                        startPosition = START_POSITION.RED_SPECIMENS;
                        menuActive = false;
                    }
                    telemetry.update();
                    break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public enum State {
        START_POSITION,
        PARK_POSITION
    }

}   // end class
