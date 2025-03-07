/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.MSMechOps;
//import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;

import java.util.Locale;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: TeleOp POV", group="Robot")
public class RobotTeleOp extends LinearOpMode {

    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
 //   public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
 public final static MSParams params = new MSParams();
    public MSMechOps mechOps = new MSMechOps(robot, opMode, params);

    private double testPosition = 0;

    public void runOpMode() {
        robot.init(hardwareMap, true);
        robot.pinpoint.recalibrateIMU();
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        robot.servoClaw.setPosition(params.CLAW_CLOSE);
        robot.servoWrist.setPosition(params.Wrist_Up);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoBar.setPosition(params.Bar_Up);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoBucket.setPosition(params.Bucket_Catch);
        robot.pinpoint.recalibrateIMU();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double leftPower = 0;
        double rightPower = 0;
        double clawPosition = params.CLAW_CLOSE;
        double spicePosition = params.SPICE_CLOSE;
        double TwistPosition = params.TWIST_HORIZONTAL;
        ElapsedTime buttonPressTimer = new ElapsedTime();
        boolean clawOpen = false;
        double botHeading;
        double x, y, rx;
        double rotX, rotY;
        double denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double powerFactor = 1;

        int mBase = params.LIFT_RESET;
        while (opModeIsActive()) {

            /* ###########################################
               #                                         #
               # New field centric drive using PinPoint  #
               #                                         #
               ###########################################*/

            y = -gamepad1.left_stick_y + gamepad1.right_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.pinpoint.recalibrateIMU();
                //recalibrates the IMU without resetting position
            }

            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Position", data);

            botHeading = 0;             //
            //botHeading = pos.getHeading(AngleUnit.RADIANS);

           rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
           rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
          rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;

            robot.motorLF.setPower(frontLeftPower * powerFactor);
            robot.motorLR.setPower(backLeftPower * powerFactor);
            robot.motorRF.setPower(frontRightPower * powerFactor);
            robot.motorRR.setPower(backRightPower * powerFactor);

            if (gamepad1.y) {
                // What Happens when we hit Y Triangle - Dump to transfer
                robot.servoExtendRight.setPosition(params.ExtendRight_CATCH);
                robot.servoExtend.setPosition(params.Extend_Catch);
               // robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
                TwistPosition = params.TWIST_HORIZONTAL;
                robot.servoBucket.setPosition(params.Bucket_Catch);
                robot.servoBar.setPosition(params.Bar_Up);
                robot.servoWrist.setPosition(params.Wrist_Up);
                mBase = params.LIFT_RESET;
            }   // end of if(gamepad1.y)

            if (gamepad1.x) {
                // Square

                robot.servoExtendRight.setPosition(params.ExtendRight_OUT);
                robot.servoExtend.setPosition(params.Extend_OUT);
                robot.servoBar.setPosition(params.Bar_Down);
                robot.servoWrist.setPosition(params.Wrist_Down);
                robot.servoBucket.setPosition(params.Bucket_Down);
                //robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
                TwistPosition = params.TWIST_HORIZONTAL;
                clawPosition = params.CLAW_OPEN;
                spicePosition = params.SPICE_OPEN;
                clawOpen = true;
                //robot.servoClaw.setPosition(params.CLAW_OPEN);

            }   // end of if(gamepad1.x)

            // A=X symbol
            if (gamepad1.a) {
                // X
                clawPosition = params.CLAW_OPEN;
                spicePosition = params.SPICE_OPEN;
                clawOpen = true;
                robot.servoWrist.setPosition(params.Wrist_Release);
                robot.servoBar.setPosition(params.Bar_Auto);
                robot.servoBucket.setPosition(params.Bucket_Dump);
            }

            if (gamepad1.b) {
                //circle
                robot.servoWrist.setPosition(params.Wrist_Release);
                robot.servoBar.setPosition(params.Bar_Auto);
                robot.servoBucket.setPosition(params.Bucket_Catch);

                mBase = params.LIFT_RESET;
            }

            if(gamepad1.dpad_left){
                robot.servoWrist.setPosition(params.Wrist_Release);
                robot.servoBar.setPosition(params.Bar_Auto);
                mBase = params.LIFT_CLIP_HIGH;
            }

            if(gamepad1.dpad_right){
                //robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
                mBase = params.LIFT_CLIP_SCORE;
            }

            if (gamepad1.dpad_up) {
                //robot.servoClaw.setPosition(params.CLAW_OPEN);
                clawPosition = params.CLAW_OPEN;
                spicePosition = params.SPICE_OPEN;
                clawOpen = true;
                robot.servoWrist.setPosition(params.Wrist_Release);
                robot.servoBar.setPosition(params.Bar_Auto);
                mBase = params.LIFT_Top_B;
            }

            if (gamepad1.dpad_down) {
                clawPosition = params.CLAW_OPEN;
                spicePosition = params.SPICE_OPEN;
                clawOpen = true;
                //robot.servoClaw.setPosition(params.CLAW_OPEN);
                robot.servoWrist.setPosition(params.Wrist_Release);
                mBase = params.LIFT_Bottom_B;
            }

            if (gamepad1.right_bumper) {
                if((buttonPressTimer.time() > 0.25) && clawOpen){
                    clawPosition = params.CLAW_CLOSE;
                    spicePosition = params.SPICE_CLOSE;
                    clawOpen = false;
                    buttonPressTimer.reset();
                } else if(buttonPressTimer.time() > 0.25) {
                    clawPosition = params.CLAW_OPEN;
                    spicePosition = params.SPICE_OPEN;
                    clawOpen = true;
                    buttonPressTimer.reset();
                }
            }

            if (gamepad1.left_bumper) {
                    if((buttonPressTimer.time() > 0.25) && TwistPosition == params.TWIST_HORIZONTAL){
                        TwistPosition = params.TWIST_VERTICAL;

                        buttonPressTimer.reset();
                    } else if(buttonPressTimer.time() > 0.25) {
                        TwistPosition = params.TWIST_HORIZONTAL;

                        buttonPressTimer.reset();
                    }
            }


            if (gamepad1.right_trigger>0.3) {
                mBase=mBase+3;
            }

            //Lower Slides
            if (gamepad1.left_trigger>0.3) {
                mBase=mBase-3;
            }

            //emergency down button
            if (gamepad2.left_trigger>0.3){
                mBase=mBase-3;
            }


            // limit the max and min value of mBase
            // robot.servoBar.setPosition(barPosition);
            robot.servoClaw.setPosition(clawPosition);
            robot.servoSpice.setPosition(spicePosition);
            robot.servoTwist.setPosition(TwistPosition);

            //if gamepad2 left trigger is active the range clip will not apply
            if(!(gamepad2.left_trigger>0.3)) {
                mBase = Range.clip(mBase, params.LIFT_MIN_LOW, params.LIFT_MAX_HIGH);
            }

            mechOps.liftPosition(mBase);

            telemetry.addData("Left Front Motor Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Left Front Motor Current = ", robot.motorLF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Rear Motor Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Left Rear Motor Current = ", robot.motorLR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Front Motor Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Right Front Motor Current = ", robot.motorRF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Rear Motor Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Right Rear Motor Current = ", robot.motorRR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("TestPosition = ", testPosition);
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Lift set point", mBase);

            telemetry.addData("Eli Pink Shirt", "yes");
            telemetry.update();
        }
    }
}
