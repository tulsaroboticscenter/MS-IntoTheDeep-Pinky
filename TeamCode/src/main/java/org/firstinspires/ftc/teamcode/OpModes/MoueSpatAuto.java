package org.firstinspires.ftc.teamcode.OpModes;
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



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;

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

@Autonomous(name="Robot: MoueSpatAuto", group="Robot")
public class MoueSpatAuto extends LinearOpMode {


    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    public final static MSParams params = new MSParams();
    private DistanceSensor sensorColorRange;
    //       private Servo servoTest;
    private final boolean pad2input = true;

    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double StrafeSpeed = 1;
    int mBase = 900;

    private boolean IsOverrideActivated = false;

    public void runOpMode() {
        robot.init(hardwareMap, true);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        robot.servoClaw.setPosition(params.CLAW_CLOSE);
        robot.servoSpice.setPosition(params.SPICE_CLOSE);
        robot.servoWrist.setPosition(params.Wrist_Up);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoBar.setPosition(params.Bar_Middle);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoBucket.setPosition(params.Bucket_Catch);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
       // sleep(14000);
        // raise slides
        mBase = Range.clip(mBase,params.LIFT_MIN_LOW,params.LIFT_MAX_HIGH);
        drive.liftPosition(mBase);
//Straff to side
        //robot.motorRR.setPower(-1);
     //   robot.motorRF.setPower(1);
      //  robot.motorLF.setPower(-1);
    //    robot.motorLR.setPower(1);
      //  sleep(1000);
       // drive.motorsHalt();
        //wait for slides
        sleep(1000);
        //drive to goal
        robot.motorRR.setPower(-1);
        robot.motorRF.setPower(-1);
        robot.motorLF.setPower(-1);
        robot.motorLR.setPower(-1);
        //wait
        sleep(500);
        //stop
        robot.motorRR.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        //wait
        sleep(3000);
        //raise slides
        mBase = 1900;
        mBase = Range.clip(mBase,params.LIFT_MIN_LOW,params.LIFT_MAX_HIGH);
        drive.liftPosition(mBase);
        sleep(1500);
        //release spice
        robot.servoSpice.setPosition(params.SPICE_OPEN);
        //lower slides
        mBase = 400;
        mBase = Range.clip(mBase,params.LIFT_MIN_LOW,params.LIFT_MAX_HIGH);
        drive.liftPosition(mBase);
        //wait for slides
        sleep(1000);
        //move forward
        robot.motorRR.setPower(1);
        robot.motorRF.setPower(1);
        robot.motorLF.setPower(1);
        robot.motorLR.setPower(1);
        sleep(500);
        //stop
        robot.motorRR.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        sleep(400);
        //stafe
        robot.motorRR.setPower(-1);
        robot.motorRF.setPower(1);
        robot.motorLF.setPower(-1);
        robot.motorLR.setPower(1);
        sleep(1000);
        drive.motorsHalt();
        //Lower bucket to zero

        mBase = 0;
        mBase = Range.clip(mBase,params.LIFT_MIN_LOW,params.LIFT_MAX_HIGH);
        drive.liftPosition(mBase);
        sleep(1000);

    }
}
