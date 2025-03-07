package org.firstinspires.ftc.teamcode.Libs;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.MSParams;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.MSParams;

public class MSMechOps {


    public HWProfile2 robot;
    public LinearOpMode opMode;

    public MSParams params = new MSParams();

    /*
     * Constructor
     */
    public MSMechOps(HWProfile2 myRobot, LinearOpMode myOpMode, MSParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close RRMechOps constructor



    public void raiseLift(int mBase) {

        mBase = Range.clip(mBase, params.LIFT_MIN_LOW, params.LIFT_MAX_HIGH);
        robot.motorLift.setPower(1);
        robot.motorLift.setTargetPosition(mBase);
    }

    public void openClaw(){
        robot.servoSpice.setPosition(params.SPICE_OPEN);
    }

    public void closeClaw() {
        robot.servoSpice.setPosition(params.CLAW_CLOSE);
    }

    public void armout() {
        robot.servoExtendRight.setPosition(params.ExtendRight_OUT);
        robot.servoExtend.setPosition(params.Extend_OUT);
        robot.servoBar.setPosition(params.Bar_Down);
        robot.servoWrist.setPosition(params.Wrist_Down);
        robot.servoBucket.setPosition(params.Bucket_Down);
        //robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoTwist.setPosition(params.TWIST_VERTICAL);
        robot.servoClaw.setPosition(params.CLAW_OPEN);


    }
    public void armin() {
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoBar.setPosition(params.Bar_Auto);
        robot.servoWrist.setPosition(params.Wrist_Auto);
        robot.servoBucket.setPosition(params.Bucket_Down);
        //robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoClaw.setPosition(params.CLAW_OPEN);


    }
}
