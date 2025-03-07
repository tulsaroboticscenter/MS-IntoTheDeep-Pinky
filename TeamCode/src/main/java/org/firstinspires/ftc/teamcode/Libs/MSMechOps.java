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
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoExtend.setPosition(params.Extend_IN);
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

    public void liftPosition(int targetPosition){
        robot.motorLift.setPower(params.LIFT_POWER);
        robot.motorLift.setTargetPosition(targetPosition);
    }   // end of liftPosition method

    public void armoutGold() {
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoBar.setPosition(params.Bar_Down);
        robot.servoWrist.setPosition(params.Wrist_Down);
        robot.servoBucket.setPosition(params.Bucket_Down);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        //robot.servoTwist.setPosition(params.TWIST_VERTICAL);
        robot.servoClaw.setPosition(params.CLAW_OPEN);
    }
    public void transfer() {
        robot.servoClaw.setPosition(params.CLAW_CLOSE);
        opMode.sleep(500);
        robot.servoExtendRight.setPosition(params.ExtendRight_CATCH);
        robot.servoExtend.setPosition(params.Extend_Catch);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoBucket.setPosition(params.Bucket_Catch);
        robot.servoBar.setPosition(params.Bar_Up);
        robot.servoWrist.setPosition(params.Wrist_Up);
        opMode.sleep(1000);
        robot.servoClaw.setPosition(params.CLAW_OPEN);
        robot.servoBar.setPosition(params.Bar_Auto);
        robot.servoWrist.setPosition(params.Wrist_Auto);
    }

    public void BucketReset() {
        robot.servoWrist.setPosition(params.Wrist_Release);
        robot.servoBar.setPosition(params.Bar_Auto);
        robot.servoBucket.setPosition(params.Bucket_Catch);
    }


    public void AutoDump(){
       // robot.servoWrist.setPosition(params.Wrist_Release);
       // robot.servoBar.setPosition(params.Bar_Auto);
       // opMode.sleep(300);
        //liftPosition(params.LIFT_Top_B);
        opMode.sleep(1500);
        robot.servoBucket.setPosition(params.Bucket_Dump);
        opMode.sleep(1000);
        robot.servoBucket.setPosition(params.Bucket_Down);
        opMode.sleep(200);
        liftPosition(params.LIFT_RESET);
    }

    public void AutoSubPark(){
        robot.servoSpice.setPosition(params.SPICE_CLOSE);
        liftPosition(params.LIFT_Auto_Park);
        opMode.sleep(1500);
    }



}
