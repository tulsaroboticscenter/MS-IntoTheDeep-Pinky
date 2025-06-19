package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
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
    public void elbowMove( double elbow) {

        //elbow = Range.clip(elbow, params.LIFT_MIN_LOW, params.LIFT_MAX_HIGH);
        robot.servoClawRotation1.setPosition(elbow);
        robot.servoClawRotation2.setPosition(1-elbow);
    }

    public void raiseLift(int mBase) {
        mBase = Range.clip(mBase, params.LIFT_MIN_LOW, params.LIFT_MAX_HIGH);
        robot.motorLift.setPower(1);
        robot.motorLift.setTargetPosition(mBase);
        robot.motorLiftRight.setPower(1);
        robot.motorLiftRight.setTargetPosition(mBase);
    }

    public void SpiceScore(){
        anglePosition(params.ANGLE_Sub_High);
        liftPosition(params.LIFT_CLIP_HIGH);

    }
    public void SpiceScore2(){
        anglePosition(params.ANGLE_Sub_High);
        liftPosition(params.LIFT_CLIP_HIGH);
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_DOWN);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_DOWN);
        robot.servoExtendRight.setPosition(params.ExtendRight_OUT);
        robot.servoExtend.setPosition(params.Extend_OUT);
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
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_FLOOR);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_FLOOR);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoClaw.setPosition(params.CLAW_OPEN);
        robot.motorLEFT.setTargetPosition(params.ANGLE_Floor);
        robot.motorRIGHT.setTargetPosition(params.ANGLE_Floor);
        robot.motorLift.setTargetPosition(params.LIFT_SWEEP);
        robot.motorLiftRight.setTargetPosition(params.LIFT_SWEEP);




    }
    public void armin() {
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoBucket.setPosition(params.Bucket_Down);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoClaw.setPosition(params.CLAW_OPEN);
        robot.motorLEFT.setTargetPosition(params.ANGLE_Floor);
        robot.motorRIGHT.setTargetPosition(params.ANGLE_Floor);
        robot.motorLift.setTargetPosition(params.LIFT_RESET);
        robot.motorLiftRight.setTargetPosition(params.LIFT_RESET);


    }

    public void liftPosition(int targetPosition){
        robot.motorLift.setPower(params.LIFT_POWER);
        robot.motorLiftRight.setPower(params.LIFT_POWER);
        robot.motorLift.setTargetPosition(targetPosition);
        robot.motorLiftRight.setTargetPosition(targetPosition);
    }   // end of liftPosition method
    
    public void anglePosition(int targetPosition){
        robot.motorRIGHT.setPower(params.ANGLE_POWER);
        robot.motorLEFT.setPower(params.ANGLE_POWER);
        robot.motorRIGHT.setTargetPosition(targetPosition);
        robot.motorLEFT.setTargetPosition(targetPosition);
    }
    public void armoutGold() {
        robot.servoExtendRight.setPosition(params.ExtendRight_OUT);
        robot.servoExtend.setPosition(params.Extend_OUT);
        //robot.servoBar.setPosition(params.Bar_Down);
        //robot.servoClawRotation1.setPosition(params.CLAWROTATION1_DOWN);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_DOWN);
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
        //robot.servoWrist.setPosition(params.Wrist_Up);
        opMode.sleep(1000);
        robot.servoClaw.setPosition(params.CLAW_OPEN);
        robot.servoBar.setPosition(params.Bar_Auto);
      //  robot.servoWrist.setPosition(params.Wrist_Auto);
    }

    public void BucketReset() {
        //robot.servoWrist.setPosition(params.Wrist_Release);
        robot.servoBar.setPosition(params.Bar_Auto);
        robot.servoBucket.setPosition(params.Bucket_Catch);
    }


    public void AutoDump(){
       // robot.servoWrist.setPosition(params.Wrist_Release);

       // robot.servoBar.setPosition(params.Bar_Auto);
       // opMode.sleep(300);
        liftPosition(params.LIFT_Top_B);
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

    public void WallGrab(){
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_DOWN);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_DOWN);
        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
        robot.servoExtend.setPosition(params.Extend_IN);
        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
        anglePosition(params.ANGLE_Wall);
        liftPosition(params.LIFT_WALL);

    }

    public void PreSweep(){
        liftPosition(params.LIFT_Floor);
        anglePosition(params.ANGLE_AUTO);
        robot.servoExtend.setPosition(params.Extend_BARCLEAR);
        robot.servoExtendRight.setPosition(params.ExtendRight_BARCLEAR);
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_FLOOR);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_FLOOR);
    }
    public void Sweep(){
        liftPosition(params.LIFT_Floor);
        anglePosition(params.ANGLE_SWEEP);
        robot.servoExtend.setPosition(params.Extend_FLOOR);
        robot.servoExtendRight.setPosition(params.ExtendRight_FLOOR);
        robot.servoClawRotation1.setPosition(params.CLAWROTATION1_FLOOR);
        robot.servoClawRotation2.setPosition(params.CLAWROTATION2_FLOOR);
    }
}
