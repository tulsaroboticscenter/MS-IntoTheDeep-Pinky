package org.firstinspires.ftc.teamcode.Hardware;

public class MSParams {
    /*
     * Constants
     */
    public final double DRIVE_TICKS_PER_INCH = 24;
    public final double STRAFE_FACTOR = 0.9;

    public final double ALIGN_ARM_GOLD = 0.45;
    public final double ALIGN_ARM_DOWN = 0.56;
    public final double ALIGN_ARM_MOVE = 0.2;
    public final double ALIGN_ARM_SCORE = 0.87;
    public final double ALIGN_ARM_HIGH = 0.0;

    //extend
    public final double ExtendRight_IN = 0;
    public final double ExtendRight_OUT = 1;
    public final double ExtendRight_CATCH = 0.7;
    public final double Extend_Catch = 0.3;
    public final double Extend_IN = 1;
    public final double Extend_OUT = 0;

    //bar
    public final double Bar_Up = 0.25;//22
    public final double Bar_Down = 0.62;//59
    public final double Bar_Middle = 0.16;
    public final double Bar_Auto = .33;//3


    //buckets
    public final double Bucket_Down = 0.1;
    public final double Bucket_Catch = 0.1;
    public final double Bucket_Dump = 0.80; //0.70

    //LIFT postions
    public final int LIFT_RESET = 0;
    public final int LIFT_MIN_LOW = 0;
    public final int LIFT_CLIP_HIGH = 1400;
    public final int LIFT_CLIP_SCORE = 1050;
    public final int LIFT_CLIP_LOW = 1050;
    public final int LIFT_MAX_HIGH = 3650;
    public final double LIFT_POWER = 0.5;
    public final int LIFT_Floor = 200;

    public final int LIFT_Top_B = 1950 ;
    public final int LIFT_Bottom_B = 401;

    public final int LIFT_Auto_Park =910;

    // ARM LIFTS

    public final double ANGLE_POWER = 0.5;
    public final int ANGLE_Wall = 50;
    public final int ANGLE_Sub_High = 100;
    public final int ANGLE_Floor = 10;
    public final int ANGLE_MAX_HIGH =1372;
    public final int ANGLE_MIN_LOW = 10;
    public final int ANGLE_Climb =50;
    public final int ANGLE_High_Bucket = 200;

    //wrist camands
    public final double Wrist_Up = .98;
    public final double Wrist_Release = 0.4;
    public final double Wrist_Down = 0.1;
    public final double Wrist_Auto = 0.5; //45
    public final double Wrist_Box = 0.68; //45

//Claw camands
    public final double CLAW_OPEN = 0.65;
    public final double CLAW_CLOSE = 0.33;

    //twist camands
    public final double TWIST_HORIZONTAL = 0;
    public final double TWIST_VERTICAL = 0.6;

    //Specimen claw
    public final double SPICE_OPEN = 0.3;
    public final double SPICE_CLOSE = .57;

    public final double CLAWROTATION2_DOWN = 0;
    public final double CLAWROTATION1_DOWN = 1;
    public final double CLAWROTATION2_UP = 1;
    public final double CLAWROTATION1_UP = 0;
}
