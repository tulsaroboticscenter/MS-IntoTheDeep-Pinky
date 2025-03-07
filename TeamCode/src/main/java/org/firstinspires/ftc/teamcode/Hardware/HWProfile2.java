package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


public class HWProfile2 {


    /*
     * Constants
     */

    public final double PIVOT_SPEED = 0.5;
    public final double COUNTS_PER_ROTATION = 28;
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value
    public final double MIN_PIDROTATE_POWER = 0.10;


    /*
     *  Constants & variables for wheel parameters
     */

    public final double DRIVE_TICKS_PER_INCH = 32;

    public final double STRAFE_FACTOR = 0.9;

    public final double RIGHT_DRIVE_CORRECTION_MULTIPLIER = 1.4;
    public final double LEFT_DRIVE_CORRECTION_MULTIPLIER = 1.2;

    public final double MAX_DRIVING_POWER = 1;

    public double MIN_STRAFE_POWER = 0.35;

    public final double PID_Kp = 0.08;
    public final double PID_Ki = 0.01;
    public final double PID_Kd = 0.000001;
    public final double PID_MIN_SPEED = 0.05;
    public final double PID_ROTATE_ERROR = 1;

    public final double DRIVE_Kp = 0.05;
    public final double DRIVE_Ki = 0.01;
    public final double DRIVE_Kd = 0.31;

    /*
     * Hardware devices
     */

//    public RevIMU imu = null;
    public IMU imu;

    public DcMotorEx motorLF;
    public DcMotorEx motorLR;
    public DcMotorEx motorRF;
    public DcMotorEx motorRR;

    public DcMotorEx motorLift;

    public Servo servoWrist;
    public Servo servoBar;
    public Servo servoExtend;
    public Servo servoExtendRight;
    public Servo servoBucket;
    public Servo servoClaw;
    public Servo servoTwist;
    public Servo servoSpice;

    public GoBildaPinpointDriverRR pinpoint; // pinpoint CH i2C port 1



//    public MecanumDrive mecanum = null;

    HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public HWProfile2() {
    }

    public void init(HardwareMap ahwMap, boolean teleop) {

        hwMap = ahwMap;
        if(teleop) {

            motorLF = ahwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setDirection(DcMotor.Direction.REVERSE);
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorLF.setPower(0);
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorLR = ahwMap.get(DcMotorEx.class, "motorLR");
            motorLR.setDirection(DcMotor.Direction.REVERSE);
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorLR.setPower(0);
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorRF = ahwMap.get(DcMotorEx.class, "motorRF");
            motorRF.setDirection(DcMotor.Direction.FORWARD);
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRF.setPower(0);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorRR = ahwMap.get(DcMotorEx.class, "motorRR");
            motorRR.setDirection(DcMotor.Direction.FORWARD);
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRR.setPower(0);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Retrieve and initialize the IMU.
            // This sample expects the IMU to be in a REV Hub and named "imu".
            imu = ahwMap.get(IMU.class, "imu");
            pinpoint = hwMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
            pinpoint.recalibrateIMU();
            pinpoint.resetPosAndIMU();

            /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
             *
             * Two input parameters are required to fully specify the Orientation.
             * The first parameter specifies the direction the printed logo on the Hub is pointing.
             * The second parameter specifies the direction the USB connector on the Hub is pointing.
             * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
             *
             * If you are using a REV 9-Axis IMU, you can use the Rev9AxisImuOrientationOnRobot class instead of the
             * RevHubOrientationOnRobot class, which has an I2cPortFacingDirection instead of a UsbFacingDirection.
             */

            /* The next two lines define Hub orientation.
             * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
             *
             * To Do:  EDIT these two lines to match YOUR mounting configuration.
             */
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            // Now initialize the IMU with this mounting orientation
            // Note: if you choose two conflicting directions, this initialization will cause a code exception.
            imu.initialize(new IMU.Parameters(orientationOnRobot));

        }
        //drivebase init
//        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        motorLift = ahwMap.get(DcMotorEx.class, "motorLift");
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setPower(0);
        motorLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /**
         * Initialize Servos
         **/
        //servoGrabber = hwMap.servo.get("servoGrabber");
        // servoGrabber2 = hwMap.servo.get("servoGrabber2");
        servoClaw = ahwMap.servo.get("servoClaw");
        servoTwist = ahwMap.servo.get("servoTwist");
        servoWrist = ahwMap.servo.get("servoWrist");
        servoBar = ahwMap.servo.get("servoBar");
        servoExtend = ahwMap.servo.get("servoExtend");
        servoBucket = ahwMap.servo.get("servoBucket");
        servoExtendRight = ahwMap.servo.get("servoExtendRight");
        servoSpice = ahwMap.servo.get("servoSpice");

        // Zeroing Servos
        //servoIntake.setPower(0.5);
        //servoWrist.setPosition(0);
        //servoBar.setPosition(1);
        //servoExtend.setPosition(0);
        //servoBucket.setPosition(0.5);
        //servoExtendRight.setPosition(0);


        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }


    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }
}
