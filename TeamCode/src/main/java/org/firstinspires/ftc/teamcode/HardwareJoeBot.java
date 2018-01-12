package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2017 FTC Relic Recovery challenge. This file has been generalized to work for both JoeBots
 * teams FTC 11855 and 13702.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor1 (left front)
 * motor2 (right front)
 * motor3 (left rear)
 * motor4 (right rear)
 * liftmotor (on the Modern Robotics controller) - lift glyph mechanism up/down
 * clampservo - open/close clamp
 * clamprotate - rotate clamp mechanism up/down
 * jewelservo - rotate jewel arm up/down
 * imu - navigation features
 * jewelsensor - detect jewel colors
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class HardwareJoeBot
{
    /* Public OpMode members. */

    // Declare Motors
    public DcMotor  motor1 = null; // Left Front
    public DcMotor  motor2 = null; // Right Front
    public DcMotor  motor3 = null; // Left Rear
    public DcMotor  motor4 = null; // Right Rear
    public DcMotor  liftMotor = null;

    // Declare Servos
    public Servo    clampServo = null;  // open/close clamp
    public Servo    clampRotate = null; // rotate clamp up/down
    public Servo    jewelServo = null;  // rotate jewel arm up/down


    // Declare Sensors
    public BNO055IMU imu;                  // The IMU sensor object
    public ColorSensor jewelSensor = null; // Rev Robotics Color Sensor


    // Declare static values
    public static final double CLAMP_OPEN_POS = 0;
    public static final double CLAMP_CLOSE_POS = 1;
    public static final double CLAMP_DOWN_POS = 0.45;
    public static final double CLAMP_UP_POS = .15; //This position ".25" is for 1813 to fix the consistent flicking motion during "init" faz

    public static final double CLAMP_MID_POS = .5;
    public static final double JEWEL_ARM_UP_POS = 0.75;
    public static final double JEWEL_ARM_DOWN_POS = 0.09;
    public static final int LIFT_STARTING_POS = 0;
    public static final int LIFT_GLYPH_ONE_POS = 2400;
    public static final int LIFT_GLYPH_TWO_POS = 4800;
    public static final int LIFT_SEARCHING_POS = 2250;
    public static final int LIFT_SEARCHING_POS2 = 4300;
    public static final double DRIVE_SPEED = .3;

    // Define static min/max for lift
    public static final int LIFT_MIN_POSITION = 0;
    public static final int LIFT_MAX_POSITION = 4800;


    // Variables used for tracking mechanism state
    public boolean bClampOpen = false;
    public boolean bClampDown = false; //Is the clamp Rotated Down?
    public boolean bLiftRaised = false;
    public boolean bJewelArmUp = false;
    public boolean xClampOpen = false;

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;

    /* Constructor */
    public HardwareJoeBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        motor1 = hwMap.dcMotor.get("motor1");
        motor2 = hwMap.dcMotor.get("motor2");
        motor3 = hwMap.dcMotor.get("motor3");
        motor4 = hwMap.dcMotor.get("motor4");
        liftMotor = hwMap.dcMotor.get("liftmotor");

        jewelSensor = hwMap.get(ColorSensor.class, "jewelsensor");
        // Set Default Motor Directions
        motor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor4.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        liftMotor.setPower(0);

        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set lift motor to run using encoder...
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the servos
        clampServo = hwMap.servo.get("clampservo");
        clampRotate = hwMap.servo.get("clamprotate");
        jewelServo = hwMap.servo.get("jewelservo");


        // IMU Initializaiton
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize the Jewel Sensor
        jewelSensor = hwMap.get(ColorSensor.class, "jewelsensor");


        //Set servos to start position
        this.raiseClamp();
        this.closeClamp();
        this.raiseJewelArm();



    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    /**
     *
     * openClamp sets the servos to the open Position and requires no parameters
     *
     */

    public void openClamp() {

        // Set both clamps to open position;
        clampServo.setPosition(CLAMP_OPEN_POS);
        bClampOpen = true;

    }

    /**
     *
     * openClamp sets the servos to the open Position and requires no parameters
     *
     */

    public void closeClamp() {

        // Set both clamps to open position;
        clampServo.setPosition(CLAMP_CLOSE_POS);
        bClampOpen = false;
    }



    public void  closeClampMid() {
        //set clamp to "mid" Pos
        clampServo.setPosition(CLAMP_MID_POS);
        bClampOpen = false;
    }

    /**
     *
     * raiseJewelArm rotates jewelServo to Up Position
     *
     */
    public void raiseJewelArm() {

        jewelServo.setPosition(JEWEL_ARM_UP_POS);
        bJewelArmUp = true;

    }

    /**
     *
     * lowerJewelArm rotates jewelServo to Up Position
     *
     */
    public void lowerJewelArm() {

        jewelServo.setPosition(JEWEL_ARM_DOWN_POS);
        bJewelArmUp = false;

    }

    /**
     *
     * raiseClamp rotates clampRotate Servo to Up Position
     *
     */
    public void raiseClamp() {

        clampRotate.setPosition(CLAMP_UP_POS);
        bClampDown = false;

    }

    /**
     *
     * raiseClamp rotates clampRotate Servo to Up Position
     *
     */
    public void lowerClamp() {

        clampRotate.setPosition(CLAMP_DOWN_POS);
        bClampDown = true;

    }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        motor1.setMode(mode);
        motor2.setMode(mode);
        motor3.setMode(mode);
        motor4.setMode(mode);
    }



}






