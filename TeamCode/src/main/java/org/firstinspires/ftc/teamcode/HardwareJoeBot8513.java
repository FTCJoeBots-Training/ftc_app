package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * 8513 Not to be used for 11855,1703.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * motor1 (left front)
 * motor2 (right front)
 * motor3 (left rear)
 * motor4 (right rear)
 * liftmotor (on the Modern Robotics controller) - lift glyph mechanism up/down
 * clampleft - left half of clamping mechanism
 * clampright - right half of clamping mechanism
 * jewelservo - rotate jewel arm up/down
 * imu - navigation features
 * jewelsensor - detect jewel colors
 *
 */


public class HardwareJoeBot8513
{
    /* Public OpMode members. */
    public DcMotor  motor1 = null; // Left Front
    public DcMotor  motor2 = null; // Right Front
    public DcMotor  motor3 = null; // Left Rear
    public DcMotor  motor4 = null; // Right Rear
    public DcMotor  liftMotor = null;

    public Servo    clampLeft = null; // left Side of Clamp
    public Servo    clampRight = null; // right side of clamp
    public Servo    jewelServo = null; // Jewel Arm

    public static final double RIGHT_CLAMP_OPEN_POS = 0.4;
    public static final double RIGHT_CLAMP_CLOSE_POS = 0.7;
    public static final double LEFT_CLAMP_OPEN_POS = 0.4;
    public static final double LEFT_CLAMP_CLOSE_POS = 0.1;

    // Define static min/max for lift
    public static final int LIFT_MIN_POSITION = 0;
    public static final int LIFT_MAX_POSITION = 3810;
    public static final int LIFT_PLATFORM_POS = 1000;
    public static final double JEWEL_ARM_UP_POS = 0;
    public static final double JEWEL_ARM_DOWN_POS = 0.75;
    public static final int LIFT_STARTING_POS = 500;
    public static final int LIFT_GLYPH_ONE_POS = 2400;
    public static final int LIFT_GLYPH_TWO_POS = 4180;


    public ColorSensor jewelSensor = null; // Rev Robotics Color Sensor

    public boolean bClampOpen = false;
    public boolean bLiftRaised = false;
    public boolean bJewelArmUp = false;


    // The IMU sensor object
    public BNO055IMU imu;

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;


    /* Constructor */
    public HardwareJoeBot8513(){

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

        motor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor4.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.REVERSE);



        // Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the servos
        clampLeft = hwMap.servo.get("clampleft");
        clampRight = hwMap.servo.get("clampright");
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



        // Raise the JewelArm
        // close the clamps
        this.raiseJewelArm();
        this.closeClamp();




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
        clampLeft.setPosition(LEFT_CLAMP_OPEN_POS);
        clampRight.setPosition(RIGHT_CLAMP_OPEN_POS);
        bClampOpen = true;

    }

    /**
     *
     * openClamp sets the servos to the open Position and requires no parameters
     *
     */

    public void closeClamp() {

        // Set both clamps to open position;
        clampLeft.setPosition(LEFT_CLAMP_CLOSE_POS);
        clampRight.setPosition(RIGHT_CLAMP_CLOSE_POS);
        bClampOpen = false;

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

}

