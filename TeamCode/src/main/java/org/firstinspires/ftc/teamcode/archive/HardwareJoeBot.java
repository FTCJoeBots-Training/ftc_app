package org.firstinspires.ftc.teamcode.archive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "motor-driveleft"
 * Motor channel:  Right drive motor:        "rightmotor"

 * Servo channel:  Servo for left wing:      "srv-left"
 * Servo channel:  Servo for right wing:     "srv-right"
 */

public class HardwareJoeBot
{
    /* Public OpMode members. */
    public DcMotor                      leftmotor          = null;
    public DcMotor                      rightmotor          = null;
    
    public BNO055IMU imu;

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;

    // public Servo                        srv_left          = null;
    //public Servo                        srv_right         = null;



    //public static final double RIGHT_SERVO_OUT   =  0.25 ;
    //public static final double RIGHT_SERVO_IN   =  0.05 ;
    //public static final double LEFT_SERVO_IN   =  0.75 ;
    //public static final double LEFT_SERVO_OUT   =  0.55 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareJoeBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
       leftmotor   = hwMap.dcMotor.get("leftmotor");
        rightmotor= hwMap.dcMotor.get("rightmotor");



        leftmotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightmotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors





        // Set all motors to zero power
        rightmotor.setPower(0);
        leftmotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //srv_left = hwMap.servo.get("srv-left");
        //srv_right = hwMap.servo.get("srv-right");
        //srv_left.setPosition(LEFT_SERVO_OUT);
        //srv_right.setPosition(RIGHT_SERVO_OUT);
        //srv_left.setPosition(LEFT_SERVO_IN);
        //srv_right.setPosition(RIGHT_SERVO_IN);
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
}

