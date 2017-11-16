package org.firstinspires.ftc.teamcode.archive;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "motor-driveleft"
 * Motor channel:  Right drive motor:        "motor-driveright"

 * Servo channel:  Servo for left wing:      "srv-left"
 * Servo channel:  Servo for right wing:     "srv-right"
 */

public class HardwareJoeBot11855
{
    /* Public OpMode members. */
    public DcMotor                      leftmotor           = null;
    public DcMotor                      rightmotor          = null;
    public DcMotor                      elevatormotor       = null;

    public Servo                        clampservo          = null;
    public Servo                        liftservo           = null;



    public static final double RIGHT_SERVO_OUT   =  0.25 ;
    public static final double RIGHT_SERVO_IN   =  0.05 ;
    public static final double LEFT_SERVO_IN   =  0.75 ;
    public static final double LEFT_SERVO_OUT   =  0.55 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareJoeBot11855(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftmotor   = hwMap.dcMotor.get("leftmotor");
        rightmotor  = hwMap.dcMotor.get("rightmotor");
        elevatormotor = hwMap.dcMotor.get("elevatormotor");

        leftmotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightmotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        elevatormotor.setDirection(DcMotor.Direction.REVERSE);


        //Define Sensors




        // Set all motors to zero power
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        elevatormotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatormotor.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        // Define and initialize ALL installed servos.
        clampservo = hwMap.servo.get("clampservo");
        liftservo = hwMap.servo.get("liftservo");


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

