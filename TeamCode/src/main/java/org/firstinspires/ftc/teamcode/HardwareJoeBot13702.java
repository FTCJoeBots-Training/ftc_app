package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * motor1 (left front)
 * motor2 (right front)
 * motor3 (left rear)
 * motor4 (right rear)
 * liftmotor (on the Modern Robotics controller)
 * clampservo
 * clamprotate
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class HardwareJoeBot13702
{
    /* Public OpMode members. */
    public DcMotor  motor1 = null; // Left Front
    public DcMotor  motor2 = null; // Right Front
    public DcMotor  motor3 = null; // Left Rear
    public DcMotor  motor4 = null; // Right Rear
    public DcMotor  liftMotor = null;

    public Servo    clampServo = null; // left Side of Clamp
    public Servo    clampRotate = null; // right side of clamp

    public static final double CLAMP_OPEN_POS = 0;
    public static final double CLAMP_CLOSE_POS = 1;
    public static final double CLAMP_DOWN_POS = 1;
    public static final double CLAMP_UP_POS = 0;

    // Define static min/max for lift
    public static final int LIFT_MIN_POSITION = 0;
    public static final int LIFT_MAX_POSITION = 5760;


    public boolean bClampOpen = false;
    public boolean bClampDown = false; //Is the clamp Rotated Down?
    public boolean bLiftRaised = false;


    // The IMU sensor object
    public BNO055IMU imu;

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;


    // Create Enum for Directions
    public enum simpleDirection {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    }


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;

    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW
    private double integratedZAxis = 0;
    private double lastHeading = 0;

    /* Constructor */
    public HardwareJoeBot13702(){

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

        motor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor4.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
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
        clampServo = hwMap.servo.get("clampservo");
        clampRotate = hwMap.servo.get("clamprotate");

        // Open and close the clamps
        //this.closeClamp();
        //this.openClamp();
        //this.closeClamp();



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



    public double getIntegratedZAxis() {

        double newHeading;
        double deltaHeading;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        newHeading = angles.firstAngle;

        deltaHeading = newHeading - lastHeading;

        if (deltaHeading < -180)
            deltaHeading += 360 ;
        else if (deltaHeading >= 180)
            deltaHeading -= 360 ;

        integratedZAxis += deltaHeading;

        lastHeading = newHeading;

        return integratedZAxis;

    }

    public void resetZAxis() {

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastHeading = angles.firstAngle;
        integratedZAxis = 0;

    }

    /***
     *
     * turnDegrees Turns the robot to a heading based on feedback from the Rev IMU
     *
     * @param degreesToTurn Number of degrees to turn (+ turns clockwise).
     * @throws InterruptedException
     *
     *
     */

    public void turnDegrees(double degreesToTurn, double turnSpeed) throws InterruptedException {

        double turnError;
        double currHeading;
        double targetHeading;

        // Reset the Z Axis so turning is easier.
        resetZAxis();

        targetHeading = getIntegratedZAxis() + degreesToTurn;
        turnError = degreesToTurn;

        while (myOpMode.opModeIsActive() && Math.abs(turnError) > 5) {

            // Turn the robot
            if (turnError > 0) {
                // Turn Clockwise
                moveRobot(0,0,-turnSpeed);
            } else if (turnError < 0) {
                moveRobot(0,0,turnSpeed);
            } else {
                // turnError is 0
                moveRobot(0,0,0);
            }

            //Update turnError
            currHeading = getIntegratedZAxis();
            turnError = targetHeading - currHeading;

            //Update Telemetry
            myOpMode.telemetry.addData("Current Heading: ", "%5.2f", currHeading);
            myOpMode.telemetry.addData("Target Heading: ", "%5.2f", targetHeading);
            myOpMode.telemetry.addData("Turn Error: ", "%5.2f", turnError);
            myOpMode.telemetry.update();


        }

        //We should be within our tolerance
        moveRobot(0,0,0);




    }

    /**
     * void simpleDriveDistance()
     * this method will move the robot in one of four directions (forward, reverse, left, or
     * right) at a fixed speed for a set distance (hopefully).
     *
     * @param direction     ordinal direction
     * @param speed         motor speed
     * @param distance      distance in inches
     *
     */
    public void simpleDriveDistance(simpleDirection direction, double speed, double distance ){




        //
        // According to RobotDrive.mecanumDrive_Cartesian in WPILib:
        //
        // LF =  x + y + rot    RF = -x + y - rot
        // LR = -x + y + rot    RR =  x + y - rot
        //
        // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
        // => (LF + RR) - (RF + LR) = 4x
        // => x = ((LF + RR) - (RF + LR))/4
        //
        // LF + RF + LR + RR = 4y
        // => y = (LF + RF + LR + RR)/4
        //
        // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
        // => (LF + LR) - (RF + RR) = 4rot
        // => rot = ((LF + LR) - (RF + RR))/4
        //

        /**
         * Based on the above information, we want to calculate the target x or y position
         * and then run the motors at the set speed and in the correct direction until the
         * encoders reach the target position
         *
         */


        double lfEnc = 0.0;
        double lrEnc = 0.0;
        double rfEnc = 0.0;
        double rrEnc = 0.0;
        double xPos = 0.0;
        double yPos = 0.0;
        double rotPos = 0.0;
        double targetXPos = 0.0;
        double targetYPos = 0.0;
        boolean atTarget = false;

        lfEnc = motor1.getCurrentPosition();
        lrEnc = motor3.getCurrentPosition();
        rfEnc = motor2.getCurrentPosition();
        rrEnc = motor4.getCurrentPosition();


        //Calculate Target positions
        switch (direction) {

            case LEFT :
                targetXPos = xPos - distance;
                targetYPos = yPos;

            case RIGHT :
                targetXPos = xPos + distance;
                targetYPos = yPos;

            case FORWARD:
                targetXPos = xPos;
                targetYPos = yPos + distance;

            case REVERSE:
                targetXPos = xPos;
                targetYPos = yPos - distance;

        }

        while (myOpMode.opModeIsActive() && !atTarget){



            // Move in the desired direction

            switch (direction) {

                case LEFT :
                    moveRobot(0,-speed,0);

                case RIGHT :
                    moveRobot(0,speed,0);

                case FORWARD:
                    moveRobot(speed,0,0);

                case REVERSE:
                    moveRobot(-speed,0,0);

            }

            // Determine if we're close enough to the target Position
            if ((direction == simpleDirection.FORWARD) || (direction == simpleDirection.REVERSE)){
                // we don't care about xPos -- only yPos
                if (Math.abs(targetYPos - yPos)<0.1){
                    atTarget = true;
                }
            } else if ((direction == simpleDirection.LEFT) || (direction == simpleDirection.RIGHT)) {
                // we don't care about yPos, only xPos
                if (Math.abs(targetXPos - xPos)<0.1){
                    atTarget = true;
                }
            }

            //Update Encoders
            lfEnc = motor1.getCurrentPosition();
            lrEnc = motor3.getCurrentPosition();
            rfEnc = motor2.getCurrentPosition();
            rrEnc = motor4.getCurrentPosition();

            xPos = ((lfEnc + rrEnc) - (rfEnc + lrEnc))/4.0;
            yPos = (lfEnc + lrEnc + rfEnc + rrEnc)/4.0;
            rotPos = ((lfEnc + lrEnc) - (rfEnc + rrEnc))/4.0;

            myOpMode.telemetry.addData("xPosition: ", "%5.2f", xPos);
            myOpMode.telemetry.addData("Target X Position: ", "%5.2f", targetXPos);
            myOpMode.telemetry.addData("yPosition: ", "%5.2f", yPos);
            myOpMode.telemetry.addData("Target Y Position: ", "%5.2f", targetYPos);
            myOpMode.telemetry.update();




        }

        // We've exited the while loop, so we must be near our target. Stop the robot.
        moveRobot(0,0,0);



    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);

    }


    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        double motor1Power = driveAxial + driveYaw + driveLateral;
        double motor2Power = driveAxial - driveYaw - driveLateral;
        double motor3Power = driveAxial + driveYaw - driveLateral;
        double motor4Power = driveAxial - driveYaw + driveLateral;


        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(motor1Power), Math.abs(motor2Power));
        max = Math.max(max, Math.abs(motor3Power));
        max = Math.max(max, Math.abs(motor4Power));
        if (max > 1.0)
        {
            motor1Power /= max;
            motor2Power /= max;
            motor3Power /= max;
            motor4Power /= max;
        }

        // Set drive motor power levels.
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);
        motor3.setPower(motor3Power);
        motor4.setPower(motor4Power);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "FL[%+5.2f], FR[%+5.2f], BL[%+5.2f], BR[%+5.2f]", motor1Power, motor2Power, motor3Power, motor4Power);
    }


    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


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

