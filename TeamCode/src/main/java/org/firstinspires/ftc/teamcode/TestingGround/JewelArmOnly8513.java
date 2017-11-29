package org.firstinspires.ftc.teamcode.TestingGround;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareJoeBot;
import org.firstinspires.ftc.teamcode.HardwareJoeBot8513;

import java.util.Locale;

@Autonomous(name="BlueOneJAO", group="auto")
public class JewelArmOnly8513 extends LinearOpMode {
    HardwareJoeBot8513 robot   = new HardwareJoeBot8513();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 540 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.1;
    String heading  ="";
    double dublheading=0.0;
    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    long intheading=0;

    @Override
    public void runOpMode() throws InterruptedException {




        robot.init(hardwareMap, this);

        waitForStart();

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading: %7d", robot.angles);

        // Close the clamp on the pre-loaded glyph
        robot.closeClamp();
        sleep(1000);


        // Raise the clamp to a safe driving height
        robot.liftMotor.setTargetPosition(robot.LIFT_GLYPH_ONE_POS);
        robot.liftMotor.setPower(.5);
        while (robot.liftMotor.isBusy()) {
            idle();
        }
        robot.liftMotor.setPower(0);

        // Read the VuMark and store the Key Column
        // Need to add the VuMark Code here...

        // Drop the jewel arm
        robot.lowerJewelArm();
        sleep(1000);

        // Read the color of the jewel in front of the jewel arm
        // Raise the jewel arm
        telemetry.addData("Blue: ", robot.jewelSensor.blue());
        telemetry.addData("Red: ", robot.jewelSensor.red());
        telemetry.update();


        telemetry.addData("Blue: ", robot.jewelSensor.blue());
        telemetry.addData("Red: ", robot.jewelSensor.red());


//Edited to BLue at 11/28/17
        // Based on the color of the jewel, rotate the bot either CW or CCW to knock off the right jewel
        if (robot.jewelSensor.red() > robot.jewelSensor.blue()) {
            //The sensor sees more Red than Blue, so the red jewel is "in front". Since this is
            //a "Red" opMode, we want to knock the blue jewel off the table.
            telemetry.addLine("Red Wins");
            //headingturn('r', -9);
       //     encoderDrive(.15,6,6,5);
            robot.raiseJewelArm();
       //     encoderDrive(.15,-6,-6,5);

        } else {
            telemetry.addLine("Blue Wins");
       //     encoderDrive(.15,-6,-6,5);
            robot.raiseJewelArm();
       //     encoderDrive(.15,6,6,5);
        }


        telemetry.update();




    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.motor3.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.motor4.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.motor1.setTargetPosition(newLeftTarget);
            robot.motor2.setTargetPosition(newRightTarget);
            robot.motor3.setTargetPosition(newLeftTarget2);
            robot.motor4.setTargetPosition(newRightTarget2);
            // Turn On RUN_TO_POSITION
            robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.motor1.setPower(Math.abs(speed));
            robot.motor2.setPower(Math.abs(speed));
            robot.motor3.setPower(Math.abs(speed));
            robot.motor4.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy() && robot.motor3.isBusy() && robot.motor4.isBusy())) {


                // Display it for the driver.
                telemetry.addData("heading: %7d ",robot.angles.firstAngle);
                telemetry.addData("target1",  "Running to %7s", newLeftTarget );
                telemetry.addData("target2",  "Running to %7s", newRightTarget );
                telemetry.addData("target3",  "Running to %7s", newLeftTarget2 );
                telemetry.addData("target4",  "Running to %7s", newRightTarget2 );
                telemetry.addData("wheel1",  "Running to %7d",  robot.motor1.getCurrentPosition());
                telemetry.addData("wheel2", "Running to %7d",  robot.motor2.getCurrentPosition());
                telemetry.addData("wheel3", "Running to %7d",  robot.motor3.getCurrentPosition());
                telemetry.addData("wheel4", "Running to %7d",  robot.motor4.getCurrentPosition());
                //robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                robot.angles =robot.imu.getAngularOrientation();
                heading = formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);

                telemetry.addData("heading: %7d ",robot.angles.firstAngle);


                telemetry.update();

                // Allow time for other processes to run.
                idle();
                }
            }









    }
}
