/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="red 2 8513", group="Testing")
//Disabled
public class r2test8513 extends LinearOpMode {

    /* Declare OpMode members. */
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
    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    double dublheading=0.0;
    long intheading=0;
    @Override
    public void runOpMode() throws InterruptedException {

        /*4
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at %7d :%7d",
        //                  robot.motor1.getCurrentPosition(),
        //                  robot.motor2.getCurrentPosition());


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Phone is facing the vu mark
        //Find it using compass
        //Read the vu mark
        //encoder drive to cryptobox(based on vu mark
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading: %7d", robot.angles);

        /*robot.lowerJewelArm();
        Color.RGBToHSV((int) (robot.jewelSensor.red() * SCALE_FACTOR),
                (int) (robot.jewelSensor.green() * SCALE_FACTOR),
                (int) (robot.jewelSensor.blue() * SCALE_FACTOR),
                hsvValues);
        if (robot.jewelSensor.red() >= 28) {
            encoderDrive(DRIVE_SPEED, -4.0, -4.0, 30);
            encoderDrive(DRIVE_SPEED, 4.0, 4.0, 30);
        }
        else {
            encoderDrive(DRIVE_SPEED, 4.0, 4.0, 30);

        }
        ?*/  robot.raiseJewelArm();
        encoderDrive(DRIVE_SPEED, 52.0, 52.0, 30);
            headingturn('l',90 );
            stopmotors();
            encoderDrive(DRIVE_SPEED, 30.0, 30.0, 30);
        telemetry.addLine("turning right to 0");
        telemetry.update();
        headingturn('r',17);
        telemetry.addLine("turning complete");
        telemetry.update();
        stopmotors();
        encoderDrive(DRIVE_SPEED, 16.0, 16.0, 30);
        robot.openClamp();
        sleep(750);
        robot.closeClamp();
        encoderDrive(DRIVE_SPEED, -15.0, -15.0, 30);
        headingturn('l',120 );
        /*encoderDrive(DRIVE_SPEED, -260, -260.0, 30);
            headingturn('l', 270);
            encoderDrive(DRIVE_SPEED, -346.0, -346.0, 30);*/


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void headingturn (char leftorright,int targetheading)

    {
        double _dblheading=181.0;
        long _intheading=181;
        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //robot.angles =robot.imu.getAngularOrientation();
        //heading = formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
        _dblheading= robot.angles.firstAngle;
        _intheading= Math.round(dublheading);

        while (opModeIsActive() && (_intheading!=targetheading))
        {
            robot.angles =robot.imu.getAngularOrientation();
            //heading = formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
            _dblheading= robot.angles.firstAngle;
            _intheading= Math.round(_dblheading);
            telemetry.addData("heading: %7d ",_intheading);
            telemetry.update();
            if (leftorright=='l') {
                robot.motor1.setPower(-.2);
                robot.motor2.setPower(.2);
                robot.motor3.setPower(-.2);
                robot.motor4.setPower(.2);
            } else {
                robot.motor1.setPower(.2);
                robot.motor2.setPower(-.2);
                robot.motor3.setPower(.2);
                robot.motor4.setPower(-.2);
            }



        }



        telemetry.addData("heading: %7d",robot.angles.firstAngle);
        telemetry.update();


    }














public void stopmotors()
    {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }
    /*

     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
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

            // Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
