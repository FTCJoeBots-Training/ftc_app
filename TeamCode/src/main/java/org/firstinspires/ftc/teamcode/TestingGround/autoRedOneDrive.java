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
package org.firstinspires.ftc.teamcode.TestingGround;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareJoeBot;

import java.util.Locale;

/**
 *
 * This is a test autonomous opMode and assumes starting the "Red One" position (on the left
 * side of the playing field from the red driver area). The objective is to:
 *
 * Close the clamp on the pre-loaded glyph
 * Raise the clamp to a safe driving height
 * Read the VuMark and store the Key Column
 * Drop the jewel arm
 * Read the color of the jewel in front of the jewel arm
 * Based on the color of the jewel, rotate the bot either CW or CCW to knock off the right jewel
 * Raise the jewel arm
 * Rotate back to the starting position
 * Drive forward a set distance based on the vumark read.
 * Rotate 90 degrees toward the CryptoLock
 * Drive Forward, placing the glyph in the cryptolock
 * open the clamp
 * back up 4"
 * rotate 180 degrees (facing the glyph pit
 *
 *
 */

@Autonomous(name="Red 1 - Drive Test", group="Auto")
//Disabled
public class autoRedOneDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot robot   = new HardwareJoeBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 540 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.1;
    static final int     CENTER_DEGREES           = -48;
    static final double     CENTER_DISTANCE         =  22.0;
    static final int     LEFT_DEGREES             = -28;
    static final double     LEFT_DISTANCE           = 38.0;
    static final int     RIGHT_DEGREES            = -65;
    static final double     RIGHT_DISTANCE          = 11.0;
    static final double     BLUE_WON_DISTANCE       = 31.0;
    static final double     RED_WON_DISTANCE        = 41.0;

    int iVuMark = 3;
    int iJewelArm = 0;





    String heading  ="";
    double dublheading=0.0;
    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    long intheading = 0;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVzCl0v/////AAAAGcfsmNB0+Ecxi9nnFUli4RtGGZORFTsrkrZTsSaEZcnHNkxhb5NbskfqT531gL1cmgLFZ5xxeICDdBlPxxEbD4JcUvUuIdXxpVesR7/EAFZ+DTSJT3YQb0sKm2SlOlfiMf7ZdCEUaXuymCZPB4JeoYdogDUOdsOrd0BTDV2Z+CtO3eSsHWfcY6bDLh8VJKSbeFdk533EzcA26uhfhwBxYlzbOsjPSVCB66P6GbIP9/UjI3lbTNi+tpCpnOZa2gwPjoTSeEjo9ZKtkPe3a/DpLq3OMnVwVnUmsDvoW++UbtOmg9WNFC/YkN7DCtMt91uPaJPL5vOERkA+uXliC1i44IT4EyfoN1ccLaJiXMFH63DE";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addLine("Initialization Complete.");
        telemetry.update();

        waitForStart();

        relicTrackables.activate();


        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading: %7d", robot.angles);
        telemetry.update();

        // Close the clamp on the pre-loaded glyph
        robot.closeClamp();
        sleep(1000);


//        // Raise the clamp to a safe driving height
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(robot.LIFT_GLYPH_ONE_POS);
        robot.liftMotor.setPower(.5);
        while (robot.liftMotor.isBusy()) {
            idle();
        }
        robot.liftMotor.setPower(0);

        // Read the VuMark and store the Key Column
        // Need to add the VuMark Code here...

//        encoderDrive(DRIVE_SPEED, 2, 2, 10);
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//            //Using VuMark to determine which coluem is the key coluem
//
//            if (vuMark == RelicRecoveryVuMark.LEFT) {
//                telemetry.addLine("Left VuMark Discovered");
//                telemetry.update();
//                iVuMark = 1;
//            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
//                telemetry.addLine("Center VuMark Discovered");
//                telemetry.update();
//                iVuMark = 2;
//            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                telemetry.addLine("Right VuMark Discovered");
//                telemetry.update();
//                iVuMark = 3;
//            }
//        }
//
//            sleep(5000);
//        encoderDrive(DRIVE_SPEED, -2, -2, 10);
//            sleep(1000);



        // Drop the jewel arm
        robot.lowerJewelArm();
        sleep(1000);

//        // Read the color of the jewel in front of the jewel arm
//        // Raise the jewel arm
        telemetry.addData("Blue: ", robot.jewelSensor.blue());
        telemetry.addData("Red: ", robot.jewelSensor.red());
        telemetry.update();
//
//
        telemetry.addData("Blue: ", robot.jewelSensor.blue());
        telemetry.addData("Red: ", robot.jewelSensor.red());
//
//        // Based on the color of the jewel, rotate the bot either CW or CCW to knock off the right jewel
        if (robot.jewelSensor.red() > robot.jewelSensor.blue()) {
            //The sensor sees more Red than Blue, so the red jewel is "in front". Since this is
            //a "Red" opMode, we want to knock the blue jewel off the table.
            telemetry.addLine("Red Wins");
            //1 means red won and were following the red path
            iJewelArm = 1;
            encoderDrive(.3,-10,-10,5);
            robot.raiseJewelArm();
            encoderDrive(.3,10,10,5);
        }

        else {
            telemetry.addLine("Blue Wins");
            // 2 means blue won and were following the blue path
            iJewelArm = 2;
            encoderDrive(.3,10,10,5);
            robot.raiseJewelArm();

        }


        telemetry.update();

        Color.RGBToHSV((int) (robot.jewelSensor.red() * SCALE_FACTOR),
                (int) (robot.jewelSensor.green() * SCALE_FACTOR),
                (int) (robot.jewelSensor.blue() * SCALE_FACTOR),
                hsvValues);

        // Drive off the balancing stone red
        if (iJewelArm == 1) {

            encoderDrive(DRIVE_SPEED, RED_WON_DISTANCE, RED_WON_DISTANCE, 30);
        }

        // Drive off the balancing stone blue
        if (iJewelArm == 2) {

            encoderDrive(DRIVE_SPEED, BLUE_WON_DISTANCE, BLUE_WON_DISTANCE, 30);
        }

//        headingturn('r', LEFT_DEGREES);
//        stopmotors();
//        encoderDrive(DRIVE_SPEED, LEFT_DISTANCE, LEFT_DISTANCE, 30);
        // Turn based on vuMark left + right jewel
        if (iVuMark == 1 && iJewelArm == 1) {
            headingturn('r', -28);
            stopmotors();
            encoderDrive(DRIVE_SPEED, LEFT_DISTANCE, LEFT_DISTANCE, 30);

        }

        // Turn based on vuMark left + blue jewel
        if (iVuMark == 1 && iJewelArm == 2) {
            headingturn('r', -28);
            stopmotors();
            encoderDrive(DRIVE_SPEED, LEFT_DISTANCE, LEFT_DISTANCE, 30);
        }


        //---------------------------------------------------------------------------------//

        // Turn based on vuMark center + right jewel
        if (iVuMark == 2 && iJewelArm == 1) {
            headingturn('r', -42);
            stopmotors();
            encoderDrive(DRIVE_SPEED, CENTER_DISTANCE, CENTER_DISTANCE, 30);

        }

        // Turn based on vuMark center + blue jewel
        if (iVuMark == 2 && iJewelArm == 2) {
            headingturn('r', -42);
            stopmotors();
            encoderDrive(DRIVE_SPEED, CENTER_DISTANCE, CENTER_DISTANCE, 30);
        }

        //---------------------------------------------------------------------------------//

        //---------------------------------------------------------------------------------//

        // Turn based on vuMark Right + right jewel
        if (iVuMark == 3 && iJewelArm == 1) {
            headingturn('r', -65);
            stopmotors();
            encoderDrive(DRIVE_SPEED, RIGHT_DISTANCE, RIGHT_DISTANCE, 30);

        }

        // Turn based on vuMark right + blue jewel
        if (iVuMark == 3 && iJewelArm == 2) {
            headingturn('r', -65);
            stopmotors();
            encoderDrive(DRIVE_SPEED, RIGHT_DISTANCE, RIGHT_DISTANCE, 30);
        }

        //---------------------------------------------------------------------------------//

//        robot.liftMotor.setTargetPosition(robot.LIFT_STARTING_POS);
//        robot.liftMotor.setPower(-.5);
//        while (robot.liftMotor.isBusy()) {
//            idle();
//        }
//        robot.liftMotor.setPower(0);

//        headingturn(180);

//        robot.openClamp();
//        robot.lowerClamp();

        // Lower the clamp to 0
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(robot.LIFT_STARTING_POS);
        robot.liftMotor.setPower(.5);
        while (robot.liftMotor.isBusy()) {
            idle();
        }

        robot.openClamp();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, -9.0, -9.0, 30);

        sleep(10000);





    }




    public void headingturn (char leftorright,int targetheading)

    {
        double _dblheading=0.0;
        long _intheading=0;
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
                robot.motor1.setPower(-.1);
                robot.motor2.setPower(.1);
                robot.motor3.setPower(-.1);
                robot.motor4.setPower(.1);
            } else {
                robot.motor1.setPower(.1);
                robot.motor2.setPower(-.1);
                robot.motor3.setPower(.1);
                robot.motor4.setPower(-.1);
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
