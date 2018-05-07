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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;


@TeleOp(name="Twins Clamp/Servo Test", group="TeleOp")
//@Disabled
public class teleOpClampServoTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot robot = new HardwareJoeBot();

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        double forward;
        double clockwise;
        double right;
        double k;
        double power1;
        double power2;
        double power3;
        double power4;
        double max;
        double clampServoPos = 0;
        double rotateServoPos = 0;
        boolean bCurrStateA;
        boolean bPrevStateA = false;
        boolean bCurrStateX;
        boolean bPrevStateX = false;
        boolean bAutomatedLiftMotion = false;
        int iRightBumperTarget = 0;
        double liftPower = 0.5;



        clampServoPos = robot.clampServo.getPosition();
        rotateServoPos = robot.clampRotate.getPosition();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;

            // Add a tuning constant "K" to tune rotate axis sensitivity
            k = .6;
            clockwise = clockwise*k;


            // Calculate motor power
            power1 = forward + clockwise + right;
            power2 = forward - clockwise - right;
            power3 = forward + clockwise - right;
            power4 = forward - clockwise + right;

            // Normalize Wheel speeds so that no speed exceeds 1.0
            max = Math.abs(power1);
            if (Math.abs(power2) > max) {
                max = Math.abs(power2);
            }
            if (Math.abs(power3) > max) {
                max = Math.abs(power3);
            }
            if (Math.abs(power4) > max) {
                max = Math.abs(power4);
            }

            if (max>1) {
                power1 /= max;
                power2 /= max;
                power3 /= max;
                power4 /= max;
            }

            robot.motor1.setPower(power1);
            robot.motor2.setPower(power2);
            robot.motor3.setPower(power3);
            robot.motor4.setPower(power4);

            // Code to Test start/stop position of servos... Just increase/decrease positions and read telemetry
            // dpad left/right manages clamp; dpad up/down is lift; right/left bumpers are rotate.

            // Code to test ClampServo
            if(opModeIsActive() && gamepad1.dpad_left && robot.clampServo.getPosition()>0) {
                clampServoPos -= 0.01;
            }
            if (opModeIsActive() && gamepad1.dpad_right && robot.clampServo.getPosition() < 1.0) {
                clampServoPos += 0.01;
            }

           //Code to test clampRotate

            if(opModeIsActive() && gamepad1.left_bumper && robot.clampRotate.getPosition()>0) {
                rotateServoPos -= 0.01;
            }
            if (opModeIsActive() && gamepad1.right_bumper && robot.clampRotate.getPosition() < 1.0) {
                rotateServoPos += 0.01;
            }



            // Manually Lift
            // Raise the lift manually via "D-PAD" (NOT Toggle)
            // make a if statement
            if( gamepad1.dpad_up && (robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX_POSITION)) {
                // Check to see if the lift is already in auto mode. If it is, disable it.
                if (bAutomatedLiftMotion) {
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.liftMotor.setPower(0);
                    bAutomatedLiftMotion = false;

                    // reset iRightBumperTarget
                    iRightBumperTarget = 1;
                }
                robot.liftMotor.setPower(liftPower);
            } else if (gamepad1.dpad_down && (robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN_POSITION)) {
                // Check to see if the lift is already in auto mode. If it is, disable it.
                if (bAutomatedLiftMotion) {
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.liftMotor.setPower(0);
                    bAutomatedLiftMotion = false;

                    // reset iRightBumperTarget
                    iRightBumperTarget = 1;
                }
                robot.liftMotor.setPower(-liftPower);
            } else {
                // Check to see if the lift is already in an automated motion. If it is not,
                // set power to 0
                if (!bAutomatedLiftMotion) {
                    robot.liftMotor.setPower(0);
                }
            }


            robot.clampRotate.setPosition(rotateServoPos);
            robot.clampServo.setPosition(clampServoPos);

            // Display the current value
            telemetry.addData("Clamp Servo Pos: ", robot.clampServo.getPosition());
            telemetry.addData("Rotate Servo Pos: ", robot.clampRotate.getPosition());
            telemetry.addData("Lift Position: ", robot.liftMotor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);

        }
    }




    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
