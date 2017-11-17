package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

*/

/*
Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
code and understand this code for the possibility that a question may be asked related to TeleOp and
you should be able to explain in good detail everything in this code.
11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)

*/
@TeleOp(name="JoeBot TeleOp", group="TeleOp")

public class teleOp2017JoeBot extends LinearOpMode {

    HardwareJoeBot robot = new HardwareJoeBot();

    @Override
    public void runOpMode() throws InterruptedException {

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
        double leftServoPos = 1;
        double rightServoPos = 0.3;
        boolean bCurrStateA;
        boolean bPrevStateA = false;
        boolean bCurrStateB;
        boolean bPrevStateB = false;
        boolean bCurrStateX;
        boolean bPrevStateX = false;
        boolean bCurrStateY;
        boolean bPrevStateY = false;
        boolean bCurrStateLB;
        boolean bPrevStateLB = false;
        boolean bCurrStateRB;
        boolean bPrevStateRB = false;
        boolean bAutomatedLiftMotion = false;
        int iLiftTargetPos = 1;
        int iRightBumperTarget = 1;
        double liftPower = .6;

        robot.jewelSensor.enableLed(false);

        waitForStart();

        // In Teleop, we want to start with open clamp
        robot.openClamp();


        //start of loop
        while (opModeIsActive()) {


            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;

            // Add a tuning constant "K" to tune rotate axis sensitivity
            k = .6;
            clockwise = clockwise * k; //Make sure the "= Clockwise" is "= -clockwise"


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

            if (max > 1) {
                power1 /= max;
                power2 /= max;
                power3 /= max;
                power4 /= max;
            }

            robot.motor1.setPower(power1);
            robot.motor2.setPower(power2);
            robot.motor3.setPower(power3);
            robot.motor4.setPower(power4);
            //------------------------------------------
            //-------------------------------------------





            // Open/Close Clamps based on "B" Button Press
            // -------------------------------------------

            bCurrStateB = gamepad2.b;

            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {

                if (robot.bClampOpen) {
                    //Clamp is open. Close it.
                    robot.closeClamp();
                } else {
                    //Clamp must be closed. Open it.
                    robot.openClamp();
                }

            }

            bPrevStateB = bCurrStateB;


            // Rotate Clamps based on "Y" Button Press
            // -------------------------------------------

            bCurrStateY = gamepad2.y;

            if ((bCurrStateY == true) && (bCurrStateY != bPrevStateY)) {

                if (robot.bClampDown) {
                    //Clamp is down. Raise it.
                    robot.raiseClamp();
                } else {
                    //Clamp is up. Lower it.
                    robot.lowerClamp();
                }

            }

            bPrevStateY = bCurrStateY;


            // Check on lift state.. If automated lift motion is active, check to see if lift is
            // near its destination. If lift motion is at destination, turn off automated lift
            // motion.

            if (bAutomatedLiftMotion) {
                // The lift is in Auto mode. Check to see if we're near our target
                if (Math.abs(iLiftTargetPos - robot.liftMotor.getCurrentPosition()) < 50 ) {
                    // We're close enough to the target to shut down auto mode.
                    robot.liftMotor.setPower(0);
                    bAutomatedLiftMotion = false;

                    // Set Motors to run without encoder
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }
            }


            // Manually Lift
            // Raise the lift manually via "D-PAD" (NOT Toggle)
            // make a if statement
            if( gamepad2.dpad_up && (robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX_POSITION)) {
                // Check to see if the lift is already in auto mode. If it is, disable it.
                if (bAutomatedLiftMotion) {
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.liftMotor.setPower(0);
                    bAutomatedLiftMotion = false;

                    // reset iRightBumperTarget
                    iRightBumperTarget = 1;
                }
                robot.liftMotor.setPower(liftPower);
            } else if (gamepad2.dpad_down && (robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN_POSITION)) {
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


            // Left Bumper Press moves lift to "base" position

           bCurrStateLB = gamepad2.left_bumper;

            if ((bCurrStateLB == true) && (bCurrStateLB != bPrevStateLB)) {

                // Left Bumper has been pressed. We should set the lift into Auto Mode with the
                // Correct target position.

                bAutomatedLiftMotion = true;
                iLiftTargetPos = robot.LIFT_STARTING_POS;

                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setTargetPosition(iLiftTargetPos);
                robot.liftMotor.setPower(0.6);


            }

            bPrevStateLB = bCurrStateLB;

            // Right Bumper toggles between Position 1 and Position 2. First Press should be
            // Position 1

            bCurrStateRB = gamepad2.right_bumper;

            if ((bCurrStateRB == true) && (bCurrStateRB != bPrevStateRB)) {

                // Check to see if this is the first or second button press
                if (iRightBumperTarget == 1) {
                    // This is the first button press, or it has rolled over...
                    // Set lift into Auto Mode and head for position 1
                    bAutomatedLiftMotion = true;
                    iLiftTargetPos = robot.LIFT_GLYPH_ONE_POS;

                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setTargetPosition(iLiftTargetPos);
                    robot.liftMotor.setPower(0.6);

                } else if (iRightBumperTarget == 2) {
                    // Set lift into Auto Mode and head for position 2
                    bAutomatedLiftMotion = true;
                    iLiftTargetPos = robot.LIFT_GLYPH_TWO_POS;

                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setTargetPosition(iLiftTargetPos);
                    robot.liftMotor.setPower(0.6);
                } else {
                    // We've received an invalid command
                    // Don't do anything right now.. May want to add cleanup code later.
                }

                // Set new Lift Target for next button press.
                iRightBumperTarget += 1;
                if (iRightBumperTarget>2) { iRightBumperTarget = 1; }

            }

            bPrevStateRB = bCurrStateRB;

            // Toggle Search Mode - raise Lift to search position; Open clamp; rotate clamp

            bCurrStateA = gamepad2.a;

            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                // Would like to Raise Clamp before moving lift because it moves better vertically
                // but can't figure out in short term how to know when lift is finished moving
                // and search mode is active to drop and open clamp. For now, will move clamp
                // while lift is in motion.

                bAutomatedLiftMotion = true;
                iLiftTargetPos = robot.LIFT_SEARCHING_POS;

                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setTargetPosition(iLiftTargetPos);
                robot.liftMotor.setPower(0.6);

                // Lower Clamp and Open
                if (!robot.bClampDown) { robot.lowerClamp(); }
                if (!robot.bClampOpen) { robot.openClamp();}

            }

            bPrevStateA = bCurrStateA;


            // Pick up Glyphs and prepare to drive

            bCurrStateX = gamepad2.x;

            if ((bCurrStateX == true) && (bCurrStateX != bPrevStateX)) {

                // This code should close the clamp on the glyphs, rotate the glyph clamp to the
                // "up" position, then lower the lift to the "driving" (or "base") position

                robot.closeClamp();
                sleep(1000);
                robot.raiseClamp();
                sleep(500);

                bAutomatedLiftMotion = true;
                iLiftTargetPos = robot.LIFT_STARTING_POS;

                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setTargetPosition(iLiftTargetPos);
                robot.liftMotor.setPower(0.6);


            }

            bPrevStateX = bCurrStateX;







            // Update Telemetry
            telemetry.addData("Clamp Open?: ", robot.bClampOpen);
            telemetry.addData("Clamp Down?: ", robot.bClampDown);
            telemetry.addData("Lift Position: ", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Lift Target: ", iLiftTargetPos);
            telemetry.addData("RB Target: ", iRightBumperTarget);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();






        }
    }
}