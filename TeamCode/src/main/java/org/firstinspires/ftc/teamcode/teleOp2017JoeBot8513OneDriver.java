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


*/
@TeleOp(name="8513 TeleOp", group="TeleOp")

public class teleOp2017JoeBot8513 extends LinearOpMode {

    HardwareJoeBot8513 robot = new HardwareJoeBot8513();

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
        int iRightBumperTarget = 1;
        int iLiftTargetPos = 0;
        double rightNumber = 0;
        double liftPower = .6;


        // Make sure Clamps are open in TeleOp
        robot.openClamp();

        waitForStart();


        //start of loop
        while (opModeIsActive()) {


            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;

            // Add a tuning constant "K" to tune rotate axis sensitivity
            k = .6;
            clockwise = clockwise * k;


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





            // Open/Close Clamps based on "A" Button Press
            // -------------------------------------------

            bCurrStateA = gamepad2.a;

            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                if (robot.bClampOpen) {
                    //Clamp is open. Close it.
                    robot.closeClamp();
                } else {
                    //Clamp must be closed. Open it.
                    robot.openClamp();
                }

            }

            bPrevStateA = bCurrStateA;



            // Manually Lift
            // Raise the lift manually via "D-PAD" (NOT Toggle)
            // make a if statement
            if( gamepad2.dpad_up && (robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX_POSITION)) {
                // Check to see if the lift is already in auto mode. If it is, disable it.
                if (bAutomatedLiftMotion) {
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.liftMotor.setPower(.1);
                    bAutomatedLiftMotion = false;

                    // reset iRightBumperTarget
                    iRightBumperTarget = 1;

                }
                robot.liftMotor.setPower(liftPower);
            } else if (gamepad2.dpad_down && (robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN_POSITION)) {
                // Check to see if the lift is already in auto mode. If it is, disable it.
                if (bAutomatedLiftMotion) {
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.liftMotor.setPower(.1);
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

            // For Testing Only -- Manually move jewel Arm
            if (gamepad2.dpad_left && (robot.jewelServo.getPosition() > 0)) {
                robot.jewelServo.setPosition(robot.jewelServo.getPosition() - .05);
                sleep(200);
            } else if (gamepad2.dpad_right && (robot.jewelServo.getPosition() < 1)) {
                robot.jewelServo.setPosition(robot.jewelServo.getPosition() + .05);
                sleep(200);
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




            // Update Telemetry
            //Third line down Finds Lift's current position.
            telemetry.addData("Clamp Open?: ", robot.bClampOpen);
            telemetry.addData("Jewel Arm Pos: ", robot.jewelServo.getPosition());
            telemetry.addData("Lift Position: ", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Lift Target: ", iLiftTargetPos);
            telemetry.addData("Right Bumper: ", iRightBumperTarget);
            telemetry.addData("Automated Lift?", bAutomatedLiftMotion);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();



        }
    }
}