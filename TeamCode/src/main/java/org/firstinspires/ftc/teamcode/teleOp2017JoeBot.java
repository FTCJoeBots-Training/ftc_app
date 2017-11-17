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
        boolean bLiftinMotion = false;
        int iLiftTargetPos = 1;
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
            clockwise = -gamepad1.right_stick_x;

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



            // Manually Lift
            // Raise the lift manually via "D-PAD" (NOT Toggle)
            // make a if statement
            if( gamepad2.dpad_up && (robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX_POSITION)) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(liftPower);
            } else if (gamepad2.dpad_down && (robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN_POSITION)) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-liftPower);
            } else {
                // Check to see if the lift is already in an automated motion. If it is not,
                // set power to 0
                if (!bLiftinMotion) {
                    robot.liftMotor.setPower(0);
                }
            }




           bCurrStateLB = gamepad2.left_bumper;

            if ((bCurrStateLB == true) && (bCurrStateLB != bPrevStateLB)) {

                if (Math.abs(robot.LIFT_STARTING_POS - robot.liftMotor.getCurrentPosition() ) < 50 ) {
                    bLiftinMotion = false;
                    robot.liftMotor.setPower(0);
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    bLiftinMotion = true;
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setTargetPosition(robot.LIFT_STARTING_POS);
                    robot.liftMotor.setPower(.5);
                }

            }

            bPrevStateLB = bCurrStateLB;

            // Toggle Search Mode - raise Lift to search position; Open clamp; rotate clamp

            bCurrStateA = gamepad2.a;

            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                if (Math.abs(robot.LIFT_SEARCHING_POS - robot.liftMotor.getCurrentPosition() ) < 50 ) {
                    bLiftinMotion = false;
                    robot.liftMotor.setPower(0);
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    bLiftinMotion = true;
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setTargetPosition(robot.LIFT_SEARCHING_POS);
                    robot.liftMotor.setPower(.5);
                }

                if (!robot.bClampDown) { robot.lowerClamp(); }
                if (!robot.bClampOpen) { robot.openClamp();}

            }

            bPrevStateA = bCurrStateA;








            // Update Telemetry
            telemetry.addData("Clamp Open?: ", robot.bClampOpen);
            telemetry.addData("Clamp Down?: ", robot.bClampDown);
            telemetry.addData("Lift Position: %5.2f", robot.liftMotor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();






        }
    }
}