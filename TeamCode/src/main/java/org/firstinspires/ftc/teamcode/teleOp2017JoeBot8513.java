package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        double rightNumber = 0;
        double liftPower = .6;


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





            // Open/Close Clamps based on "B" Button Press
            // -------------------------------------------

            bCurrStateA = gamepad1.a;

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
            if( gamepad1.dpad_up && (robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX_POSITION)) {
                robot.liftMotor.setPower(liftPower);
            } else if (gamepad1.dpad_down && (robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN_POSITION)) {
                robot.liftMotor.setPower(-liftPower);
            } else {
                robot.liftMotor.setPower(0);
            }



            // Update Telemetry
            telemetry.addData("Clamp Open?: ", robot.bClampOpen);
            telemetry.addData("Lift Position: ", "%5.2f", robot.liftMotor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();



        }
    }
}