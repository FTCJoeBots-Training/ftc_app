package org.firstinspires.ftc.teamcode.TestingGround;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

*/import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareJoeBot13702;
import org.firstinspires.ftc.teamcode.HardwareJoeBot8513;

import java.util.Locale;

/*
Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
code and understand this code for the possibility that a question may be asked related to TeleOp and
you should be able to explain in good detail everything in this code.


*/
@TeleOp(name="ManuallyLists", group="TeleOp")

public class ManuallyLists extends LinearOpMode {

    HardwareJoeBot13702 robot = new HardwareJoeBot13702();

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
        boolean bCurrStateX;
        boolean bPrevStateX = false;
        double rightNumber = 0;
        double liftPower = .25;
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





            // Code to Test start/stop position of servos... Just increase/decrease positions and read telemetry
            // -------------------------------------------
            //clamps, Manual Clamp Via "A" (Toggle)
            bCurrStateA = gamepad1.a;

            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                if (robot.bClampOpen) {
                    //Clamp is open. Close it.
                    robot.closeClamp();
                    robot.bClampOpen = false;
                    telemetry.addLine("Clamp is Closed");
                } else {
                    //Clamp must be closed. Open it.
                    robot.openClamp();
                    robot.bClampOpen = true;
                    telemetry.addLine("Clamp is Open");
                }

            }

            bPrevStateA = bCurrStateA;







            //------------------------------------------------------
            //Auto Lift Lifter Via "X" (Toggle)
            /*
            bCurrStateX = gamepad1.x;

            if ((bCurrStateX == true) && (bCurrStateX != bPrevStateX)) {

                // if Clamp is up, lower it, otherwise, raise it.

                if (robot.bLiftRaised) {
                    robot.liftMotor.setTargetPosition(robot.LIFT_MIN_POSITION);
                } else {
                    robot.liftMotor.setTargetPosition(2880);
                }
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.5);
                }
                robot.liftMotor.setPower(0);
                robot.bLiftRaised = !robot.bLiftRaised;

                robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
                */
           //------------------------------------------------







            //Manually Lift
            //Raise the lift manually via "D-PAD" (NOT Toggle)
            //make a if statement
            if( gamepad1.dpad_up && (robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX_POSITION)) {
                robot.liftMotor.setPower(liftPower);
            }
            else {
                robot.liftMotor.setPower(0);
            }
            telemetry.update();
            //lower the lift Manually
            if( gamepad1.dpad_down && (robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN_POSITION)) {
                robot.liftMotor.setPower(-liftPower);
            }
            else {
                robot.liftMotor.setPower(0);
            }
            telemetry.update();

            // Display the current value
            telemetry.addData("MotorPosition: ", robot.liftMotor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();


            //---------------



        /*
            //Rotate (Toggle)
            //Rotate Lift piviting down and up in verticle positioning
            bCurrStateX = gamepad1.x;

            if ((bCurrStateX == true) && (bCurrStateX != bPrevStateX)) {

                if (robot.bRotateDown) {
                    //Clamp is open. Close it.
                    robot.rotateUp();
                } else {
                    //Clamp must be closed. Open it.
                    robot.rotateDown();
                }

            }

            bPrevStateX = bCurrStateX;
        */
            bCurrStateX = gamepad1.x;

            if ((bCurrStateX == true) && (bCurrStateX != bPrevStateX)) {

                if (robot.bClampOpen) {
                robot.closeClamp();
                }
                else{
                    robot.openClamp();
                }

               telemetry.update();


            }

            bPrevStateX = bCurrStateX;


        }
    }
}