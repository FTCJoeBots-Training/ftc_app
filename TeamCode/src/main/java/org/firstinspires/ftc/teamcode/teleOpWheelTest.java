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
@TeleOp(name="Wheel Test", group="TeleOp")

public class teleOpWheelTest extends LinearOpMode {

    HardwareJoeBot8513 robot = new HardwareJoeBot8513();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);


        double power1;
        double power2;

        double power3;
        double power4;

        waitForStart();

        //start of loop
        while (opModeIsActive()) {


            while (gamepad1.a) {
                robot.motor1.setPower(.5);
            }
            while (gamepad1.b) {
                robot.motor2.setPower(.5);
            }
            //while (gamepad1.x) {
              //  robot.motor3.setPower(.5);
            //}
            while (gamepad1.y) {
                robot.motor4.setPower(.5);
            }


            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);
            idle();


            if (gamepad1.x) {

            }




        }
    }
}