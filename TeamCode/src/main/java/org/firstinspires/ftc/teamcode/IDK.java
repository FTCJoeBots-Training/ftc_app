package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp (name ="TankToAll", group = "TeleOp")
//@Disabled
//IDK what to call this so IDK will do... Don't ever question me.
public class IDK extends LinearOpMode {

    HardwareJoeBot robot = new HardwareJoeBot();


    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double rightTrigger;
        double leftTrigger;
        double rPosition;
        double lPosition;

        //double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in Tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.motor_driveleft.setPower(left);
            robot.motor_driveright.setPower(right);

            // Raise Arm with Triggers
            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            //If the right trigger is pressed, we're going to ignore left trigger
            if (rightTrigger > 0) {
                // robot.motor_arm.setPower(rightTrigger);
            } else if (leftTrigger > 0) {
                // robot.motor_arm.setPower(-leftTrigger);
            } else {
                //robot.motor_arm.setPower(0);
            }

3
        }


    }
}