package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

@TeleOp (name ="DriverTraining-Tank", group = "DriverTraining")
//@Disabled
//IDK what to call this so IDK will do... Don't ever question me.
public class teleOpTankDriveTraining extends LinearOpMode {

    HardwareJoeBotDriveTraining robot = new HardwareJoeBotDriveTraining();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        double left;
        double right;
        boolean bCurrStateA;
        boolean bPrevStateA;
        double speedConstant;

        waitForStart();

        //double max;
        speedConstant = 0.5;
        telemetry.addLine("Low Speed");
        bPrevStateA = false;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Toggle Intake Elevator On/Off

            bCurrStateA = gamepad1.a;

            // Toggle Motor Speed.

            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                if (speedConstant == .5) {
                    speedConstant = .75;
                    telemetry.addLine("Medium Speed");
                } else if (speedConstant == .75) {
                    speedConstant = 1;
                    telemetry.addLine("High Speed");
                } else if (speedConstant == 1) {
                    speedConstant = .5;
                    telemetry.addLine("Low Speed");
                }

            }
            bPrevStateA = bCurrStateA;

            telemetry.update();

            // Run wheels in Tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.leftmotor.setPower(left*speedConstant);
            robot.rightmotor.setPower(right*speedConstant);

        }


    }
}