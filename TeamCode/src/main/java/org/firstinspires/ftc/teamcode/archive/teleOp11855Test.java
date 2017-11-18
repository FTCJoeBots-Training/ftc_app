package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.archive.HardwareJoeBot11855;

@TeleOp (name ="11855-Test", group = "Testing")
@Disabled
//IDK what to call this so IDK will do... Don't ever question me.
public class teleOp11855Test extends LinearOpMode {

    HardwareJoeBot11855 robot = new HardwareJoeBot11855();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        double left;
        double right;
        boolean bCurrStateA = false;
        boolean bPrevStateA = false;
        boolean bCurrStateX = false;
        boolean bPrevStateX = false;
        boolean bCurrStateY = false;
        boolean bPrevStateY = false;
        boolean bCurrStateB = false;
        boolean bPrevStateB = false;
        boolean bElevatorOn = false;
        boolean bElevatorUp = false;
        double clampPosition = 0;
        double liftPosition = 0;
        double elevatorPower = 0;

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

            // Toggle Motor Speed

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

            // Toggle Elevator on/off

            bCurrStateY = gamepad1.y;

            // Toggle Motor Speed.

            if ((bCurrStateY == true) && (bCurrStateY != bPrevStateY)) {

                bElevatorOn = !bElevatorOn;

            }
            bPrevStateY = bCurrStateY;

            // Toggle Elevator Up/Down

            bCurrStateX = gamepad1.x;

            // Toggle Motor Speed.

            if ((bCurrStateX == true) && (bCurrStateX != bPrevStateX)) {

                bElevatorUp = !bElevatorUp;
            }
            bPrevStateX = bCurrStateX;


            //Toggle Elevator Power
            bCurrStateB = gamepad1.b;

            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {
                elevatorPower = elevatorPower + .1;
            }


            if (gamepad1.dpad_up) {
                liftPosition = liftPosition + .01;
            }

            if (gamepad1.dpad_down){
                liftPosition = liftPosition - .01;
            }

            if (gamepad1.dpad_right) {
                clampPosition = clampPosition + .01;
            }

            if (gamepad1.dpad_left) {
                clampPosition = clampPosition - .01;
            }

            if (liftPosition > 1) {
                liftPosition = 1;
            } else if (liftPosition < 0) {
                liftPosition = 0;
            }

            if (clampPosition > 1) {
                clampPosition = 1;
            } else if (clampPosition < 0) {
                clampPosition = 0;
            }

            telemetry.addData("Elevator On?:", bElevatorOn);
            telemetry.addData("Elevator Up?:", bElevatorUp);
            telemetry.addData("Clamp Position: ", clampPosition);
            telemetry.addData("Lift Position: ", liftPosition);

            telemetry.update();

            robot.clampservo.setPosition(clampPosition);
            robot.liftservo.setPosition(liftPosition);

            // Run wheels in Tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;

            robot.leftmotor.setPower(left * speedConstant);
            robot.rightmotor.setPower(right * speedConstant);

            if (bElevatorOn) {
                if (bElevatorUp) {
                    robot.elevatormotor.setPower(elevatorPower);
                } else {
                    robot.elevatormotor.setPower(-elevatorPower);
                }
            } else {
                robot.elevatormotor.setPower(0);
            }


        }


    }
}