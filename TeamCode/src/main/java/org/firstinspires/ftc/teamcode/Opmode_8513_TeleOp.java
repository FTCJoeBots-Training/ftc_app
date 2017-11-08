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


/**
 * This OpMode uses the JoeBot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareJoeBot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a Tank Drive style Teleop for the 2015 JoeBot
 */

@TeleOp(name="8513_Drive", group="TeleOp")

public class Opmode_8513_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot robot = new HardwareJoeBot();     // Use a JoeBot's hardware
    boolean intakeEnabledX;
    boolean intakeEnabledY;
    boolean intakeEnabledA;
    boolean bCurrStateY;
    boolean bPrevStateY;
    boolean bCurrStateX;
    boolean bPrevStateX;
    boolean bCurrStateA;
    Boolean bPrevStateA;
    boolean bCurrStateB;
    boolean bPrevStateB;
    boolean bSpinnersOn;
    boolean bIntakeOn;
    double liftLimit = .8;
    double liftPower = 0;
    boolean RBPon;
    boolean LBPon;
    boolean bCurrStateDpad;
    boolean bPrevStateDpad;
    Boolean bCurrStateRbump;
    boolean bPrevStateRbump;
    boolean bCurrStateLbump;
    boolean bPrevStateLbump;
    boolean Intake_eleon;
    boolean LifTRout;
    boolean bCurrStateDpadD;
    boolean bPrevStateDpadD;
    double intakePower = 0.65;
    double elevatorPower = -0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;


        //double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Ethan");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in Tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.

            left = -gamepad1.left_stick_y / -1.4;
            right = -gamepad1.right_stick_y / -1.5;
            robot.rightmotor.setPower(left);
            robot.leftmotor.setPower(right);

            //-----------------------------------------------//


            // Toggle Intake  On/Off

            bCurrStateB = gamepad2.b;

            // check for button state transitions.
            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {

                bIntakeOn = !bIntakeOn;

            }
            bPrevStateB = bCurrStateB;

            /**
            if (bIntakeOn == true) {
                robot.Intake.setPower(intakePower);
            } else {
                robot.Intake.setPower(0);

            }

            // Toggle Intake Direction

            bCurrStateX = gamepad2.x;
            if ((bCurrStateX == true) && (bCurrStateX!=bPrevStateX)) {
                intakePower = intakePower * -1;
            }
            bPrevStateX = bCurrStateX;

            **/


            // Toggle Intake Elevator On/Off

            bCurrStateY = gamepad2.y;

            // check for button state transitions.

            if ((bCurrStateY == true) && (bCurrStateY != bPrevStateY)) {

                Intake_eleon = !Intake_eleon;

            }
            bPrevStateY = bCurrStateY;

            /**if (Intake_eleon == true) {
                robot.Intake_Elevator.setPower(elevatorPower);
            } else {
                robot.Intake_Elevator.setPower(0);
            }

            // Toggle Elevator Direction
            bCurrStateA = gamepad2.a;

            // check for button state transitions.
            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                elevatorPower = elevatorPower * -1;

            }
            bPrevStateA = bCurrStateA;


            // Toggle Spinners (shooters)
            // check the status of the a button on either gamepad.
            bCurrStateDpad = gamepad2.dpad_up;

            // check for button state transitions.
            if ((bCurrStateDpad == true) && (bCurrStateDpad != bPrevStateDpad)) {
                bSpinnersOn = !bSpinnersOn;
            }
            bPrevStateDpad = bCurrStateDpad;

            if (bSpinnersOn == true) {

                robot.RightSpinner.setPower(.7);
                robot.LeftSpinner.setPower(-.7);

            } else {

                robot.RightSpinner.setPower(0);
                robot.LeftSpinner.setPower(0);

            }


            // Map Lift Power to Right_Stick_Y
            liftPower = gamepad2.right_stick_y * liftLimit;
            robot.motor_lift.setPower(liftPower);
            //-----------------------------------------------//


            // check the status of the X button on  gamepad2.
            bCurrStateRbump = gamepad2.right_bumper;

            // check for button state transitions.
            if ((bCurrStateRbump == true) && (bCurrStateRbump != bPrevStateRbump)) {

                RBPon = !RBPon;

            }
            bPrevStateRbump = bCurrStateRbump;

            if (RBPon == true) {
                robot.RBP.setPosition(.7);
            } else {
                robot.RBP.setPosition(0);
            }


            //-----------------------------------------------//


            // check the status of the left bumper button on  gamepad2.
            bCurrStateLbump = gamepad2.left_bumper;

            // check for button state transitions.
            if ((bCurrStateLbump == true) && (bCurrStateLbump != bPrevStateLbump)) {
                 LBPon = !LBPon;
            }
            bPrevStateLbump = bCurrStateLbump;

            if (LBPon == true) {
                robot.LBP.setPosition(.7);
            } else {
                robot.LBP.setPosition(0);
            }



//--------------------------------------------------------------------------------------//

            // check the status of the Dpad Down button on  gamepad2.
            bCurrStateDpadD = gamepad2.dpad_down;

            // check for button state transitions.
            if ((bCurrStateDpadD == true) && (bCurrStateDpadD != bPrevStateDpadD)) {
                LifTRout = !LifTRout;
            }
            bCurrStateDpadD = bPrevStateDpadD;

            if (LifTRout == true) {
                robot.Lift_R.setPosition(.7);
            } else {
                robot.Lift_R.setPosition(0);
            }


            **/


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            }
        }

    }
