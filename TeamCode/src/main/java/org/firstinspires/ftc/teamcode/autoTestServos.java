package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Continous Servo Test", group = "Concept")
@Disabled
public class autoTestServos extends LinearOpMode {

    static final double INCREMENT       = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS        =   50;     // period of each cycle
    static final double LEFT_IN_POS     =  0.8;     // Left Side "in" position
    static final double LEFT_OUT_POS    =  0.5;     // Left Side "out" position
    static final double RIGHT_IN_POS    =  0.0;     // Right Side "in" position
    static final double RIGHT_OUT_POS   =  0.3;     // Right Side "out" position

    // Define class members
    HardwareJoeBot robot = new HardwareJoeBot(); //Use HardwareJoeBot

    double  lPosition = 0.8; // Start at halfway position
    double  rPosition = 0.0;
    boolean rampUp = true;


    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the Robot
        robot.init(hardwareMap);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                lPosition -= INCREMENT;
                rPosition += INCREMENT;
                if (lPosition <= LEFT_OUT_POS ) {
                    lPosition = LEFT_OUT_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                lPosition += INCREMENT ;
                rPosition -= INCREMENT;
                if (lPosition >= LEFT_IN_POS ) {
                    lPosition = LEFT_IN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Left Servo Position", "%5.2f", lPosition);
            telemetry.addData("Right Servo Position", "%5.2f", rPosition);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            //robot.srv_left.setPosition(lPosition);
            //robot.srv_right.setPosition(rPosition);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
