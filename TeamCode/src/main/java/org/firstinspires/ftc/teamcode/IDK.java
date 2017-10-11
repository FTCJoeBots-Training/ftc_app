package org.firstinspires.ftc.teamcode;

/**
 * Created by 535540 on 10/11/17.
 */
//IDK what to call this so IDK will do... Don't ever question me.
public class IDK {
}

{
    public void runOpMode() throws InterruptedException{
        double left;
        double right;


        //double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say","Hello Ethan");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){

        // Run wheels in Tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.

        left=-gamepad1.left_stick_y/-1.4;
        right=-gamepad1.right_stick_y/-1.5;
        robot.motor_driveleft.setPower(left);
        robot.motor_driveright.setPower(right);
        }
        }