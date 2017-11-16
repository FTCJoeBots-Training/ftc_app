package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Auto", group = "TeleOp")
@Disabled
public class mecanumauto extends LinearOpMode {
    //Motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor rearRight;
    private DcMotor rearLeft;

    private enum HolonomicDirection {
        FORWARDS,
        BACKWARDS,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        ROTATE_CLOCKWISE,
        ROTATE_COUNTERCLOCKWISE,
        STRAFE_NORTH_EAST,
        STRAFE_NORTH_WEST,
        STRAFE_SOUTH_EAST,
        STRAFE_SOUTH_WEST
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Init everything
        runInitSequence();

        //Wait for the start button to be pressed.
        waitForStart();

        //Autonomous driving commands
        //YOUR CODE HERE!
    }

    private void runInitSequence() {
        //Define HW objects
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        rearLeft = hardwareMap.dcMotor.get("RL");
        rearRight = hardwareMap.dcMotor.get("RR");

        //Account for reversed motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);


        //Run without internal PID
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopDriveChain() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    private void Drive(HolonomicDirection dir, double pow) {
        if (dir == HolonomicDirection.FORWARDS) {
            frontLeft.setPower(pow);
            frontRight.setPower(pow);
            rearLeft.setPower(pow);
            rearRight.setPower(pow);
        } else if (dir == HolonomicDirection.BACKWARDS) {
            frontLeft.setPower(-pow);
            frontRight.setPower(-pow);
            rearLeft.setPower(-pow);
            rearRight.setPower(-pow);
        } else if (dir == HolonomicDirection.STRAFE_LEFT) {
            frontLeft.setPower(-pow);
            frontRight.setPower(pow);
            rearLeft.setPower(pow);
            rearRight.setPower(-pow);
        } else if (dir == HolonomicDirection.STRAFE_RIGHT) {
            frontLeft.setPower(pow);
            frontRight.setPower(-pow);
            rearLeft.setPower(-pow);
            rearRight.setPower(pow);
        } else if (dir == HolonomicDirection.ROTATE_CLOCKWISE) {
            frontLeft.setPower(pow);
            frontRight.setPower(-pow);
            rearLeft.setPower(pow);
            rearRight.setPower(-pow);
        } else if (dir == HolonomicDirection.ROTATE_COUNTERCLOCKWISE) {
            frontLeft.setPower(-pow);
            frontRight.setPower(pow);
            rearLeft.setPower(-pow);
            rearRight.setPower(pow);
        } else if (dir == HolonomicDirection.STRAFE_NORTH_WEST) {
            frontLeft.setPower(0);
            frontRight.setPower(pow);
            rearLeft.setPower(pow);
            rearRight.setPower(0);
        } else if (dir == HolonomicDirection.STRAFE_NORTH_EAST) {
            frontLeft.setPower(pow);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(pow);
        } else if (dir == HolonomicDirection.STRAFE_SOUTH_WEST) {
            frontLeft.setPower(-pow);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(-pow);
        } else if (dir == HolonomicDirection.STRAFE_SOUTH_EAST) {
            frontLeft.setPower(0);
            frontRight.setPower(-pow);
            rearLeft.setPower(-pow);
            rearRight.setPower(0);
        }
    }

    public void pause(int mill) throws InterruptedException {
        Thread.sleep(mill * 1000);
    }
}