package org.firstinspires.ftc.teamcode.TestingGround;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareJoeBotMechTest;


 //* ///This OpMode illustrates the basics of using the Vuforia engine to determine
 //* the identity of Vuforia VuMarks encountered on the field. The code is structured as
/// * //a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 ///* //duplicate the core Vuforia documentation found there, but rather instead focus on the
 ////*// differences between the use of Vuforia for navigation vs VuMark identification.
 //*//
 //*// @see ConceptVuforiaNavigation
 //*// @see VuforiaLocalizer
 //*// @see VuforiaTrackableDefaultListener

@Autonomous(name="Finishing_Levi_Charlie_Controller_Mapping", group ="Concept")

public class Finishing_Levi_Charlie_Controller_Mapping extends LinearOpMode  {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    HardwareJoeBotMechTest robot    = new HardwareJoeBotMechTest();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    @Override public void runOpMode() {
        robot.init(hardwareMap);

        OpenGLMatrix lastLocation = null;
        HardwareJoeBotMechTest robot    = new HardwareJoeBotMechTest();
        //Joysticks-> Left Joystick=Foward/backwards drive,RightJoystick=Left/Right drive Also Diagnaol
        //Ignore the +1 in -> \/  its not the offical #
        robot.motor1.setPower(+1);
        robot.motor2.setPower(+1);
        robot.motor3.setPower(+1);
        robot.motor4.setPower(+1);
        //Buttons-> Not Completed
        gamepad2.x.();
        gamepad2.y.();
        gamepad2.b.();
        //DPad->Not Complete Look At the notes right about here \/
        gamepad1.dpad_up(robot.motorLift.setpower(+1));//<-Need to make a declaration for the motor connected to the lift
        gamepad1.dpad_down(robot.motorlift.setpower(-1));


    }
}
