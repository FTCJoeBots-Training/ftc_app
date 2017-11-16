package org.firstinspires.ftc.teamcode.TestingGround;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
/*import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
*/import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareJoeBot8513;
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
//@Disabled
public class Finishing_Levi_Charlie_Controller_Mapping extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    @Override
    public void runOpMode() throws InterruptedException {
    }

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
    boolean bClampOpen;
    boolean closeClamp;


    OpenGLMatrix lastLocation = null;

    HardwareJoeBot8513 robot= new HardwareJoeBot8513();

    //private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();

     public void Joysticks() {
         waitForStart();
         robot.init(hardwareMap, this);


         OpenGLMatrix lastLocation = null;


         //------------------------------
         //Joysticks for moving foward
         //Joysticks-> Left Joystick=Foward/backwards drive,RightJoystick=Left/Right drive Also Diagnaol
         //Ignore the +1 in -> \/  its not the offical #
         forward = -gamepad1.left_stick_y;
         right = gamepad1.left_stick_x;
         clockwise = gamepad1.right_stick_x;

         k = .6;
         clockwise = clockwise * k;


         // Calculate motor power (For Turning)
         power1 = forward + clockwise + right;
         power2 = forward - clockwise - right;
         power3 = forward + clockwise - right;
         power4 = forward - clockwise + right;
         //--------
     }
        public void buttonmaping() {


        //----------------------------
        //Buttons-> Not Completed

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

         // Run the lift to a pre-set position when the "x" button is pressed
         bCurrStateX = gamepad1.x;

         if ((bCurrStateX == true) && (bCurrStateX != bPrevStateX)) {

             // if Clamp is up, lower it, otherwise, raise it.

             if (robot.bLiftRaised) {
                 robot.liftMotor.setTargetPosition(robot.LIFT_MIN_POSITION);
             } else {
                 robot.liftMotor.setTargetPosition(2880); //<- That 2880 is not the final #
             }
             robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.liftMotor.setPower(0.5);

             while (opModeIsActive() && robot.liftMotor.isBusy()) {
                 telemetry.addData("encoder-fwd", robot.liftMotor.getCurrentPosition() + "  busy=" + robot.liftMotor.isBusy());
                 telemetry.update();
                 idle();
             }
         }





        //---------------------- IGNORE
        //DPad->Not Complete Look At the notes right about here \/
        //Motor's Factory Given Name=
        /*gamepad1.dpad_up(robot.liftMotor.setPower(+1);
        gamepad1.dpad_down(robot.liftMotor.setPower(-1);
    */
    }
}