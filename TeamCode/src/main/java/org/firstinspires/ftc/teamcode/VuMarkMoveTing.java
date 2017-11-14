/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

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

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="VuMarkMoveTing", group ="Concept")


public class VuMarkMoveTing extends LinearOpMode {

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

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AVzCl0v/////AAAAGcfsmNB0+Ecxi9nnFUli4RtGGZORFTsrkrZTsSaEZcnHNkxhb5NbskfqT531gL1cmgLFZ5xxeICDdBlPxxEbD4JcUvUuIdXxpVesR7/EAFZ+DTSJT3YQb0sKm2SlOlfiMf7ZdCEUaXuymCZPB4JeoYdogDUOdsOrd0BTDV2Z+CtO3eSsHWfcY6bDLh8VJKSbeFdk533EzcA26uhfhwBxYlzbOsjPSVCB66P6GbIP9/UjI3lbTNi+tpCpnOZa2gwPjoTSeEjo9ZKtkPe3a/DpLq3OMnVwVnUmsDvoW++UbtOmg9WNFC/YkN7DCtMt91uPaJPL5vOERkA+uXliC1i44IT4EyfoN1ccLaJiXMFH63DE";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            // encoderDrive(DRIVE_SPEED,  -5,  -5, 5.0);  // S1: Forward 48// Inches with 5 Sec timeout - = forward (place holder for real #s)
            //encoderDrive(TURN_SPEED,   3, -3, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            //encoderDrive(TURN_SPEED, -48, 48, 5.0);  // S3: Reverse 24 Inches with 4 Sec timeout

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


                if (vuMark  != RelicRecoveryVuMark.LEFT) {


                        // Send telemetry message to signify robot waiting;
                        telemetry.addData("Status", "Resetting Encoders");    //
                        telemetry.update();

                        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        idle();

                        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//                        // Send telemetry message to indicate successful Encoder reset
//                        telemetry.addData("Path0",  "Starting at %7d :%7d",
//                                robot.motor1.getCurrentPosition(),
//                                robot.motor2.getCurrentPosition());
                    //also motor 3 and 4
                    //
//
//
// telemetry.update();





                }



                if (
                        robotRotate(==);
                        )

                if (vuMark  != RelicRecoveryVuMark.RIGHT) {

                    // Send telemetry message to signify robot waiting;
                    telemetry.addData("Status", "Resetting Encoders");    //
                    telemetry.update();

                    robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    idle();

                    robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // Send telemetry message to indicate successful Encoder reset
                    //telemetry.addData("Path0",  "Starting at %7d :%7d",
                    //        robot.motor1.getCurrentPosition(),
                    //        robot.motor2.getCurrentPosition());
                    //
                    //telemetry.update();


                    telemetry.addLine("This is da Right");



                }






                if (vuMark  != RelicRecoveryVuMark.CENTER) {

                    // Send telemetry message to signify robot waiting;
                    telemetry.addData("Status", "Resetting Encoders");    //
                    telemetry.update();

                    robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    idle();

                    robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // Send telemetry message to indicate successful Encoder reset
                    //telemetry.addData("Path0",  "Starting at %7d :%7d",
                    //        robot.motor1.getCurrentPosition(),
                    //        robot.motor2.getCurrentPosition());
                    //motor 3 and 4
                    //telemetry.update();


                }








                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    public void robotRotate(double turnToDegrees) throws InterruptedException {

        // Get Current IMU Heading

        // Calculate Target IMU Heading

        // Engage Motors in proper sequence until heading is reached





    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}


//find the jewels
//drop arm and read color sensor, knock off opponentes jewel
//find and read Vumark
//move to cyptobox and place glyph into key column
//park on ballencing stone