/*
Copyright 2024 FIRST Tech Challenge Team 22029

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.ColorSense;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Set;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.text.SimpleDateFormat;
import java.util.Date;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;


/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp(name = "ControllerModeV10" )

public class ControllerModeV10 extends OpMode {
    /* Declare OpMode members. */


    Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private DcMotor leftIntakeSlide;
    private DcMotor rightIntakeSlide;
    private TouchSensor rightIntakeLimitSwitch;
    private TouchSensor leftIntakeLimitSwitch;
    private TouchSensor upDownLimitSwitch;
    private Servo clawFlipper;
    private Servo outputClaw;
    private Servo intakeArmRight;
    private Servo intakeArmLeft;
    private ElapsedTime servoTimer;
    private Servo intakeClaw;
    private ColorSensor colorSensor;
    private Servo clawRotater;

    // Conversion factors for the GoBILDA linear slide
    private static final double TICKS_PER_REV = 537.6;        // Encoder ticks per revolution (GoBILDA 5202 motor)
    private static final double LEAD_SCREW_TRAVEL = 4.724410;      // Distance the slide travels per motor revolution (in inches)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / LEAD_SCREW_TRAVEL;


    // Constants for acceleration
    private static final double MAX_POWER = 0.8; // Maximum motor power
    private static final double MIN_POWER = 0.2; // Minimum motor power
    private static final double RAMP_DISTANCE = 200; // Distance in encoder ticks for ramping
    private int targetPosition = 1000; // Example target position


    // Servo Stuff
    private double clawRotaterParallel = .25;
    private double clawRotater45Deg = .5;
    private double clawRotaterPerpendicular = .75;
    private double clawRotater135Deg = 0;

    private double outputClawOpen = 0;
    private double outputClawClosed = .465;
    private double intakeArmPickUp = .18;
    private double intakeArmSearching = .25;
    private double intakeArmTransfer = .515;
    private double intakeClawOpen = .219;
    private double intakeClawClosed = .45;
    private double targetPositionIntakeArm = .325; // Desired servo position
    private double currentPositionIntakeArm = .325; // Initial position
    private double stepIntakeArm = 0.03; // Step size for smooth movement
    private double lastUpdateTime = 0; // Time of the last update
    private double interval = 20; // Interval between updates (in milliseconds)


    private double upStartPos = 0;

    private double intakeStartPos = 0;

    double adjustedTPosition = 0; // Default value

    private double slowFactor = 1;

    private double startTime = 0;
    private double startTime2 = 0;
    private double elapsedTime2 = 0;
    private double elapsedTime3 = 0;

    private ElapsedTime     runtime = new ElapsedTime();

    String getFloorSampleState = "rest"; // ready, armToPickUp,
    String transferState = "waiting";
    String slidePositionIntakeRight = "in";
    String slidePositionIntakeLeft = "in";
    String slidePositionUpDown = "in";
    String intakeArmPosition = "intakeArmTransfer";
    String getWallSpecimenState = "neutral";
    String waitingToRotate = "rest";
    String gameMode = "sample";

    boolean autoPickUp = false;
    public boolean sampleSeen = false;

     boolean prevPressed = false;

    String buttonPreviouslyPressed = "false";


    public double inchesPerPixel = 0.01015625;
    public double xMiddleScreen = 320;
    public double yMiddleScreen = 240;

    public double targetX;

    public double targetY;

    public double targetAngle;
    // OpenCV processing
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal portal;

    //ColorSense senseColor;

    public MecanumDrive drive;





    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");



         drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

       // telemetry.addData("red", ColorRange.RED);
       // telemetry.addData("yellow", ColorRange.YELLOW);
       // telemetry.addData("blue", ColorRange.BLUE);

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                //.setTargetColorRange(ColorRange.RED)
                .setTargetColorRange(ColorRange.YELLOW)

                //.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                      //  new Scalar( 100, 0,  0),
                      //  new Scalar(255, 255, 50)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setErodeSize(10)

                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // FIX: put this here, not above the init()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .build();

        //sensors
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        //wheels
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);


        // SLIDES:

        // Intake
        rightIntakeSlide = hardwareMap.get(DcMotor.class, "rightIntakeSlide");
        rightIntakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftIntakeSlide = hardwareMap.get(DcMotor.class, "leftIntakeSlide");
        leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);
        leftIntakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // updown slides
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // sensors
        rightIntakeLimitSwitch = hardwareMap.get(TouchSensor.class, "rightIntakeLimitSwitch");
        leftIntakeLimitSwitch = hardwareMap.get(TouchSensor.class, "leftIntakeLimitSwitch");
        upDownLimitSwitch = hardwareMap.get(TouchSensor.class, "upDownLimitSwitch");

        // SERVOS

        clawFlipper = hardwareMap.get(Servo.class, "clawFlipper");
        outputClaw = hardwareMap.get(Servo.class, "outputClaw");
        intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        clawRotater = hardwareMap.get(Servo.class, "clawRotater");
        intakeArmLeft.setDirection(Servo.Direction.REVERSE);
        clawFlipper.setDirection(Servo.Direction.REVERSE);

        outputClaw.setPosition(outputClawOpen);
        clawRotater.setPosition(clawRotaterParallel);


        servoTimer = new ElapsedTime();
        //intakeArmRight.setPosition(currentPositionIntakeArm); // Start at the initial position
        //intakeArmLeft.setPosition(currentPositionIntakeArm);
        intakeClaw.setPosition(intakeClawOpen);


    }



    public void setPos(double pos) {
        targetPositionIntakeArm = pos;
    }


    public void tick() {
        // Check if it's time for the next update
        if (servoTimer.milliseconds() - lastUpdateTime >= interval) {  //  if enough time is past
            // Update the servo position incrementally
            if (Math.abs(targetPositionIntakeArm - currentPositionIntakeArm) > stepIntakeArm) {  //  if we have far enough to go
                if (targetPositionIntakeArm > currentPositionIntakeArm) {
                    currentPositionIntakeArm += stepIntakeArm; // Move up
                } else {
                    currentPositionIntakeArm -= stepIntakeArm; // Move down
                }
                intakeArmLeft.setPosition(currentPositionIntakeArm); // Set the new position
                intakeArmRight.setPosition(currentPositionIntakeArm); // Set the new position
            } else {
                // Snap to the target position once close enough
                intakeArmLeft.setPosition(targetPositionIntakeArm);
                intakeArmRight.setPosition(targetPositionIntakeArm);
                currentPositionIntakeArm = targetPositionIntakeArm;

                //if (currentPositionIntakeArm == intakeArmPickUp) {

                //}
            }

            // Update the last update time
            lastUpdateTime = servoTimer.milliseconds();
        }

        // Other robot logic can go here without being blocked
        //telemetry.addData("Servo Position", clawFlipper.getPosition());
        //telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }





    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {



    }
    public boolean actionsAreRunning() {
        return !runningActions.isEmpty();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if(autoPickUp) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        }
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(10000, 70000, blobs);
        //ColorBlobLocatorProcessor.Util.filterByDensity(37000, 60000, blobs);
        //ColorBlobLocatorProcessor.Util.filterByAspectRatio(1.5, 3.5, blobs);
// Find the blob closest to the screen center
        double closestDistance = Double.MAX_VALUE;
        ColorBlobLocatorProcessor.Blob bestBlob = null;

        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            RotatedRect rect = blob.getBoxFit();
            double dx = rect.center.x - xMiddleScreen;
            double dy = rect.center.y - yMiddleScreen;
            double distance = Math.hypot(dx, dy);

            if (distance < closestDistance) {
                closestDistance = distance;
                bestBlob = blob;
            }
        }

        if (bestBlob != null) {
            RotatedRect boxFit = bestBlob.getBoxFit();
            org.opencv.core.Size size = boxFit.size;
            sampleSeen = true;
            // Normalize angle
            double angle = boxFit.angle;
            if (size.width < size.height) {
                angle -= 90;
            }
           // angle = Math.round(angle);

            // Set orientation from angle
            if (angle > 70.5 || angle <= -77.5) {
                targetAngle = 0.25; // Vertical
            } else if (angle >= 33.5 && angle <= 70.5) {
                targetAngle = 0.0; // 45 Left
            } else if (angle >= 11.5 && angle < 33.5) {
                targetAngle = 0.75; // 22.5 Left
            } else if (angle <= -55 && angle > -77.5) {
                targetAngle = 0.375; // 67.5 Right
            } else if (angle <= -33.5 && angle > -55) {
                targetAngle = 0.5; // 45 Right
            } else if (angle <= -11.5 && angle > -33.5) {
                targetAngle = 0.625; // 22.5 Right
            } else {
                targetAngle = 0.75; // Horizontal
            }

            // Compute position from center
            targetX = (boxFit.center.x - xMiddleScreen) * inchesPerPixel;
            targetY = (boxFit.center.y - yMiddleScreen) * inchesPerPixel;

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Target X (in)", targetX);
            telemetry.addData("Target Y (in)", targetY);
        }




        elapsedTime2 = servoTimer.milliseconds() - startTime;



        double targetInches = 14.0; // Example distance in inches 1
        double targetInchesIntake = 7.0;
        double targetInchesUpDown = 26; // Example distance in inches 2, full extensoin = 27 inches full extension
        double targetInchesHang = 20.75;  //  inches up yo go to for hang prep
        double targetInchesHangDown = 10;  //  inches down to go after the above for hanging
        double targetInchesMakeRoom = 1.5;
        double targetInchesSpeciman = 11.5;
        double targetInchesDown = 2.75;
        double targetInchesPass = 3;

        // Convert the target distance from inches to encoder ticks
        double tPositionInOut = (double)(targetInches * TICKS_PER_INCH);//1
        double tPositionUpDown = (double)(targetInchesUpDown * TICKS_PER_INCH);//2
        double tPositionHang = (double)(targetInchesHang * TICKS_PER_INCH);
        double tPositionHangDown = (double)(-(targetInchesHangDown * TICKS_PER_INCH));
        double tPositionSpeciman = (double)(targetInchesSpeciman * TICKS_PER_INCH);
        double tPositionMakeRoom = (double)(targetInchesMakeRoom * TICKS_PER_INCH);
        double tPositionDown = (double)(targetInchesDown * TICKS_PER_INCH);
        double tPositionPass = (double)(targetInchesPass * TICKS_PER_INCH);
        double tPositionIntake = (double)(targetInchesIntake * TICKS_PER_INCH);

        telemetry.addData("target inches up down:", targetInchesUpDown);
        telemetry.addData("target inches intake:", targetInches);
        telemetry.addData("target position up down:", tPositionUpDown);
        telemetry.addData("target position intake:", tPositionUpDown);




        elapsedTime3 = servoTimer.milliseconds() - startTime2;



        if(gamepad2.a && buttonPreviouslyPressed.equals("false")){
            buttonPreviouslyPressed = "true";
            startTime2 = servoTimer.milliseconds();
        }



        if (!gamepad2.a && buttonPreviouslyPressed.equals("true")) {




            if (elapsedTime3 < 300) {
                adjustedTPosition = tPositionInOut * .5; // Reduce tPositionIntake if button is held > 0.5 sec
            }else{
                adjustedTPosition = tPositionInOut;
            }


            buttonPreviouslyPressed = "false";

            if (slidePositionIntakeLeft.equals("in") && slidePositionIntakeRight.equals("in")) {
                leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftIntakeSlide.setTargetPosition((int) adjustedTPosition);
                rightIntakeSlide.setTargetPosition((int) adjustedTPosition);
                leftIntakeSlide.setPower(0.8);
                rightIntakeSlide.setPower(0.8);
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                clawFlipper.setPosition(0);

                setPos(intakeArmTransfer); // Switch to the partially extended position
                intakeArmPosition = "intakeArmTransfer";
                slidePositionIntakeLeft = "moving out";
                slidePositionIntakeRight = "moving out";

                intakeStartPos = leftIntakeSlide.getCurrentPosition();

            } else {
                if ( slidePositionIntakeLeft == "out" && slidePositionIntakeRight == "out" ) {
                    setPos(intakeArmTransfer);
                    rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slidePositionIntakeLeft = "moving in";
                    slidePositionIntakeRight = "moving in";
                    leftIntakeSlide.setPower(-0.75);
                    rightIntakeSlide.setPower(-0.75);
                    setPos(intakeArmTransfer);
                    intakeArmPosition = "intakeArmTransfer";
                    outputClaw.setPosition(outputClawOpen);

                }

            }

        }



        if( slidePositionIntakeLeft == "moving out") {

            double accEndPos2 = (adjustedTPosition - intakeStartPos) * .05 + intakeStartPos;
            double decStartPos2 = (adjustedTPosition - intakeStartPos) * .95 + intakeStartPos;

            if (leftIntakeSlide.getCurrentPosition() < accEndPos2) {

                double accPower2 = .8 + .2 * Math. pow(((leftIntakeSlide.getCurrentPosition() - intakeStartPos) / (accEndPos2 - intakeStartPos)), 2);

                leftIntakeSlide.setPower(accPower2);
                rightIntakeSlide.setPower(accPower2);

            }else {
                if(leftIntakeSlide.getCurrentPosition() < decStartPos2){

                    leftIntakeSlide.setPower(1);
                    rightIntakeSlide.setPower(1);
                }else{

                    if(leftIntakeSlide.getCurrentPosition() < adjustedTPosition){


                        double accPower2 = 1 - .2 * Math. pow(((leftIntakeSlide.getCurrentPosition() - decStartPos2) / (adjustedTPosition - decStartPos2)),2);

                        leftIntakeSlide.setPower(accPower2);
                        rightIntakeSlide.setPower(accPower2);

                    }else{
                        if (leftIntakeSlide.getCurrentPosition() >= adjustedTPosition ) {

                            leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            leftIntakeSlide.setPower(0);
                            rightIntakeSlide.setPower(0);


                            setPos(intakeArmTransfer);

                            intakeArmPosition = "intakeArmTransfer";
                            getFloorSampleState = "ready";
                            slidePositionIntakeLeft = "out";
                            slidePositionIntakeRight = "out";
                        } // intake stuff stops, this part is the same for both game modes
                    }
                }
            }
        }







        if (leftIntakeLimitSwitch.isPressed() && slidePositionIntakeLeft == "moving in" && gameMode == "sample") {
            leftIntakeSlide.setPower(0); //Stop the motor
            slidePositionIntakeLeft = "in";
        }

        if (rightIntakeLimitSwitch.isPressed() && slidePositionIntakeRight == "moving in" && gameMode == "sample") {
            rightIntakeSlide.setPower(0); //Stop the motor
            slidePositionIntakeRight = "in";
        }








        if(slidePositionIntakeLeft == "out" && gameMode == "speciman"){

            slowFactor = .5;
        }else{
            slowFactor = 1;
        }


        // up and sown slides only for sample game mode
        if (gamepad2.b && gameMode == "sample") {
            if (slidePositionUpDown == "in" && slidePositionIntakeLeft == "in" && slidePositionIntakeRight == "in") {
                leftSlide.setMode(RUN_USING_ENCODER);
                rightSlide.setMode(RUN_USING_ENCODER);

                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                leftSlide.setTargetPosition((int) tPositionUpDown);
                rightSlide.setTargetPosition((int) tPositionUpDown);

                leftSlide.setPower(0.85);
                rightSlide.setPower(0.85);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidePositionUpDown = "moving out";

                upStartPos = leftSlide.getCurrentPosition();

            } else {

                if ( slidePositionUpDown == "out" && gameMode == "sample") {
                    clawFlipper.setPosition(0); // flip in claw
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slidePositionUpDown = "moving in";
                    leftSlide.setPower(-0.7);
                    rightSlide.setPower(-0.7);

                }
            }
        }

        //BACKUP SLIDE RESET

        if (gamepad2.y){
            clawFlipper.setPosition(0); // flip in claw
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidePositionUpDown = "moving in";
            leftSlide.setPower(-0.4);
            rightSlide.setPower(-0.4);

            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidePositionIntakeLeft = "moving in";
            slidePositionIntakeRight = "moving in";
            leftIntakeSlide.setPower(-0.65);
            rightIntakeSlide.setPower(-0.65);
            setPos(intakeArmTransfer);
            intakeArmPosition = "intakeArmTransfer";
            outputClaw.setPosition(outputClawOpen);

        }


        if (upDownLimitSwitch.isPressed() && slidePositionUpDown == "moving in" && gameMode == "sample") {

            outputClaw.setPosition(outputClawOpen); //
            leftSlide.setPower(0); //Stop the motor
            rightSlide.setPower(0);
            slidePositionUpDown = "in";
        }

        if( slidePositionUpDown == "moving out" && gameMode == "sample") {

            double accEndPos = (tPositionUpDown - upStartPos) * .1 + upStartPos;
            double decStartPos = (tPositionUpDown - upStartPos) * .95 + upStartPos;

            if (leftSlide.getCurrentPosition() < accEndPos) {

                double accPower = .85 + .15 * Math. pow(((leftSlide.getCurrentPosition() - upStartPos) / (accEndPos - upStartPos)), 2);

                leftSlide.setPower(accPower);
                rightSlide.setPower(accPower);

            }else {
                if(leftSlide.getCurrentPosition() < decStartPos){

                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                }else{

                    if(leftSlide.getCurrentPosition() < tPositionUpDown){


                        double accPower = 1 - .15 * Math. pow(((leftSlide.getCurrentPosition() - decStartPos) / (tPositionUpDown - decStartPos)),2);

                        leftSlide.setPower(accPower);
                        rightSlide.setPower(accPower);

                    }else{
                        if (leftSlide.getCurrentPosition() >= tPositionUpDown) {

                            leftSlide.setMode(RUN_USING_ENCODER);
                            rightSlide.setMode(RUN_USING_ENCODER);
                            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            clawFlipper.setPosition(.59); // flip out claw
                            slidePositionUpDown = "out";
                        }
                    }
                }
            }
        }
/*
        if (gamepad2.right_trigger >= 0.9 && slidePositionIntakeLeft == "out"  && !(getFloorSampleState == "holding sample")
                && intakeArmPosition == "intakeArmTransfer" && autoPickUp == false && prevPressed == false) {
            autoPickUp = true;

            prevPressed = true;

            double finalTargetX = -targetY;
            double finalTargetY = -targetX;


            telemetry.addData("final Target X (in)", finalTargetX);
            telemetry.addData("final Target Y (in)", finalTargetY);
            clawRotater.setPosition(targetAngle);

            runningActions.add(new SequentialAction(
                    drive.actionBuilder(new Pose2d(0, 0, 0 * Math.PI / 4))

                            .splineToLinearHeading( new Pose2d(finalTargetX * 1.5, finalTargetY* 1.5, Math.toRadians(0)), Math.PI / 4,
                                    new TranslationalVelConstraint(250.0), new ProfileAccelConstraint(-160.0, 200.0))
                            // drop first SAMPLE
                            .build()


            ));

            getFloorSampleState = "armToPickUp";
            intakeClaw.setPosition(intakeClawOpen);

            setPos(intakeArmPickUp);
            startTime = servoTimer.milliseconds();
        }


        if(elapsedTime2 > 1000 && autoPickUp == true){

            autoPickUp = false;
            prevPressed = false;

        }

        /*
        float y;
        double x;
        float rx;
        double denominator;


        if(!autoPickUp) {
            // Remember, Y stick value is reversed
            y = -gamepad1.right_stick_y;
            // Factor to counteract imperfect strafing
            x = gamepad1.right_stick_x * 1.1;
            rx = gamepad1.left_stick_x;
            // Denominator is the largest motor power (absolute value) or 1.
            // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            // Make sure your ID's match your configuration
            drive.leftFront.setPower(((y + x + rx) / denominator) * slowFactor);
            drive.leftBack.setPower((((y - x) + rx) / denominator) * slowFactor);
            drive.rightFront.setPower((((y - x) - rx) / denominator) * slowFactor);
            drive.rightBack.setPower((((y + x) - rx) / denominator) * slowFactor);
        }

*/

        if(gamepad2.left_bumper){
            getFloorSampleState = "stopAllIntake";
            setPos(intakeArmTransfer);

            intakeArmPosition = "intakeArmTransfer";
            intakeClaw.setPosition(intakeClawOpen);

        }

        if (currentPositionIntakeArm == intakeArmPickUp && getFloorSampleState == "armToPickUp"
                && elapsedTime2 > 600 && !(getFloorSampleState == "stopAllIntake" )) { //grab sample after certain time


            intakeClaw.setPosition(intakeClawClosed);
            startTime = servoTimer.milliseconds();
            clawRotater.setPosition(clawRotaterParallel);
            getFloorSampleState = "holding sample";
            waitingToRotate = "waitingToRotate";

        }

        if( elapsedTime2 > 200 && getFloorSampleState == "holding sample"
                && waitingToRotate == "waitingToRotate"  ){


            clawRotater.setPosition(clawRotaterParallel);
            startTime = servoTimer.milliseconds();
            waitingToRotate = "rest";
        }

        if (getFloorSampleState == "holding sample" && waitingToRotate == "rest" && elapsedTime2 > 300
                && !(getFloorSampleState == "stopAllIntake" )){// set the intake arm position to transfer

            setPos(intakeArmTransfer);
            intakeArmPosition = "intakeArmTransfer";
            startTime = servoTimer.milliseconds();

        }


        if (Objects.equals(slidePositionIntakeLeft, "in") && Objects.equals(getFloorSampleState, "holding sample")
                && Objects.equals(intakeArmPosition, "intakeArmTransfer") && elapsedTime2 > 200
                && !(getFloorSampleState.equals("stopAllIntake"))){


            transferState = "mid transfer";
            outputClaw.setPosition(outputClawClosed); // mid transfer
            startTime = servoTimer.milliseconds();
            getFloorSampleState = "rest";

        }

        if (elapsedTime2 > 250 && Objects.equals(transferState, "mid transfer") &&
                !(Objects.equals(getFloorSampleState, "stopAllIntake")
        )){

            intakeClaw.setPosition(intakeClawOpen); // transfer done
            transferState =  "waiting";
            if(Objects.equals(gameMode, "speciman")){

                startTime = servoTimer.milliseconds();
                slidePositionIntakeLeft = "readyToMakeRoom";
                slidePositionIntakeRight = "readyToMakeRoom";
            }
        }

        if(elapsedTime2 > 100 && transferState == "waiting" && slidePositionIntakeRight == "readyToMakeRoom"
                && gameMode == "speciman"){// extend intake so slides can go up


            leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftIntakeSlide.setTargetPosition((int) tPositionMakeRoom);
            rightIntakeSlide.setTargetPosition((int) tPositionMakeRoom);
            leftIntakeSlide.setPower(0.75);
            rightIntakeSlide.setPower(0.75);
            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            setPos(intakeArmTransfer); // Switch to the partially extended position
            intakeArmPosition = "intakeArmTransfer";
            slidePositionIntakeLeft = "moving out for speciman";
            slidePositionIntakeRight = "moving out for speciman";





        }

        if (rightIntakeSlide.getCurrentPosition() >= tPositionMakeRoom
                && slidePositionIntakeLeft == "moving out for speciman"
                && slidePositionIntakeRight == "moving out for speciman" ) {

            leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftIntakeSlide.setPower(0);
            rightIntakeSlide.setPower(0);

            slidePositionIntakeLeft = "madeRoom";
            slidePositionIntakeRight = "madeRoom";
        }





        if (gamepad2.left_trigger >= 0.9 && slidePositionUpDown == "out"  && gameMode == "sample"){
            outputClaw.setPosition(outputClawOpen);

        }




        if (gamepad1.a && slidePositionUpDown == "in" && gameMode == "sample" ){

            gameMode = "speciman";


        }



        if (gamepad1.b && slidePositionUpDown == "in" && gameMode == "speciman"  ){

            gameMode = "sample";

        }


        telemetry.addData("gameMode", gameMode);

        if (gamepad2.right_trigger >= 0.9 && slidePositionUpDown == "in" && getWallSpecimenState == "ready to grab"){
            outputClaw.setPosition(outputClawClosed);
            getWallSpecimenState = "neutral";
        }




        // SPECIMAN MODE ------------------------------------------------------------


        telemetry.addData("SlidePosition:", slidePositionUpDown);
        telemetry.addData("intakeSlidePosition:", slidePositionIntakeLeft);





        // up and sown slides only for speciman game mode
        if (gamepad2.b && gameMode == "speciman") {
            if (slidePositionUpDown == "in" && slidePositionIntakeLeft == "madeRoom" && slidePositionIntakeRight == "madeRoom") {
                leftSlide.setMode(RUN_USING_ENCODER);
                rightSlide.setMode(RUN_USING_ENCODER);

                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftSlide.setTargetPosition((int) tPositionSpeciman);
                rightSlide.setTargetPosition((int) tPositionSpeciman);

                leftSlide.setPower(0.7);
                rightSlide.setPower(0.7);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidePositionUpDown = "moving out";

            } else {

                if ( slidePositionUpDown == "finished hang" && gameMode == "speciman" ) {
                    clawFlipper.setPosition(0); // flip in claw
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slidePositionUpDown = "moving in";
                    leftSlide.setPower(-0.6);
                    rightSlide.setPower(-0.6);

                }
            }
        }

        if (upDownLimitSwitch.isPressed() && slidePositionUpDown == "moving in" && gameMode == "speciman") {

            outputClaw.setPosition(outputClawOpen);
            leftSlide.setPower(0); //Stop the motor
            rightSlide.setPower(0);
            slidePositionUpDown = "in";



        }




        if (leftSlide.getCurrentPosition() >= tPositionSpeciman && slidePositionUpDown == "moving out" && gameMode == "speciman") {

            leftSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawFlipper.setPosition(.59); // flip out claw
            slidePositionUpDown = "out";



            setPos(intakeArmTransfer);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidePositionIntakeLeft = "moving in";
            slidePositionIntakeRight = "moving in";
            leftIntakeSlide.setPower(-0.4);
            rightIntakeSlide.setPower(-0.4);
            setPos(intakeArmTransfer);
            intakeArmPosition = "intakeArmTransfer";


        }




        if (leftIntakeLimitSwitch.isPressed() && slidePositionIntakeLeft == "moving in" && gameMode == "speciman") {
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntakeSlide.setPower(0); //Stop the motor
            slidePositionIntakeLeft = "in";





        }

        if (rightIntakeLimitSwitch.isPressed() && slidePositionIntakeRight == "moving in" && gameMode == "speciman") {
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightIntakeSlide.setPower(0); //Stop the motor
            slidePositionIntakeRight = "in";
        }




        if (gamepad2.left_trigger >= 0.9 && slidePositionUpDown == "out"  && gameMode == "speciman"){

            // ServoController clawFlipperController = clawFlipper.getController();
            // clawFlipperController.pwmDisable();

            rightSlide.setDirection(DcMotor.Direction.REVERSE);
            leftSlide.setDirection(DcMotor.Direction.FORWARD);

            leftSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(RUN_USING_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setTargetPosition((int) tPositionDown);
            rightSlide.setTargetPosition((int) tPositionDown);

            leftSlide.setPower(0.7);
            rightSlide.setPower(0.7);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePositionUpDown = "goingDownForSpecimanHang";



        }


        if ( !leftSlide.isBusy() && !rightSlide.isBusy() && slidePositionUpDown == "goingDownForSpecimanHang"
                && gameMode == "speciman") {

            leftSlide.setPower(0);
            rightSlide.setPower(0);
            rightSlide.setDirection(DcMotor.Direction.FORWARD);
            leftSlide.setDirection(DcMotor.Direction.REVERSE);

            outputClaw.setPosition(0);
            slidePositionUpDown = "finished hang";

        }



        if (gamepad2.x && Objects.equals(gameMode, "speciman")) {
            if (Objects.equals(slidePositionUpDown, "in") && slidePositionIntakeRight == "readyToMakeRoom" || slidePositionIntakeRight == "madeRoom") {
                startTime = servoTimer.milliseconds();
                leftSlide.setMode(RUN_USING_ENCODER);
                rightSlide.setMode(RUN_USING_ENCODER);

                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftSlide.setTargetPosition((int) tPositionPass);
                rightSlide.setTargetPosition((int) tPositionPass);

                leftSlide.setPower(0.6);
                rightSlide.setPower(0.6);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidePositionUpDown = "moving out for pass";
                startTime = servoTimer.milliseconds();

            }
        }


        if (leftSlide.getCurrentPosition() >= tPositionPass && Objects.equals(slidePositionUpDown, "moving out for pass") && Objects.equals(gameMode, "speciman")) {

            leftSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // flip out claw
            slidePositionUpDown = "out for pass";



            setPos(intakeArmTransfer);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidePositionIntakeLeft = "moving in";
            slidePositionIntakeRight = "moving in";
            leftIntakeSlide.setPower(-0.4);
            rightIntakeSlide.setPower(-0.4);
            setPos(intakeArmTransfer);
            intakeArmPosition = "intakeArmTransfer";
            clawFlipper.setPosition(.59);


        }

        telemetry.addData("elapsed time", elapsedTime2);


        if ( elapsedTime2 > 900  && Objects.equals(slidePositionUpDown, "out for pass")){
            slidePositionUpDown = "ready to go down";
            outputClaw.setPosition(outputClawOpen);
            startTime = servoTimer.milliseconds();
        }


        if ( elapsedTime2 > 2000  && Objects.equals(slidePositionUpDown, "ready to go down")){

            outputClaw.setPosition(outputClawOpen);
            clawFlipper.setPosition(0); // flip in claw
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidePositionUpDown = "moving in";
            leftSlide.setPower(-0.6);
            rightSlide.setPower(-0.6);
        }




        // SPECIMAN HANG DONE

        // Claw Rotations
        if (gamepad2.dpad_right && intakeArmPosition == "intakeArmSearching"){
            clawRotater.setPosition(clawRotater45Deg);
        }
        if (gamepad2.dpad_up && intakeArmPosition == "intakeArmSearching"){
            clawRotater.setPosition(clawRotaterParallel);
        }

        if (gamepad2.dpad_down && intakeArmPosition == "intakeArmSearching"){
            clawRotater.setPosition(clawRotaterPerpendicular);
        }
        if (gamepad2.dpad_left && intakeArmPosition == "intakeArmSearching"){
            clawRotater.setPosition(clawRotater135Deg);
        }// claw rotations ends

        //Hang
        if (gamepad1.right_bumper) {
            if (slidePositionUpDown == "in" && slidePositionIntakeLeft == "in") {
                clawFlipper.setPosition(0);
                leftSlide.setMode(RUN_USING_ENCODER);
                rightSlide.setMode(RUN_USING_ENCODER);

                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftSlide.setTargetPosition((int) tPositionHang);
                rightSlide.setTargetPosition((int) tPositionHang);

                leftSlide.setPower(0.65);
                rightSlide.setPower(0.65);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidePositionUpDown = "moving out for hang";

            } else {  //  going down

                if ( slidePositionUpDown == "out for hang") {
                    slidePositionUpDown = "moving in from hang";

                    leftSlide.setMode(RUN_USING_ENCODER);
                    rightSlide.setMode(RUN_USING_ENCODER);

                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftSlide.setTargetPosition((int) tPositionHangDown);
                    rightSlide.setTargetPosition((int) tPositionHangDown);

                    leftSlide.setPower(0.5);
                    rightSlide.setPower(0.5);

                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while(tPositionHangDown <= leftSlide.getCurrentPosition());

                    leftSlide.setMode(RUN_USING_ENCODER);
                    rightSlide.setMode(RUN_USING_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
            }
        }

        if (leftSlide.getCurrentPosition() >= tPositionHang && slidePositionUpDown == "moving out for hang") {

            leftSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidePositionUpDown = "out for hang";
        }


        // hang ends

        // Smoothly move the servo
        tick();


        // ROAD RUNNER STUFF

        TelemetryPacket packet = new TelemetryPacket();

// updated based on gamepads
/*
        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
*/



        // Display telemetry for debugging
        telemetry.addData("Current Position Intake Arm", currentPositionIntakeArm);
        telemetry.addData("Target Position Intake Arm", targetPositionIntakeArm);
        //telemetry.addData("Color", niceColor);
        telemetry.update();





            // Check if we should start auto pick up
            if (gamepad2.right_trigger >= 0.9 && Objects.equals(slidePositionIntakeLeft, "out") && !(Objects.equals(getFloorSampleState, "holding sample"))
                    && Objects.equals(intakeArmPosition, "intakeArmTransfer") && !autoPickUp && !prevPressed && sampleSeen) {
                autoPickUp = true;
                prevPressed = true;

                // Calculate target for action
                double finalTargetX = -targetY;
                double finalTargetY = -targetX;

                telemetry.addData("final Target X (in)", finalTargetX);
                telemetry.addData("final Target Y (in)", finalTargetY);
                clawRotater.setPosition(targetAngle);

                // Add the new action to the runningActions list
                runningActions.add(new SequentialAction(
                        drive.actionBuilder(new Pose2d(0, 0, 0 * Math.PI / 4))
                                .splineToLinearHeading(new Pose2d(finalTargetX * 1.5, finalTargetY * 1.5, Math.toRadians(0)),
                                        Math.PI / 4, new TranslationalVelConstraint(200.0), new ProfileAccelConstraint(-160.0, 200.0))
                                // drop first SAMPLE
                                .build()
                ));

                // Change the state to pick up mode
                getFloorSampleState = "armToPickUp";
                intakeClaw.setPosition(intakeClawOpen);
                setPos(intakeArmPickUp);
                //startTime = servoTimer.milliseconds();
            }
        //if (gamepad1.back) {
           // runningActions.clear(); // Kill all actions manually

       // }

            // Check if autoPickUp is done (only disable it if the action has finished running)
            if (autoPickUp && !actionsAreRunning()) {
                autoPickUp = false;
                sampleSeen = false;
                prevPressed = false;

            }

            // Reset prevPressed when the right trigger is released
            if (gamepad2.right_trigger < 0.1) {
                prevPressed = false;
            }




            // Joystick control for manual movement if autoPickUp is not active------- && !actionsAreRunning()
            if (!autoPickUp  && runningActions.isEmpty()) {
                runningActions.clear(); // Kills auto actions


                float y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.right_stick_x * 1.1; // Factor to counteract imperfect strafing
                float rx = gamepad1.left_stick_x;
                //double denominator = Math.max(Math.max(Math.abs(y), Math.abs(x)), Math.abs(rx)); // Ensure max motor power ratio
                BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                double  denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
                // Apply control to motors
                FrontLeft.setPower(((y + x + rx) / denominator) * slowFactor);
                BackLeft.setPower((((y - x) + rx) / denominator) * slowFactor);
                FrontRight.setPower((((y - x) - rx) / denominator) * slowFactor);
                BackRight.setPower((((y + x) - rx) / denominator) * slowFactor);
            }


            if(autoPickUp) {
                // Update running actions
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;
            }
                // Send telemetry packet
                dash.sendTelemetryPacket(packet);







    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public class pickUpSample implements Action {
        Servo intakeClaw;

        Servo intakeArmLeft;
        Servo intakeArmRight;



        public pickUpSample(Servo O, Servo L, Servo R ) {
            this.intakeArmLeft = L;
            this.intakeArmRight =R;

            this.intakeClaw = O;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeArmLeft.setPosition(.18);
            intakeArmRight.setPosition(.18);

            intakeClaw.setPosition(.45);




            return false;
        }
    }
}




