package org.firstinspires.ftc.teamcode;



import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;


import com.acmerobotics.roadrunner.ParallelAction;
// Conversion factors for the GoBILDA linear slide
@Autonomous (name = "fiveSpecimanAuto" )
public class fiveSpecimanAuto extends LinearOpMode {


    private static final double TICKS_PER_REV = 537.6;        // Encoder ticks per revolution (GoBILDA 5202 motor)
    private static final double LEAD_SCREW_TRAVEL = 4.724410;      // Distance the slide travels per motor revolution (in inches)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / LEAD_SCREW_TRAVEL;

    double targetInchesDown = 3.1;
    double targetInchesUpDown = 11.5;
    double targetInchesUp = 2;

    double targetInchesIn = 10;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //sensors
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        // SLIDES:

        // Intake
        DcMotor rightIntakeSlide = hardwareMap.get(DcMotor.class, "rightIntakeSlide");
        DcMotor leftIntakeSlide = hardwareMap.get(DcMotor.class, "leftIntakeSlide");
        leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);

        // updown slides
        DcMotor rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        DcMotor leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);

        // sensors
        TouchSensor rightIntakeLimitSwitch = hardwareMap.get(TouchSensor.class, "rightIntakeLimitSwitch");
        TouchSensor leftIntakeLimitSwitch = hardwareMap.get(TouchSensor.class, "leftIntakeLimitSwitch");
        TouchSensor upDownLimitSwitch = hardwareMap.get(TouchSensor.class, "upDownLimitSwitch");

        // SERVOS

        Servo clawFlipper = hardwareMap.get(Servo.class, "clawFlipper");
        Servo outputClaw = hardwareMap.get(Servo.class, "outputClaw");
        Servo intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        Servo intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        Servo clawRotater = hardwareMap.get(Servo.class, "clawRotater");
        clawFlipper.setDirection(Servo.Direction.REVERSE);
        intakeArmLeft.setDirection(Servo.Direction.REVERSE);
        clawFlipper.setPosition(.585);// grabbing off wall and speciman scoreing
        outputClaw.setPosition(.477);//close
        intakeClaw.setPosition(.22);//open
        intakeArmLeft.setPosition(.235);//.22
        intakeArmRight.setPosition(.235);
        clawRotater.setPosition(.25);

        waitForStart();



        //drive to the submersible while extending the slides
        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(0, 0, 0 * Math.PI / 4))

                        //.splineToLinearHeading( new Pose2d(-30, 0, Math.toRadians(0)), Math.PI / 4,
                              //  new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-30.0, 75.0))

                        .lineToX(-28, new TranslationalVelConstraint(55.0))
                        .build(),


                // extendSlide action
                new extendSlide(leftSlide, rightSlide, clawFlipper)// 1600


        ));



        Actions.runBlocking( new SequentialAction(
                new scoreTheSpeciman(leftSlide, rightSlide, upDownLimitSwitch, outputClaw, clawFlipper),//First speciman
                new SleepAction(0.18),
                new resetServos(clawFlipper, outputClaw)

        ));




        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-28, 0, 0 * Math.PI / 4))
                        //.splineToConstantHeading(new Vector2d(-20,25), Math.PI / 4)
                        .splineToLinearHeading( new Pose2d(-20, 21, Math.toRadians(125)), Math.PI / 4,
                                new TranslationalVelConstraint(70.0))
                        .build(),

                // extendSlide action
                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),// bring slides down
                new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 1500),
                new rotateIntakeClaw(clawRotater, outputClaw)
        ));


        Actions.runBlocking( new SequentialAction(
                new resetServos(clawFlipper, outputClaw),//grabe First sample
                new pickUpSample(intakeClaw,intakeArmLeft, intakeArmRight)

        ));



        Actions.runBlocking( new SequentialAction(

                drive.actionBuilder(new Pose2d(-20, 21, 125 * Math.PI / 4))

                        .splineToLinearHeading( new Pose2d(-16, 20, Math.toRadians(80)), Math.PI / 4)
                        // drop first SAMPLE
                        .build(),
                new drop(clawFlipper, outputClaw, intakeClaw)
        ));



        Actions.runBlocking( new SequentialAction(

                drive.actionBuilder(new Pose2d(-16, 20, 80 * Math.PI / 4))

                        .splineToLinearHeading( new Pose2d(-18, 31, Math.toRadians(125)), Math.PI / 4)
                        .build(),
                new resetServos(clawFlipper, outputClaw),
                new pickUpSample(intakeClaw,intakeArmLeft, intakeArmRight) // grab second sample
        ));

        Actions.runBlocking( new SequentialAction(

                drive.actionBuilder(new Pose2d(-18, 31, 125 * Math.PI / 4))

                        .splineToLinearHeading( new Pose2d(-16, 31.5, Math.toRadians(55)), Math.PI / 4)
                        .build(),
                new drop(clawFlipper, outputClaw, intakeClaw),// drop second
                new rotateIntakeClawPerp(clawRotater, outputClaw)

        ));



        Actions.runBlocking( new SequentialAction(

                new ParallelAction(
                drive.actionBuilder(new Pose2d(-16, 31.5, 55 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(-38, 40, Math.toRadians(90)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 180.0))
                        .splineToLinearHeading( new Pose2d(-38, 47, Math.toRadians(90)), Math.PI / 4,
                                new TranslationalVelConstraint(150.0))
                        .build(),
                        new resetServos(clawFlipper, outputClaw),
                        new intakeSlideInALittle(leftIntakeSlide,rightIntakeSlide)
                        ),

                new pickUpSample(intakeClaw,intakeArmLeft, intakeArmRight)// grab the third sample
        ));





        Actions.runBlocking( new SequentialAction(

                drive.actionBuilder(new Pose2d(-37, 47, 90 * Math.PI / 4))

                        .splineToLinearHeading( new Pose2d(-7, 40, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 180.0))
                        .build(),

                new drop(clawFlipper, outputClaw, intakeClaw),
                new rotateIntakeClawParalel(clawRotater, outputClaw),
                new flipTheClawTransfer(clawFlipper, outputClaw)

        ));


        //Second ONE -----------------------------------------------------------------------------
        Actions.runBlocking( new SequentialAction(

                drive.actionBuilder(new Pose2d(-7, 40, 0 * Math.PI / 4))

                        //.splineToLinearHeading( new Pose2d(-15, 30, Math.toRadians(0)), Math.PI / 4,
                               // new TranslationalVelConstraint(200.0))
                        .splineToLinearHeading( new Pose2d(-15, 32, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-30.0, 190.0))

                        //.strafeToSplineHeading(new Vector2d(-21, 19.5), Math.toRadians(180),
                               // new TranslationalVelConstraint(60.0))8

                        .splineToLinearHeading( new Pose2d(-9, 27, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(160.0))

                        .build(),

                        new pickUpSpeciman(intakeClaw,intakeArmLeft, intakeArmRight, outputClaw)


        ));











        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-9, 27, 0 * Math.PI / 4))
                       // .splineToLinearHeading( new Pose2d(-20, 5, Math.toRadians(0)), Math.PI / 4,
                               // new TranslationalVelConstraint(110.0))


                        .splineToLinearHeading( new Pose2d(-37, -4, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))

                        .build(),


                new SequentialAction(

                        new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater),
                        new SleepAction(0.6),
                        new transfer(clawFlipper, outputClaw, intakeClaw),
                        new extendIntakeSlide2(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 200),
                        new extendSlide(leftSlide, rightSlide, clawFlipper),// 1600 slides up for second specimen
                        new SleepAction(0.3),
                        new flipTheClaw(clawFlipper, outputClaw)
        )));





        Actions.runBlocking( new SequentialAction(


                new scoreTheSpeciman(leftSlide, rightSlide, upDownLimitSwitch, outputClaw, clawFlipper),// score the Second Speciman
                new SleepAction(0.15),
                new resetServos(clawFlipper, outputClaw)


        ));



//THIRD ONE -----------------------------------------------------------------

        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-37, -4, 0 * Math.PI / 4))
                        //.splineToConstantHeading(new Vector2d(-20,25), Math.PI / 4)
                        .splineToLinearHeading( new Pose2d(-15, 26, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))
                        .splineToLinearHeading( new Pose2d(-9, 26, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0))

                        .build(),
                new SequentialAction(
                // extendSlide action

                new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 300),
                new flipTheClawTransfer(clawFlipper, outputClaw),
                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),// bring slides down

                new resetServos(clawFlipper, outputClaw)

                )));


        Actions.runBlocking( new SequentialAction(


                new pickUpSpeciman(intakeClaw,intakeArmLeft, intakeArmRight, outputClaw)


        ));









        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-9, 27, 0 * Math.PI / 4))//x -20 , y 5
                       // .splineToLinearHeading( new Pose2d(-25, -8, Math.toRadians(0)), Math.PI / 4,
                                //new TranslationalVelConstraint(170.0))

                        .splineToLinearHeading( new Pose2d(-37, -4, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))
                        .build(),

                new SequentialAction(

                        new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater),
                        new SleepAction(0.6),
                        new transfer(clawFlipper, outputClaw, intakeClaw),
                        new extendIntakeSlide2(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 200),
                        new extendSlide(leftSlide, rightSlide, clawFlipper ),// 1600 Thrid time scoring
                        new SleepAction(0.3),
                        new flipTheClaw(clawFlipper, outputClaw)
                )));




        Actions.runBlocking( new SequentialAction(


                new scoreTheSpeciman(leftSlide, rightSlide, upDownLimitSwitch, outputClaw, clawFlipper),
                new SleepAction(0.15),
                new resetServos(clawFlipper, outputClaw)


        ));





        //FOUR ------------------------------

        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-37, -4, 0 * Math.PI / 4))
                        //.splineToConstantHeading(new Vector2d(-20,25), Math.PI / 4)
                        .splineToLinearHeading( new Pose2d(-12, 25.75, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))

                        .splineToLinearHeading( new Pose2d(-9.25, 25.75, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0))



                        .build(),

                // extendSlide action
                new flipTheClawTransfer(clawFlipper, outputClaw),
                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),// bring slides down
                new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 300),
                new resetServos(clawFlipper, outputClaw)
        ));


        Actions.runBlocking( new SequentialAction(


                new pickUpSpeciman(intakeClaw,intakeArmLeft, intakeArmRight, outputClaw)


        ));






        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-9.25, 25.75, 0 * Math.PI / 4))//-20 , 5

                .splineToLinearHeading( new Pose2d(-37, -4, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))
                        .build(),
                new SequentialAction(

                        new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater),
                        new SleepAction(0.6),
                        new transfer(clawFlipper, outputClaw, intakeClaw),
                        new extendIntakeSlide2(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 200),
                        new extendSlide(leftSlide, rightSlide, clawFlipper),// 1600
                        new SleepAction(0.3),
                        new flipTheClaw(clawFlipper, outputClaw)
        )));




        Actions.runBlocking( new SequentialAction(


                new scoreTheSpeciman(leftSlide, rightSlide, upDownLimitSwitch, outputClaw, clawFlipper),
                new SleepAction(0.15),
                new resetServos(clawFlipper, outputClaw)


        ));



        // FIVEEEEEEE--------------------------------



        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-37, -4, 0 * Math.PI / 4))
                        //.splineToConstantHeading(new Vector2d(-20,25), Math.PI / 4)
                        .splineToLinearHeading( new Pose2d(-12, 25.25, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))

                        .splineToLinearHeading( new Pose2d(-9, 25.25, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0))



                        .build(),

                // extendSlide action
                new flipTheClawTransfer(clawFlipper, outputClaw),
                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),// bring slides down
                new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 300),
                new resetServos(clawFlipper, outputClaw)
        ));


        Actions.runBlocking( new SequentialAction(


                new pickUpSpeciman(intakeClaw,intakeArmLeft, intakeArmRight, outputClaw)


        ));






        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(-9.25, 26, 0 * Math.PI / 4))//-20 , 5
                        //.splineToLinearHeading( new Pose2d(-25, -12, Math.toRadians(0)), Math.PI / 4,
                        // new TranslationalVelConstraint(170.0)

                        .splineToLinearHeading( new Pose2d(-37, -4, Math.toRadians(0)), Math.PI / 4,
                                new TranslationalVelConstraint(270.0), new ProfileAccelConstraint(-30.0, 190.0))
                        .build(),
                new SequentialAction(

                        new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater),
                        new SleepAction(0.6),
                        new transfer(clawFlipper, outputClaw, intakeClaw),
                        new extendIntakeSlide2(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 200),
                        new extendSlide(leftSlide, rightSlide, clawFlipper),// 1600
                        new SleepAction(0.3),
                        new flipTheClaw(clawFlipper, outputClaw)
                )));





        Actions.runBlocking( new SequentialAction(


                new scoreTheSpeciman(leftSlide, rightSlide, upDownLimitSwitch, outputClaw, clawFlipper),
                new SleepAction(0.15),
                new resetServos(clawFlipper, outputClaw),
                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw )// bring slides down

        ));



sleep(4000);












    }


    public class scoreTheSpeciman implements Action {
        DcMotor leftSlide;
        DcMotor rightSlide;
        TouchSensor upDownLimitSwitch;
        Servo outputClaw;
        Servo clawFlipper;

        double tPositionDown = targetInchesDown * TICKS_PER_INCH; // Ensure targetInchesDown is defined

        public scoreTheSpeciman(DcMotor L, DcMotor R, TouchSensor T, Servo O, Servo F) {
            this.leftSlide = L;
            this.rightSlide = R;
            this.upDownLimitSwitch = T;
            this.outputClaw = O;
            this.clawFlipper = F;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Set motor directions
            rightSlide.setDirection(DcMotor.Direction.REVERSE);
            leftSlide.setDirection(DcMotor.Direction.FORWARD);

            // Set encoders
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target position
            leftSlide.setTargetPosition((int) tPositionDown);
            rightSlide.setTargetPosition((int) tPositionDown);

            leftSlide.setPower(0.8);
            rightSlide.setPower(0.8);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // **Wait until slides reach the target position**
            if (!leftSlide.isBusy() && !rightSlide.isBusy()) {
                // Stop motors once target is reached
                leftSlide.setPower(0);
                rightSlide.setPower(0);

                // Reset directions if needed
                rightSlide.setDirection(DcMotor.Direction.FORWARD);
                leftSlide.setDirection(DcMotor.Direction.REVERSE);

                return true; // **This tells Road Runner that the action is finished**
            }

            return false; // **Keep running until motors finish moving**
        }
    }

    public class intakeSlideInALittle implements Action {
        DcMotor leftIntakeSlide;
        DcMotor rightIntakeSlide;
        double tPositionIn = (double) (targetInchesIn * TICKS_PER_INCH);

        public intakeSlideInALittle(DcMotor L, DcMotor R) {

            this.leftIntakeSlide = L;
            this.rightIntakeSlide = R;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            rightIntakeSlide.setDirection(DcMotor.Direction.REVERSE);
            leftIntakeSlide.setDirection(DcMotor.Direction.FORWARD);

            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftIntakeSlide.setTargetPosition((int) tPositionIn);
            rightIntakeSlide.setTargetPosition((int) tPositionIn);

            leftIntakeSlide.setPower(0.8);
            rightIntakeSlide.setPower(0.8);

            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (leftIntakeSlide.getCurrentPosition() >= tPositionIn) {
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightIntakeSlide.setDirection(DcMotor.Direction.FORWARD);
                leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);



            }
            return false;
        }
    }
    public class pickUpSample implements Action {
        Servo intakeClaw;

        Servo intakeArmLeft;
        Servo intakeArmRight;

        double tPositionUp = (double) (targetInchesUp * TICKS_PER_INCH);

        public pickUpSample(Servo O, Servo L, Servo R ) {
            this.intakeArmLeft = L;
            this.intakeArmRight =R;

            this.intakeClaw = O;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeArmLeft.setPosition(.18);
            intakeArmRight.setPosition(.18);
            sleep(50);
            intakeClaw.setPosition(.45);
            sleep(100);
            intakeArmLeft.setPosition(.25);
            intakeArmRight.setPosition(.25);


            return false;
        }
    }


    public class pickUpSpeciman implements Action {
        Servo intakeClaw;

        Servo intakeArmLeft;
        Servo intakeArmRight;
        Servo outputClaw;
        double tPositionUp = (double) (targetInchesUp * TICKS_PER_INCH);

        public pickUpSpeciman(Servo O, Servo L, Servo R, Servo P ) {
            this.intakeArmLeft = L;
            this.intakeArmRight =R;
            this.outputClaw = P;
            this.intakeClaw = O;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeArmLeft.setPosition(.18);
            intakeArmRight.setPosition(.18);
            sleep(100);
            intakeClaw.setPosition(.45);
            sleep(100);
            intakeArmLeft.setPosition(.515);
            intakeArmRight.setPosition(.515);

            outputClaw.setPosition(0);
            return false;
        }
    }



    public class extendSlide implements Action {
        DcMotor leftSlide;
        DcMotor rightSlide;
        Servo clawFlipper;
        double tPositionUpDown = (double) (targetInchesUpDown * TICKS_PER_INCH);

        public extendSlide(DcMotor L, DcMotor R, Servo S) {
            this.leftSlide = L;
            this.rightSlide = R;

            this.clawFlipper = S;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            rightSlide.setDirection(DcMotor.Direction.FORWARD);
            leftSlide.setDirection(DcMotor.Direction.REVERSE);

            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setTargetPosition((int) tPositionUpDown);
            rightSlide.setTargetPosition((int) tPositionUpDown);

            leftSlide.setPower(0.85);
            rightSlide.setPower(0.85);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            if (leftSlide.getCurrentPosition() >= tPositionUpDown) {
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            return false;

        }

    }

    public class extendIntakeSlide2 implements Action {
        DcMotor leftIntakeSlide;
        DcMotor rightIntakeSlide;

        Servo intakeArmRight;
        Servo intakeArmLeft;

        double tPositionUpDown = (double) (targetInchesUpDown * TICKS_PER_INCH);

        public extendIntakeSlide2(DcMotor L, DcMotor R, Servo I, Servo A, double p) {
            this.leftIntakeSlide = L;
            this.rightIntakeSlide = R;
            this.tPositionUpDown = p;
            this.intakeArmLeft = I;
            this.intakeArmRight = A;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            rightIntakeSlide.setDirection(DcMotor.Direction.FORWARD);
            leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);

            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftIntakeSlide.setTargetPosition((int) tPositionUpDown);
            rightIntakeSlide.setTargetPosition((int) tPositionUpDown);

            leftIntakeSlide.setPower(0.6);
            rightIntakeSlide.setPower(0.6);

            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (leftIntakeSlide.getCurrentPosition() >= tPositionUpDown) {
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            return false;
        }

    }




    public class extendIntakeSlide implements Action {
        DcMotor leftIntakeSlide;
        DcMotor rightIntakeSlide;

        Servo intakeArmRight;
        Servo intakeArmLeft;

        double tPositionUpDown = (double) (targetInchesUpDown * TICKS_PER_INCH);

        public extendIntakeSlide(DcMotor L, DcMotor R, Servo I, Servo A, double p) {
            this.leftIntakeSlide = L;
            this.rightIntakeSlide = R;
            this.tPositionUpDown = p;
            this.intakeArmLeft = I;
            this.intakeArmRight = A;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeArmRight.setPosition(.25);
            intakeArmLeft.setPosition(.25);
            rightIntakeSlide.setDirection(DcMotor.Direction.FORWARD);
            leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);

            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightIntakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftIntakeSlide.setTargetPosition((int) tPositionUpDown);
            rightIntakeSlide.setTargetPosition((int) tPositionUpDown);

            leftIntakeSlide.setPower(0.6);
            rightIntakeSlide.setPower(0.6);

            leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (leftIntakeSlide.getCurrentPosition() >= tPositionUpDown) {
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftIntakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            return false;
        }

    }



    public class intakeSlideIn implements Action {

        DcMotor leftIntakeSlide;
        DcMotor rightIntakeSlide;
        Servo outputClaw;
        Servo clawRotater;
        TouchSensor rightIntakeLimitSwitch;




        boolean isMovingIntakeSlides = true;

        public intakeSlideIn(DcMotor L, DcMotor R, TouchSensor T, Servo O, Servo CR) {

            this.leftIntakeSlide = L;
            this.rightIntakeSlide = R;
            this.rightIntakeLimitSwitch = T;
            this.outputClaw = O;
            this.clawRotater = CR;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawRotater.setPosition(.25);
            if (isMovingIntakeSlides) {
                if (!rightIntakeLimitSwitch.isPressed()) {
                    rightIntakeSlide.setDirection(DcMotor.Direction.FORWARD);
                    leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);
                    rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    leftIntakeSlide.setPower(-0.8);
                    rightIntakeSlide.setPower(-0.8);
                } else {
                    // Stop the motors when the limit switch is pressed
                    leftIntakeSlide.setPower(0);
                    rightIntakeSlide.setPower(0);

                    isMovingIntakeSlides = false; // Stop further motor commands
                }
            }

            return false;
        }
    }

    public class slideDown implements Action {

        DcMotor leftSlide;
        DcMotor rightSlide;
        TouchSensor upDownLimitSwitch;

        Servo clawFlipper;

        Servo outputClaw;


        boolean isMovingSlides = true;

        public slideDown(DcMotor L, DcMotor R, TouchSensor T, Servo F, Servo O) {

            this.leftSlide = L;
            this.rightSlide = R;
            this.upDownLimitSwitch = T;
            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            clawFlipper.setPosition(0);
            if (isMovingSlides) {
                if (!upDownLimitSwitch.isPressed()) {
                    rightSlide.setDirection(DcMotor.Direction.FORWARD);
                    leftSlide.setDirection(DcMotor.Direction.REVERSE);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    leftSlide.setPower(-0.8);
                    rightSlide.setPower(-0.8);
                } else {
                    // Stop the motors when the limit switch is pressed
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    outputClaw.setPosition(0);
                    isMovingSlides = false; // Stop further motor commands
                }
            }

            return false;
        }
    }

    public class drop implements Action {

        Servo clawFlipper;

        Servo outputClaw;

        Servo intakeClaw;

        public drop( Servo F, Servo O, Servo I) {

            this.intakeClaw = I;
            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            intakeClaw.setPosition(.22);//open



            return false;
        }
    }


    public class transfer implements Action {

        Servo clawFlipper;

        Servo outputClaw;

        Servo intakeClaw;

        public transfer( Servo F, Servo O, Servo I) {

            this.intakeClaw = I;
            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            outputClaw.setPosition(.495);//close
            sleep(100);
            intakeClaw.setPosition(.22);//open



            return false;
        }
    }






    public class scoreTheSample implements Action {

        Servo clawFlipper;

        Servo outputClaw;

        public scoreTheSample( Servo F, Servo O) {


            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            outputClaw.setPosition(0);



            return false;
        }
    }

    public class resetServos implements Action {

        Servo clawFlipper;

        Servo outputClaw;

        public resetServos( Servo F, Servo O) {


            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            outputClaw.setPosition(0);



            return false;
        }
    }


    public class flipTheClaw implements Action {

        Servo clawFlipper;

        Servo outputClaw;

        public flipTheClaw( Servo F, Servo O) {


            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            clawFlipper.setPosition(.585);//.59



            return false;
        }
    }


    public class flipTheClawTransfer implements Action {

        Servo clawFlipper;

        Servo outputClaw;

        public flipTheClawTransfer( Servo F, Servo O) {


            this.clawFlipper = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            clawFlipper.setPosition(0);//.59



            return false;
        }
    }



    public class rotateIntakeClaw implements Action {

        Servo clawRotater;

        Servo outputClaw;

        public rotateIntakeClaw( Servo F, Servo O) {


            this.clawRotater = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            clawRotater.setPosition(0);



            return false;
        }
    }

    public class rotateIntakeClawParalel implements Action {

        Servo clawRotater;

        Servo outputClaw;

        public rotateIntakeClawParalel( Servo F, Servo O) {


            this.clawRotater = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            clawRotater.setPosition(.25);



            return false;
        }
    }



    public class rotateIntakeClawPerp implements Action {

        Servo clawRotater;

        Servo outputClaw;

        public rotateIntakeClawPerp( Servo F, Servo O) {


            this.clawRotater = F;
            this.outputClaw = O;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            clawRotater.setPosition(.75);



            return false;
        }
    }

}
