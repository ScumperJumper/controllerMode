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
@Autonomous (name = "BasketAuto" )
public class BasketAuto extends LinearOpMode {


    private static final double TICKS_PER_REV = 537.6;        // Encoder ticks per revolution (GoBILDA 5202 motor)
    private static final double LEAD_SCREW_TRAVEL = 4.724410;      // Distance the slide travels per motor revolution (in inches)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / LEAD_SCREW_TRAVEL;


    double targetInchesUpDown = 35;
    double targetInchesUp = 2;


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
        clawFlipper.setPosition(.59);// grabbing off wall and speciman scoreing
        outputClaw.setPosition(.48);
        intakeClaw.setPosition(.24);//open
        intakeArmLeft.setPosition(.25);//.22
        intakeArmRight.setPosition(.25);
        clawRotater.setPosition(.25);

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(0, 0, 0 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(10, 20, Math.toRadians(335)), Math.PI / 4,
                                new TranslationalVelConstraint(12.0))// Drive too basket for first drop
                        .build(),


                // extendSlide action
                new extendSlide(leftSlide, rightSlide, intakeClaw, clawFlipper, 3050)// 1600,


        ));

        Actions.runBlocking( new SequentialAction(
                drive.actionBuilder(new Pose2d(10, 20, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(7,23), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build(),
        new scoreTheSample(clawFlipper, outputClaw) // score the sample in the basket for the first time
        ));


        sleep(100);

        //SECOND---------------------------------------------------------------------------------------

        Actions.runBlocking( new SequentialAction(
                drive.actionBuilder(new Pose2d(7, 23, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(10,20), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build()

        ));


        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(7, 20, 335 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(17.5, 16.5, Math.toRadians(356)), Math.PI / 4,
                                new TranslationalVelConstraint(8.0))
                        .build(),


                // extendSlide action

                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),
                new resetServos(clawFlipper, outputClaw),
            new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight, 1050)

        ));
        sleep(200);



        Actions.runBlocking( new SequentialAction(
                new resetServos(clawFlipper, outputClaw),
                new pickUpSample(intakeClaw,intakeArmLeft, intakeArmRight),
                new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater)

        ));

        sleep(1100);
        Actions.runBlocking( new SequentialAction(


                new transfer(clawFlipper, outputClaw, intakeClaw)
        ));

        sleep(200);

        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(18, 16, 356 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(12, 23, Math.toRadians(335)), Math.PI / 4,
                                new TranslationalVelConstraint(10.0))
                        .build(),


                // extendSlide action
                new extendSlide(leftSlide, rightSlide, intakeClaw, clawFlipper, 3050)// 1600


        ));


        Actions.runBlocking( new SequentialAction(

                new flipTheClaw(clawFlipper, outputClaw)


        ));
        sleep(200);

        Actions.runBlocking( new SequentialAction(


                drive.actionBuilder(new Pose2d(12, 23, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(7,23), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build()

        ));
        sleep(50);


        Actions.runBlocking( new SequentialAction(

                new scoreTheSample(clawFlipper, outputClaw)

        ));
        sleep(100);




        Actions.runBlocking( new SequentialAction(
                drive.actionBuilder(new Pose2d(7, 23, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(12,20), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build()

        ));

//THIRD----------------------------------------------------------------------------------------------


        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(12, 20, 335 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(18, 26.75, Math.toRadians(357)), Math.PI / 4,
                                new TranslationalVelConstraint(6.0))
                        .build(),


                // extendSlide action

                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),
                new resetServos(clawFlipper, outputClaw),
                new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight,  1050)

        ));
        sleep(100);

        Actions.runBlocking( new SequentialAction(
                new resetServos(clawFlipper, outputClaw),
                new pickUpSample(intakeClaw,intakeArmLeft, intakeArmRight),
                new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater)

        ));
        sleep(1000);

        Actions.runBlocking( new SequentialAction(


                new transfer(clawFlipper, outputClaw, intakeClaw)
        ));

        sleep(100);

        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(18, 24, 357 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(17, 19, Math.toRadians(335)), Math.PI / 4,
                                new TranslationalVelConstraint(10.0))
                        .build(),


                // extendSlide action
                new extendSlide(leftSlide, rightSlide, intakeClaw, clawFlipper, 3050)// 1600


        ));


        Actions.runBlocking( new SequentialAction(

                new flipTheClaw(clawFlipper, outputClaw)


        ));

        sleep(200);
        Actions.runBlocking( new SequentialAction(


                drive.actionBuilder(new Pose2d(17, 19, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(7,23), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build()

        ));
        sleep(50);


        Actions.runBlocking( new SequentialAction(

                new scoreTheSample(clawFlipper, outputClaw)

        ));
        sleep(100);


        //LAST ONE-------------------------------------------------------------------------------------


        Actions.runBlocking( new SequentialAction(
                drive.actionBuilder(new Pose2d(7, 23, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(12,20), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build()

        ));

        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(12, 20, 335 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(37, 9, Math.toRadians(80)), Math.PI / 4,
                                new TranslationalVelConstraint(30))  // Y ====== 14.2
                        .waitSeconds(.1)

                        .splineToLinearHeading( new Pose2d(37, 15.5, Math.toRadians(90)), Math.PI / 4,
                                new TranslationalVelConstraint(10))



                        .build(),


                // extendSlide action

                new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw ),
                new resetServos(clawFlipper, outputClaw),
                new extendIntakeSlide(leftIntakeSlide, rightIntakeSlide, intakeArmLeft, intakeArmRight,  1150)

        ));






        Actions.runBlocking( new SequentialAction(

                new rotateIntakeClaw(clawRotater, outputClaw)

        ));

        sleep(300);

        Actions.runBlocking( new SequentialAction(
                new resetServos(clawFlipper, outputClaw),
                new pickUpSample(intakeClaw,intakeArmLeft, intakeArmRight),
                new intakeSlideIn( leftIntakeSlide, rightIntakeSlide, leftIntakeLimitSwitch, outputClaw, clawRotater)

        ));
        sleep(1100);

        Actions.runBlocking( new SequentialAction(


                new transfer(clawFlipper, outputClaw, intakeClaw)
        ));






        Actions.runBlocking(new ParallelAction(
                // SplineTo action
                drive.actionBuilder(new Pose2d(37, 15.5, 90 * Math.PI / 4))
                        .splineToLinearHeading( new Pose2d(12, 23, Math.toRadians(335)), Math.PI / 4,
                                new TranslationalVelConstraint(30.0))
                        .build(),


                // extendSlide action
                new extendSlide(leftSlide, rightSlide, intakeClaw, clawFlipper, 3050)// 1600


        ));


        Actions.runBlocking( new SequentialAction(

                new flipTheClaw(clawFlipper, outputClaw)


        ));

        sleep(150);
        Actions.runBlocking( new SequentialAction(


                drive.actionBuilder(new Pose2d(12, 23, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(7,23), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build()

        ));
        sleep(50);


        Actions.runBlocking( new SequentialAction(

                new scoreTheSample(clawFlipper, outputClaw)

        ));
        sleep(90);

        Actions.runBlocking( new SequentialAction(
                drive.actionBuilder(new Pose2d(7, 23, 335 * Math.PI / 4))


                        .splineToConstantHeading(new Vector2d(12,20), Math.PI / 4, new TranslationalVelConstraint(20.0))


                        .build(),
        new slideDown(leftSlide, rightSlide, upDownLimitSwitch, clawFlipper, outputClaw)
        ));




        sleep(3000);



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
            sleep(200);
            intakeClaw.setPosition(.45);
            sleep(200);
            intakeArmLeft.setPosition(.515);
            intakeArmRight.setPosition(.515);


            return false;
        }
    }


    public class extendSlide implements Action {
        DcMotor leftSlide;
        DcMotor rightSlide;
        Servo intakeClaw;
        Servo clawFlipper;
        double tPositionUpDown = (double) (targetInchesUpDown * TICKS_PER_INCH);

        public extendSlide(DcMotor L, DcMotor R, Servo I, Servo F, double p) {
            this.leftSlide = L;
            this.rightSlide = R;
            this.tPositionUpDown = p;
            this.intakeClaw = I;
            this.clawFlipper = F;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeClaw.setPosition(.24);

            rightSlide.setDirection(DcMotor.Direction.FORWARD);
            leftSlide.setDirection(DcMotor.Direction.REVERSE);

            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setTargetPosition((int) tPositionUpDown);
            rightSlide.setTargetPosition((int) tPositionUpDown);

            leftSlide.setPower(0.8);
            rightSlide.setPower(0.8);

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
        TouchSensor leftIntakeLimitSwitch;




        boolean isMovingIntakeSlides = true;

        public intakeSlideIn(DcMotor L, DcMotor R, TouchSensor T, Servo O, Servo CR) {

            this.leftIntakeSlide = L;
            this.rightIntakeSlide = R;
            this.leftIntakeLimitSwitch = T;
            this.outputClaw = O;
            this.clawRotater = CR;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawRotater.setPosition(.25);
            if (isMovingIntakeSlides) {
                if (!leftIntakeLimitSwitch.isPressed()) {
                    rightIntakeSlide.setDirection(DcMotor.Direction.FORWARD);
                    leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);
                    rightIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftIntakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    leftIntakeSlide.setPower(-0.7);
                    rightIntakeSlide.setPower(-0.7);
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
            outputClaw.setPosition(.48);

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

                    isMovingSlides = false; // Stop further motor commands
                }
            }

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

            outputClaw.setPosition(.48);//close
            sleep(100);
            intakeClaw.setPosition(.24);//open



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

            clawFlipper.setPosition(0);
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

            clawFlipper.setPosition(.59);



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

            clawRotater.setPosition(.75);



            return false;
        }
    }



}
