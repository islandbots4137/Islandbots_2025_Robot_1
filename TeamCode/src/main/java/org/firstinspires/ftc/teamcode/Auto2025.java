package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import androidx.annotation.NonNull;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
// Non-RR imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
@Config
@Autonomous(name = "RightAuto", group = "Autonomous")

public class AutoTest extends LinearOpMode {
    
    public class LinearSlide {
        private DcMotorEx slideExtend;
        private DcMotorEx slideRotate;
        private  Servo clawRotate;
        private  Servo grabber;


        public LinearSlide(HardwareMap hardwareMap) {
            slideExtend = hardwareMap.dcMotor.get("slideExtend");
            slideRotate = hardwareMap.dcMotor.get("slideRotate");
            slideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
            slideRotate.setDirection(DcMotorSimple.Direction.REVERSE);
            slideRotate.setTargetPosition(0);
            slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRotate.setPower(1.0);
            slideExtend.setTargetPosition(0);
            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideExtend.setPower(1.0);
            //now, the servos 
            Servo clawRotate = hardwareMap.servo.get("clawRotate");
            Servo grabber = hardwareMap.servo.get("grabber");
            clawRotate.setDirection(Servo.Direction.FORWARD);
            grabber.setDirection(Servo.Direction.FORWARD);
        }


        public Action setSlide(int extendVal, int rotateVal) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    slideExtend.setTargetPosition(extendVal);
                    slideRotate.setTargetPosition(rotateVal);
                    //return true while motors are in action 
                    return(slideExtend.isBusy() || slideRotate.isBusy() );
                }
            };
        }
        public Action setClaw(double grabberVal, double clawRotateVal){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    slideExtend.setTargetPosition(extendVal);
                    slideRotate.setTargetPosition(rotateVal);
                    //return false (action done) right away
                    return false;
                }
            };
        }
    }

    @Override
    public void runOpMode() {
        int startpos = 0;
        int mediumpos = -1300;
        int maxpos = -5000;
        double grabber_open = .65;
        double grabber_close = .2;
        double grabber_up = .5;
        int elementExtendStart = 600;
        int elementExtendEnd = 0;
        double grabber_down = .6;
        int maxSlideExtend = 2370;

        AutoTest startingPosition;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        

        Pose2d initialPose = new Pose2d(-70, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        LinearSlide slide = new LinearSlide(hardwareMap);




        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-38, 0), 0);
                //.strafeTo(new Vector2d(20, 20));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-72, 0), 0);
                //.waitSeconds(0)
                //.splineTo(new Vector2d(0, -45), Math.toRadians(-90))

        TrajectoryActionBuilder tab2a = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-71, -63), Math.toDegrees(90))
                .waitSeconds(.2);
                //.splineTo(new Vector2d(-50, -40), Math.toRadians(-90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .waitSeconds(5);


        Action segment2 = tab1.endTrajectory().fresh()
                .waitSeconds(0)
                .build();
        clawRotate.setPosition(grabber_up);
        grabber.setPosition(grabber_close);

        waitForStart();
        if (isStopRequested()) return;

        Action segment1 = tab1.build(); //going forward to the beam

        Actions.runBlocking(
                new SequentialAction(
                        slide.setClaw(grabber_close, grabber_up),
                        slide.setSlide(1300, -3100),
                        segment1, //go to the submersible, 
                        slide.setSlide(500, -3100), //retract the slide, hanging the specimen
                        segment2, //back away from the submersible, 
                        segment3, //go to pick up the second specimen 

                )
        );

    }
    }