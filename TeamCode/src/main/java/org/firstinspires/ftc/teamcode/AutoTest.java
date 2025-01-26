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
        DcMotor slideExtend = hardwareMap.dcMotor.get("slideExtend");
        DcMotor slideRotate = hardwareMap.dcMotor.get("slideRotate");
        Servo clawRotate = hardwareMap.servo.get("clawRotate");
        Servo grabber = hardwareMap.servo.get("grabber");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawRotate.setDirection(Servo.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        slideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotate.setDirection(DcMotorSimple.Direction.REVERSE);


        Pose2d initialPose = new Pose2d(-70, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pose2d Pose_2 = new Pose2d(-38, 0, Math.toRadians(0));
        Pose2d Pose_3 = new Pose2d(-72, 0, Math.toRadians(0));
        Pose2d Pose_4 = new Pose2d(-71, -63, Math.toDegrees(90));



        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-38, 0), 0);
                //.strafeTo(new Vector2d(20, 20));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-72, 0), Math.toRadians(0))
                .setTangent(Math.toRadians(0));
                // .turn(Math.toRadians(180))
                //.waitSeconds(0)
                //.splineTo(new Vector2d(0, -45), Math.toRadians(-90))

        TrajectoryActionBuilder tab2a = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .splineTo(new Vector2d(-71, -63), Math.toDegrees(90))
                .waitSeconds(.2);
                //.splineTo(new Vector2d(-50, -40), Math.toRadians(-90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(drive.pose)
                .waitSeconds(1);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(drive.pose)
                .waitSeconds(5);


        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .waitSeconds(0)
                .build();
        clawRotate.setPosition(grabber_up);
        grabber.setPosition(grabber_close);

        waitForStart();

        if (isStopRequested()) return;
        clawRotate.setPosition(grabber_up);
        grabber.setPosition(grabber_close);
        slideRotate.setPower(1);
        slideExtend.setPower(1);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotate.setTargetPosition(-3100);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideExtend.setTargetPosition(1300);


        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
        slideRotate.setPower(1);
        clawRotate.setPosition(1);
        slideExtend.setTargetPosition(500);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        grabber.setPosition(grabber_open);

        trajectoryActionChosen = tab2.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                ));
        //slideRotate.setTargetPosition(startpos);
        slideExtend.setTargetPosition(elementExtendStart);
        clawRotate.setPosition(.4);
        slideRotate.setTargetPosition(mediumpos);
        trajectoryActionChosen = tab2a.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                ));
        //clawRotate.setPosition(.4);
        trajectoryActionChosen = tab3.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                ));
        grabber.setPosition(grabber_close);
        slideRotate.setTargetPosition(maxpos);
        trajectoryActionChosen = tab4.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                ));



    }}