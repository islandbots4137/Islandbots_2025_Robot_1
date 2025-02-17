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


//import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
@Config
@Autonomous(name = "OneHangAuto", group = "Autonomous")

public class leftside extends LinearOpMode {

    public class LinearSlide {
        private DcMotor slideExtend;
        private DcMotor slideRotate;
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
            clawRotate = hardwareMap.servo.get("clawRotate");
            grabber = hardwareMap.servo.get("grabber");
            clawRotate.setDirection(Servo.Direction.FORWARD);
            grabber.setDirection(Servo.Direction.FORWARD);
        }


        public Action setSlide(int extendVal, int rotateVal) {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        slideExtend.setTargetPosition(extendVal);
                        slideRotate.setTargetPosition(rotateVal);
                        slideExtend.setPower(1.0);
                        slideRotate.setPower(1.0);
                        initialized = true;
                    }
                    //return true while motors are in action
                    return(slideExtend.isBusy() || slideRotate.isBusy() );
                }
            };
        }
        public Action setClaw(double grabberVal, double clawRotateVal){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    clawRotate.setPosition(clawRotateVal);
                    grabber.setPosition(grabberVal);
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
        double grabber_close = .1;
        double grabber_up = .4;
        int elementExtendStart = 600;
        int elementExtendEnd = 0;
        double grabber_down = .6;
        int maxSlideExtend = 2370;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        Pose2d initialPose = new Pose2d(-70, 30, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        LinearSlide slide = new LinearSlide(hardwareMap);



        //moving in position to hang specimen
        TrajectoryActionBuilder tab0 = drive.actionBuilder(initialPose)
                .waitSeconds(12)
                .splineToConstantHeading(new Vector2d(-70, 0), 0);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-37, 0), 0);
        //.strafeTo(new Vector2d(20, 20));

        //moving to pick up next specimen from the wall
        TrajectoryActionBuilder  tab2 = tab1.endTrajectory().fresh()
                .waitSeconds(0.1)
                .setTangent(-Math.PI ) //start in the negative direction of y axis
                .splineToConstantHeading(new Vector2d(-65, 0), 0);

        TrajectoryActionBuilder  tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(0.1)
                .setTangent(-Math.PI/2 )
                .splineToLinearHeading(new Pose2d(-65, -65, -Math.PI/2), -Math.PI/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder  tab4 = tab2.endTrajectory().fresh()
                .waitSeconds(0.1)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(-65, -40, -Math.PI/2), -Math.PI/2)
                .waitSeconds(.2)
                .splineToLinearHeading(new Pose2d(-56.5, -65, -Math.PI/2), -Math.PI/2)
                .waitSeconds(.2);

        TrajectoryActionBuilder  tab5 = tab2.endTrajectory().fresh()
                .waitSeconds(0.1)
                .splineTo(new Vector2d(-55, 0), 0)
                .waitSeconds(.1)
                .splineTo(new Vector2d(-54, 0), 0)
                .waitSeconds(.1)
                .splineTo(new Vector2d(-38, 20), 0)
                .setTangent(0);


        //TrajectoryActionBuilder tab6 = tab2.endTrajectory().fresh()
        //        .waitSeconds(.1)
        //        .splineTo(new Vector2d(-70, 0), 0)
        //        .waitSeconds(.1)
        //        .splineTo(new Vector2d(-70, -75), 0);



        TrajectoryActionBuilder  tab6 = tab2.endTrajectory().fresh()
                .waitSeconds(0.1)
                .setTangent(-Math.PI /2 ) //start in the negative direction of y axis
                .splineTo(new Vector2d(-71, -71), 0);


        // now, build trajectories, turning TrajectoryActionBuilder to Action:
        Action move0 = tab0.build();
        Action move1 = tab1.build();
        Action move2 = tab2.build();
        Action move3 = tab3.build();
        Action move4 = tab4.build();
        Action move5 = tab5.build();
        Action move6 = tab6.build();
        //Action move3 = tab3.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        slide.setClaw(grabber_close, grabber_up),
                        slide.setSlide(1200, -3100),
                        move0,
                        move1, //go to the submersible,
                        slide.setClaw(grabber_close, grabber_down),
                        slide.setSlide(200, -2800), //retract the slide, hanging the specimen

                        move6
                        //move3 //go to pick up the second specimen
                )
        );

    }
}
//Completely change into a proper left side auto;w preload a yellow, put on top basket, then get the three on the field in top basket, then park.
//Should end with a 35 pt auto, if accurate could be better than right side(23 pts, plus one in the human player zone).