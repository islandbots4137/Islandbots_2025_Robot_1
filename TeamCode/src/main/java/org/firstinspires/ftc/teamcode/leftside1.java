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
@Autonomous(name = "Experimental_basket_Auto", group = "Autonomous")

public class leftside1 extends LinearOpMode {

    public class LinearSlide {
        private DcMotor slideExtend;
        private DcMotor slideRotate;
        private  Servo clawRotate;
        private  Servo grabber;


        public LinearSlide(HardwareMap hardwareMap) {
            slideExtend = hardwareMap.dcMotor.get("slideExtend");
            slideRotate = hardwareMap.dcMotor.get("slideRotate");
            slideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
            slideRotate.setDirection(DcMotorSimple.Direction.FORWARD);
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
        int mediumpos = 1300;
        int maxpos = 5000;
        double grabber_open = .65;
        double grabber_close = .1;
        double grabber_up = .4;
        int elementExtendStart = 600;
        int elementExtendEnd = 0;
        double grabber_down = .6;
        int maxSlideExtend = 2370;
        // headings
        double NORTH = 0;
        double EAST = - Math.PI/2;
        double SOUTH = - Math.PI;
        double WEST = Math.PI/2;

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
                .setTangent(WEST)
                .turn(WEST)

                .splineToSplineHeading(new Pose2d(-80, 66, WEST), WEST);

        TrajectoryActionBuilder tab1 = tab0.endTrajectory().fresh()
                .setTangent(NORTH)
                .splineToSplineHeading(new Pose2d(-40, 49, WEST), NORTH);

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .setTangent(NORTH)
                .turn(EAST)
                .splineToSplineHeading(new Pose2d(-38, 49, NORTH), NORTH)
                .waitSeconds(1)
                .setTangent(EAST)
                .splineToSplineHeading(new Pose2d(-38, 47, NORTH), EAST)
                .waitSeconds(.5);

        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(.5)
                .setTangent(SOUTH)
                .splineToSplineHeading(new Pose2d(-80, 30, WEST), SOUTH)
                .waitSeconds(.1);


        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .setTangent(WEST)
                .splineToSplineHeading(new Pose2d(-80, 65, WEST), WEST)
                .waitSeconds(.5);


        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .setTangent(EAST)
                .splineToSplineHeading(new Pose2d(-35, 57, NORTH ), EAST);

        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                .waitSeconds(.1);

        TrajectoryActionBuilder tab6_5 = tab6.endTrajectory().fresh()
                .waitSeconds(.1);

        TrajectoryActionBuilder tab7 = tab6.endTrajectory().fresh()
                .waitSeconds(.1)
                .turn(SOUTH + EAST/2)
                .splineToConstantHeading(new Vector2d(-73, 67.5), SOUTH + EAST/2);
                //.splineToSplineHeading(new Pose2d(-70, 57, SOUTH), SOUTH + EAST);

        TrajectoryActionBuilder tab8 = tab7.endTrajectory().fresh()
                .waitSeconds(.1)
                .splineToSplineHeading(new Pose2d(-50, 40, WEST), SOUTH)
                .waitSeconds(.1);

        TrajectoryActionBuilder tab9 = tab8.endTrajectory().fresh()
                .waitSeconds(.1);



        // now, build trajectories, turning TrajectoryActionBuilder to Action:
        Action move0 = tab0.build();
        Action move1 = tab1.build();
        Action move2 = tab2.build();
        Action move3 = tab3.build();
        Action move4 = tab4.build();
        Action move5 = tab5.build();
        Action move6 = tab6.build();
        Action move6_5 = tab6_5.build();
        Action move7 = tab7.build();
        Action move8 = tab8.build();
        Action move9 = tab9.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                slide.setClaw(grabber_close, grabber_up),
                                slide.setSlide(2400, 5000),
                                move0
                        ),
                        slide.setClaw(grabber_open, grabber_up),
                        move1,
                        new ParallelAction(
                                slide.setClaw(grabber_open, grabber_up),
                                slide.setSlide(0, 0),
                                move2
                        ),
                        slide.setClaw(grabber_open, grabber_down),
                        slide.setClaw(grabber_close, grabber_down),
                        move3,
                        new ParallelAction(
                                slide.setClaw(grabber_close, grabber_up),
                                slide.setSlide(0, 5000),
                                move4

                        ),
                        slide.setSlide(2400, 5000),
                        slide.setClaw(grabber_open, grabber_up),
                        move5,


                        new ParallelAction(
                            move6,
                            slide.setSlide(0, 0),
                            slide.setClaw(grabber_open, grabber_down)
                                ),
                        slide.setClaw(grabber_close, grabber_down),
                        move6_5,

                        new ParallelAction(
                                move7,
                                slide.setSlide(2400, 5000)
                        ),
                        slide.setClaw(grabber_close, grabber_up),
                        slide.setClaw(grabber_open, grabber_up),
                        move8,
                        slide.setClaw(grabber_open, grabber_down),
                        slide.setSlide(0, 0)



                )
        );

    }
}
//Completely change into a proper left side auto; preload a yellow, put on top basket, then get the three on the field in top basket, then park.
//Should end with a 35 pt auto, if accurate could be better than right side(23 pts, plus one in the human player zone).
