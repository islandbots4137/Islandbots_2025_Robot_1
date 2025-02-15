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
@Autonomous(name = "RightAuto2", group = "Autonomous")

public class Auto2025 extends LinearOpMode {

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
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
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


        /* Not needed - it is all defined in MecanumDrive class 
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         */


        Pose2d initialPose = new Pose2d(-70, 0, NORTH);
        /* warning: this expects that motor names in hardware map are:
         * leftFront, leftBack, rightFront, rightBack
         * and that the two odometry pods are plugged into these ports:
         *  parallel: leftBack motor port 
         *  perp = rightBack motor port 
         * Do not rename them!
         */
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        LinearSlide slide = new LinearSlide(hardwareMap);



        //moving in position to hang specimen
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-38, 0), NORTH);
        //.strafeTo(new Vector2d(20, 20));

        //moving back from submersible
        TrajectoryActionBuilder  tab2 = tab1.endTrajectory().fresh()
                .waitSeconds(.1)
                .setTangent(SOUTH) 
                .splineToConstantHeading(new Vector2d(-68, 0), 0);

        //going to side wall for second speciment         
        TrajectoryActionBuilder  tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(.1)
                .setTangent(EAST)
                .splineToSplineHeading(new Pose2d(-68, -40, EAST), EAST)
                .waitSeconds(.2)
                .splineToSplineHeading(new Pose2d(-68, -66, EAST), EAST);

        //going back to submersible 
        TrajectoryActionBuilder  tab4 = tab3.endTrajectory().fresh()
                .waitSeconds(.1)
                .setTangent(WEST)
                .splineToSplineHeading(new Pose2d(-50, 2, NORTH), WEST)
                .waitSeconds(.1);
        //advance to submersible to hang specimen        
        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .waitSeconds(.1)
                .setTangent(NORTH)
                .splineToSplineHeading(new Pose2d(-37, 2, NORTH), NORTH);

        //TrajectoryActionBuilder tab6 = tab2.endTrajectory().fresh()
        //        .waitSeconds(.1)
        //        .splineTo(new Vector2d(-70, 0), 0)
        //        .waitSeconds(.1)
        //        .splineTo(new Vector2d(-70, -75), 0);



        TrajectoryActionBuilder  tab6 = tab5.endTrajectory().fresh()
                .waitSeconds(0.1)
                .setTangent(SOUTH) 
                .splineToSplineHeading(new Pose2d(-44, -40, WEST), EAST)
                .waitSeconds(0.1)
                .setTangent(NORTH)
                .splineToSplineHeading(new Pose2d(-10, -53, WEST), EAST);


        // now, build trajectories, turning TrajectoryActionBuilder to Action:
        Action move1 = tab1.build();
        Action move2 = tab2.build();
        Action move3 = tab3.build();
        Action move4 = tab4.build();
        Action move5 = tab5.build();
        Action move6 = tab6.build();
        

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        slide.setClaw(grabber_close, grabber_up),
                        slide.setSlide(1300, 2820),
                        move1, //go to the submersible,
                        slide.setClaw(grabber_close, grabber_down),
                        slide.setSlide(200, 2820), //retract the slide, hanging the specimen
                        slide.setClaw(grabber_open, grabber_down),
                        move2, //move back to avoid collisions
                        slide.setClaw(grabber_open, grabber_up),
                        new ParallelAction(
                            slide.setSlide(150, 1250), //put slide in position to pick up second
                            move3
                        ), //move to pick up the second specimen
                        slide.setClaw(grabber_close, grabber_up),//pick up specimen
                        slide.setSlide(150, 2800),
                        move4, //moves to submersible again
                        slide.setSlide(1175, 2800),
                        move5, //approach submersible for hanging specimen
                        slide.setClaw(grabber_close, grabber_down),
                        slide.setSlide(50, 2800), //retract the slide, hanging the specimen
                        slide.setClaw(grabber_open,grabber_down),
                        new ParallelAction(                        
                            move6,
                            slide.setSlide(0, 0)
                        )

                )
        );

    }
}