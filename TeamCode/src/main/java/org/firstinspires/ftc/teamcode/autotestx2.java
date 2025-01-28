//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//
//
//
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import androidx.annotation.NonNull;
//// RR-specific imports
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.SequentialAction;
//// Non-RR imports
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Config
//public class autotestestx2 extends LinearOpMode {
//
//    public class slideRotate {
//        private DcMotorEx slideRotate;
//
//        public slideRotate(HardwareMap hardwareMap) {
//            slideRotate = hardwareMap.get(DcMotorEx.class, "slideRotate");
//            slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slideRotate.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public class LiftUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    slideRotate.setPower(1);
//                    initialized = true;
//                }
//
//                double pos = slideRotate.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos > -3100.0) {
//                    return true;
//                } else {
//                    slideRotate.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public Action liftUp() {
//            return new LiftUp();
//        }
//    }
//        public class slideExtend {
//            private DcMotorEx slideExtend;
//
//            public slideExtend(HardwareMap hardwareMap) {
//                slideExtend = hardwareMap.get(DcMotorEx.class, "slideExtend");
//                slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                slideExtend.setDirection(DcMotorSimple.Direction.FORWARD);
//            }
//
//            public class extendHang implements Action {
//                private boolean initialized = false;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
//                        slideExtend.setPower(1);
//                        initialized = true;
//                    }
//
//                    double pos = slideExtend.getCurrentPosition();
//                    packet.put("Hangpos", pos);
//                    if (pos < 1300) {
//                        return true;
//                    } else {
//                        slideExtend.setPower(0);
//                        return false;
//                    }
//                }
//            }
//            public Action extendHang() {
//                return new extendHang();
//            }
//
//        public class extendGrab implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    slideExtend.setPower(-1);
//                    initialized = true;
//                }
//
//                double pos = slideExtend.getCurrentPosition();
//                packet.put("extendPos", pos);
//                if (pos < 600) {
//                    return true;
//                } else {
//                    slideExtend.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action extendGrab(){
//            return new extendGrab();
//        }
//    }
//
//
//
//
//
//
//    public class claw {
//        private Servo claw;
//
//        public claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "grabber");
//        }
//
//        public class CloseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(0.2);
//                return false;
//            }
//        }
//        public Action closeClaw() {
//            return new CloseClaw();
//        }
//
//        public class OpenClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(.65);
//                return false;
//            }
//        }
//        public Action openClaw() {
//            return new OpenClaw();
//        }
//    }
//
//
//
//    public class clawRotate {
//        private Servo clawRotate;
//
//        public clawRotate(HardwareMap hardwareMap) {
//            clawRotate = hardwareMap.get(Servo.class, "clawRotate");
//        }
//
//        public class raiseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                clawRotate.setPosition(0.5);
//                return false;
//            }
//        }
//        public Action raiseClaw() {
//            return new raiseClaw();
//        }
//
//        public class lowerClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                clawRotate.setPosition(.6);
//                return false;
//            }
//        }
//        public Action lowerClaw() {
//            return new lowerClaw();
//        }
//    }
//
//    @Override
//
//
//    public void runOpMode() {
//        int startpos = 0;
//        int mediumpos = -1300;
//        int maxpos = -5000;
//        double grabber_open = .65;
//        double grabber_close = .2;
//        double grabber_up = .5;
//        int elementExtendStart = 600;
//        int elementExtendEnd = 0;
//        double grabber_down = .6;
//        int maxSlideExtend = 2370;
//
//        AutoTest startingPosition;
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        DcMotor slideExtend = hardwareMap.dcMotor.get("slideExtend");
//        DcMotor slideRotate = hardwareMap.dcMotor.get("slideRotate");
//        Servo clawRotate = hardwareMap.servo.get("clawRotate");
//        Servo grabber = hardwareMap.servo.get("grabber");
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        clawRotate.setDirection(Servo.Direction.FORWARD);
//        grabber.setDirection(Servo.Direction.FORWARD);
//        slideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideRotate.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        Pose2d initialPose = new Pose2d(-70, 0, Math.toRadians(0));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .waitSeconds(1)
//                .splineTo(new Vector2d(-38, 0), 0);
//        //.strafeTo(new Vector2d(20, 20));
//        Action tab2 = tab1.endTrajectory().fresh()
//                .strafeTo(new Vector2d(-72, 0))
//                .build();
//        //TrajectoryActionBuilder tab2 = drive.actionBuilder(drive.pose)
//        //        .waitSeconds(1)
//        //        .splineTo(new Vector2d(-72, 0), Math.toRadians(0))
//        //        .setTangent(Math.toRadians(0));
//
//
//        Action tab2a = tab2.endTrajectory().fresh()
//                .waitSeconds(1)
//                .splineTo(new Vector2d(-71, -63), Math.toDegrees(90))
//                .waitSeconds(.2)
//                .build();
//        //TrajectoryActionBuilder tab2a = drive.actionBuilder(drive.pose)
//        //        .waitSeconds(1)
//        //        .splineTo(new Vector2d(-71, -63), Math.toDegrees(90))
//        //        .waitSeconds(.2);
//        //.splineTo(new Vector2d(-50, -40), Math.toRadians(-90));
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(drive.pose)
//                .waitSeconds(1);
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(drive.pose)
//                .waitSeconds(5);
//
//
//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .waitSeconds(0)
//                .build();
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//
//
//
//        slideRotate.setPower(1);
//        clawRotate.setPosition(1);
//        slideExtend.setTargetPosition(500);
//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        grabber.setPosition(grabber_open);
//
//
//        Action trajectoryActionChosen;
//        trajectoryActionChosen = tab1.build();
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        trajectoryActionCloseOut
//                )
//        );
//        grabber.setPosition(grabber_close);
//        clawRotate.setPosition(grabber_up);
//        grabber.setPosition(grabber_close);
//        slideRotate.setPower(1);
//        slideExtend.setPower(1);
//        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideRotate.setTargetPosition(-3100);
//        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideExtend.setTargetPosition(1300);
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        claw.openClaw(),
//                        clawRotate.raiseClaw(),
//
//
//                        tab2,
//
//                        tab2a,
//
//                        trajectoryActionCloseOut
//
//                        )
//        );
//        //slideRotate.setTargetPosition(startpos);
//        slideExtend.setTargetPosition(elementExtendStart);
//        clawRotate.setPosition(.4);
//        slideRotate.setTargetPosition(mediumpos);
//        trajectoryActionChosen = tab2a.build();
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        trajectoryActionCloseOut
//                ));
//        //clawRotate.setPosition(.4);
//        trajectoryActionChosen = tab3.build();
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        trajectoryActionCloseOut
//                ));
//        grabber.setPosition(grabber_close);
//        slideRotate.setTargetPosition(maxpos);
//        trajectoryActionChosen = tab4.build();
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        trajectoryActionCloseOut
//                ));
//
//
//
//    }}