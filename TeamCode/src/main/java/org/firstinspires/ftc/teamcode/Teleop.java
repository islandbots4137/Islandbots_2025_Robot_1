package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor slideExtend = hardwareMap.dcMotor.get("slideExtend");
        DcMotor slideRotate = hardwareMap.dcMotor.get("slideRotate");
        Servo clawRotate = hardwareMap.servo.get("clawRotate");
        Servo grabber = hardwareMap.servo.get("grabber");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        clawRotate.setDirection(Servo.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);

        slideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        int startpos = 0;
        int mediumpos = -1100;
        int maxpos = -5000;
        double grabber_up = .45;
        double grabber_down = .65;
        double grabber_open = .4;
        double grabber_close = .7;
        int maxSlideExtend = 2370;


        slideRotate.setTargetPosition(startpos);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        slideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        grabber.setPosition(grabber_open);
        clawRotate.setPosition(.5);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get the current position of the motor
            double rotpos = slideRotate.getCurrentPosition();
            double extpos = slideExtend.getCurrentPosition();

            // Show the position of the motor on telemetry
            telemetry.addData("Rotational encoder position", rotpos);
            telemetry.addData("Extension encoder position", extpos);

            telemetry.update();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double a = -gamepad2.right_stick_y;
            //double b = gamepad2.left_stick_y * 1.1;
            //boolean c = gamepad2.square;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double slideExtendPower = (a);
            //double slideRotatePower = (b);
            if (gamepad2.cross) {
                //setToPosition
                slideRotate.setTargetPosition(startpos);
            }
            if (gamepad2.triangle) {
                //setToPosition
                slideRotate.setTargetPosition(maxpos);
            }
            if (gamepad2.square) {
                //setToPosition
                slideRotate.setTargetPosition(mediumpos);
            }
            if (gamepad2.circle) {
                slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideExtend.setTargetPosition(0);
                slideRotate.setTargetPosition(startpos);
                slideExtend.setPower(1.0);  // Move towards target
                slideRotate.setPower(1.0);
                while (slideExtend.isBusy() || slideRotate.isBusy()) {
                    telemetry.addData("Slide Extend is Busy", slideExtend.isBusy());
                    slideExtend.setTargetPosition(0);
                    slideExtend.setPower(1.0);
                    y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    rx = gamepad1.right_stick_x;
                    frontLeftPower = (y + x + rx) / denominator;
                    backLeftPower = (y - x + rx) / denominator;
                    frontRightPower = (y - x - rx) / denominator;
                    backRightPower = (y + x - rx) / denominator;
                    if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                        frontLeftMotor.setPower(frontLeftPower);
                        backLeftMotor.setPower(backLeftPower);
                        frontRightMotor.setPower(frontRightPower);
                        backRightMotor.setPower(backRightPower);
                    } else {
                        frontLeftMotor.setPower(frontLeftPower/2);
                        backLeftMotor.setPower(backLeftPower/2);
                        frontRightMotor.setPower(frontRightPower/2);
                        backRightMotor.setPower(backRightPower/2);
                    }
                    if (gamepad2.right_bumper) {
                        grabber.setPosition(grabber_open);
                    }
                    if (gamepad2.left_bumper) {
                        grabber.setPosition(grabber_close);
                    }
                    if (gamepad1.left_bumper) {
                        clawRotate.setPosition(grabber_up);
                    }
                    if (gamepad1.right_bumper) {
                        clawRotate.setPosition(grabber_down);
                    }
                    }
                slideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }



            if (gamepad2.right_bumper) {
                grabber.setPosition(grabber_open);
            }
            if (gamepad2.left_bumper) {
                grabber.setPosition(grabber_close);
            }
            if (gamepad1.left_bumper) {
                clawRotate.setPosition(grabber_up);
            }
            if (gamepad1.right_bumper) {
                clawRotate.setPosition(grabber_down);
            }

            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            } else {
                frontLeftMotor.setPower(frontLeftPower/2);
                backLeftMotor.setPower(backLeftPower/2);
                frontRightMotor.setPower(frontRightPower/2);
                backRightMotor.setPower(backRightPower/2);
            }

            //telemetry.addData("Slide Extend Power", slideExtendPower);
            //telemetry.update();
            //telemetry.addData("Right Stick Y (Gamepad 2)", gamepad2.right_stick_y);
            //telemetry.update();

//            slideExtend.setPower(slideExtendPower);
            double position_x = slideExtend.getCurrentPosition();


            if (position_x < maxSlideExtend && position_x > 20) {
                slideExtend.setPower(slideExtendPower);
            } else if (slideExtendPower < 0 && position_x >= maxSlideExtend) {
                slideExtend.setPower(slideExtendPower);
            } else if (slideExtendPower > 0 && position_x < maxSlideExtend) {
                slideExtend.setPower(slideExtendPower);
            } else if (slideExtendPower < 0 && position_x > 20) {
                slideExtend.setPower(slideExtendPower);
            } else {
                slideExtend.setPower(0);
            }





//            if (slideExtendPower < 0 && position_x >= 2395);
//                slideExtend.setPower(slideExtendPower);
//            if (slideExtendPower > 0 && position_x >= 2395);
//                slideExtend.setPower(0);
            //slideRotate.setPower(slideRotatePower);
            if (slideRotate.isBusy()) {
                slideRotate.setPower(1);  // Move towards target
            } else {
                slideRotate.setPower(0);  // Stop when target is reached
            }




        }
    }

}
//notes: add more into the while loop, fix variable names, start auto code.