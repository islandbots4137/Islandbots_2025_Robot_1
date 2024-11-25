package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        int startpos = 0;
        int testpos = -4000;
        slideRotate.setTargetPosition(startpos);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
            if (gamepad2.square) {
                //setToPosition
                slideRotate.setTargetPosition(0);
            }
            if (gamepad2.triangle) {
                //setToPosition
                slideRotate.setTargetPosition(testpos);
            }
            if (gamepad2.cross) {
                //setToPosition
                slideRotate.setTargetPosition(-750);
            }



            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //telemetry.addData("Slide Extend Power", slideExtendPower);
            //telemetry.update();
            //telemetry.addData("Right Stick Y (Gamepad 2)", gamepad2.right_stick_y);
            //telemetry.update();
            slideExtend.setPower(slideExtendPower);
            //slideRotate.setPower(slideRotatePower);
            if (slideRotate.isBusy()) {
                slideRotate.setPower(0.8);  // Move towards target
            } else {
                slideRotate.setPower(0);  // Stop when target is reached
            }



        }
    }
}
