package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Strafe")
public class Strafe extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor right_front = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor left_front = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor left_back = hardwareMap.get(DcMotor.class, "left_back");
        DcMotor right_back = hardwareMap.get(DcMotor.class, "right_back");

        double power;

        // You will have to determine which motor to reverse for your robot.
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                power = 2.5;
                if (gamepad2.b || gamepad1.b){
                    power = 0.0000000000001;
                }
                if (gamepad2.a) {
                    power = 1;
                }
                left_front.setPower(-gamepad1.left_stick_y/power + gamepad1.left_stick_x/power + gamepad1.right_stick_x/power);
                right_front.setPower(-gamepad1.left_stick_y/power - gamepad1.left_stick_x/power - gamepad1.right_stick_x/power);
                left_back.setPower(-gamepad1.left_stick_y/power - gamepad1.left_stick_x/power + gamepad1.right_stick_x/power);
                right_back.setPower(-gamepad1.left_stick_y/power + gamepad1.left_stick_x/power - gamepad1.right_stick_x/power);
                
                telemetry.addData("LF POW", left_front.getPower());
                telemetry.addData("LB POW", left_back.getPower());
                telemetry.addData("RF POW", right_front.getPower());
                telemetry.addData("RB POW", right_back.getPower());
                telemetry.update();
            }
        }
    }
}