package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Strafe")
public class Strafe extends LinearOpMode {

    private DcMotor right_front;
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_back;
    private DcMotor inta1;
    private DcMotor inta2;
    private DcMotor slide;
    private DcMotor slided;
    private Servo hand;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        inta1 = hardwareMap.get(DcMotor.class, "inta1");
        inta2 = hardwareMap.get(DcMotor.class, "inta2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        hand = hardwareMap.get(Servo.class, "hand");
        double power;
        double power2;

        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        inta1.setDirection(DcMotorSimple.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        // Reverse one of the drive motors.
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power
                power = 2.5;
                power2 = 0.4;
                if (gamepad2.b|| gamepad1.b){
                    power = 0.0000000000001;
                }
                if (gamepad2.y) {
                    power = 1;
                }
                left_front.setPower(gamepad1.left_stick_y/power + gamepad1.left_stick_x/power + gamepad1.right_stick_x/power);
                right_front.setPower(gamepad1.left_stick_y/power - gamepad1.left_stick_x/power - gamepad1.right_stick_x/power);
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                left_back.setPower(gamepad1.left_stick_y/power - gamepad1.left_stick_x/power + gamepad1.right_stick_x/power);
                right_back.setPower(gamepad1.left_stick_y/power + gamepad1.left_stick_x/power - gamepad1.right_stick_x/power);
                
                if (gamepad2.dpad_up) {
                    slide.setPower(-1);
                } else if (gamepad2.dpad_down) {
                    slide.setPower(0.8);
                } else if (gamepad2.left_bumper) {
                    slide.setPower(-0.01);
                }else{
                    slide.setPower(0);
                }

                if (gamepad2.x) {
                    hand.setPosition(0.6);
                } else if (gamepad2.a) {
                    hand.setPosition(1);
                }
                // Intake
                if (gamepad1.right_bumper) {
                    inta1.setPower(power2);
                    inta2.setPower(power2);
                } else if (gamepad1.left_bumper) {
                    inta1.setPower(-power2);
                    inta2.setPower(-power2);
                }else{
                    inta1.setPower(0);
                    inta2.setPower(0);
                }
                telemetry.addData("LF POW", left_front.getPower());
                telemetry.addData("LB POW", left_back.getPower());
                telemetry.addData("RF POW", right_front.getPower());
                telemetry.addData("RB POW", right_back.getPower());
                telemetry.update();
            }
        }
    }
}