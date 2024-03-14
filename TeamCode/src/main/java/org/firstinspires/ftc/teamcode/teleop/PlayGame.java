package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

@TeleOp(name = "! Play Game")
public class PlayGame extends LinearOpMode {
    private Servo droneLauncher;
    //elbow 1
    private Servo elbow1;
    //elbow 2
    private Servo elbow2;
    //grabber right
    private Servo grabberRight;
    //grabber left
    private Servo grabberLeft;

    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;
    public DcMotor mainBoom = null;
    public DcMotor jibBoom = null;

    public Servo launcher = null;

    //WHEELS
    public DcMotor right_front = null;
    public DcMotor left_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;



    @Override
    public void runOpMode() {

        mainBoom = hardwareMap.get(DcMotor.class, "main_arm");
        jibBoom = hardwareMap.get(DcMotor.class, "jib_arm");
        elbow1 = hardwareMap.get(Servo.class, "servo1");
        //elbow2 = hardwareMap.get(Servo.class, "servo2");
        grabberRight = hardwareMap.get(Servo.class, "servo3");
        grabberLeft = hardwareMap.get(Servo.class, "servo4");
        droneLauncher = hardwareMap.get(Servo.class, "drone");


        mainBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mainBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jibBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jibBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jibBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double Kp = 0;
        double Kd = 0;
        double m_Kp = 0;
        double m_Kd = 0;

        double angle1;
        double angle2;
        //Power to hold Main Boom when Jib Boom is at 90 Degrees
        double c1 = .001;
        //Power to hold Main Boom when Jib Boom is fully extended
        double c2 = .005;
        //Power to hold Jib Boom when it is fully extended.
        //double c3 = .01;

        double c1Tmp;
        double c2Tmp;
        //double c3Tmp;

        double Pa = 0;
        double Pb = 0;

        double holdTorque;

        boolean rightBumperPing = true;
        boolean leftBumperPing = true;
        boolean b_pressed = false;

        double rightTarget = 1;
        double leftTarget = 0;
        grabberRight.setPosition(rightTarget);
        grabberLeft.setPosition(leftTarget);
        elbow1.setPosition(.5);
        double mainBoomTicks = 28 * 125 ; //ticks per revolution
        double jibBoomTicks = 28 * 125;


        //WHEELS
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        double power;

        // Reverse one of the drive motors.
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //ARM
                angle1 = mainBoom.getCurrentPosition() * 360/mainBoomTicks;
                angle2 = jibBoom.getCurrentPosition() * 360/jibBoomTicks;
                angle1 += 24;
                angle2 -= 170;

                if (angle1 < 0) {
                    angle1 = 360+angle1;
                }
                if (angle2 < 0) {
                    angle2 = 360 + angle1;
                }

                if (gamepad2.back) {
                    droneLauncher.setPosition(1 );
                }

                if (rightBumperPing & !gamepad2.right_bumper) {
                    rightBumperPing = false;
                }

                if (gamepad2.right_bumper && rightBumperPressed && !rightBumperPing) {
                    rightTarget = 1;
                    rightBumperPressed = false;
                    rightBumperPing = true;
                }
                if (gamepad2.right_bumper && !rightBumperPressed && !rightBumperPing) {
                    rightTarget = .3;
                    rightBumperPressed = true;
                    rightBumperPing = true;
                }

                if (leftBumperPing & !gamepad2.left_bumper) {
                    leftBumperPing = false;
                }
                if (gamepad2.left_bumper && leftBumperPressed && !leftBumperPing) {
                    leftTarget = 0;
                    leftBumperPressed = false;
                    leftBumperPing = true;
                }
                if (gamepad2.left_bumper && !leftBumperPressed && !leftBumperPing) {
                    leftTarget = .35;
                    //leftTarget = 1;
                    leftBumperPressed = true;
                    leftBumperPing = true;
                }

                if (gamepad2.dpad_up) {
                    elbow1.setPosition(elbow1.getPosition() + .005);
                }
                if (gamepad2.dpad_down) {
                    elbow1.setPosition(elbow1.getPosition() - .005);
                }

                if (gamepad2.b) {
                    rightTarget = 0;
                    leftTarget = 1;
                    leftBumperPressed = true;
                    rightBumperPressed = true;
                }



                grabberRight.setPosition(rightTarget);
                grabberLeft.setPosition(leftTarget);

                c1Tmp = c1;
                c2Tmp = c2;

                holdTorque = (c1Tmp * cos(toRadians(angle1))) + (c2Tmp*cos(toRadians(angle1 + abs(angle2))));

                //Pb = (c3Tmp * cos(toRadians(abs(angle1)+angle2)));

                Pa = (-gamepad2.left_stick_y * .5);
                if (Pa < 0) {
                    Pa -= holdTorque;
                } else {
                    Pa += holdTorque;
                }

                Pb = gamepad2.right_stick_y * .7;



                //WHEELS
                power = 2.5;
                if (gamepad1.b){
                    power = 0.0000000000001;
                }
                if (gamepad1.y) {
                    power = 1;
                }
                mainBoom.setPower(Pa);
                jibBoom.setPower(Pb);

                left_front.setPower(-gamepad1.left_stick_y/power + gamepad1.left_stick_x/power + gamepad1.right_stick_x/power);
                right_front.setPower(-gamepad1.left_stick_y/power - gamepad1.left_stick_x/power - gamepad1.right_stick_x/power);
                left_back.setPower(-gamepad1.left_stick_y/power - gamepad1.left_stick_x/power + gamepad1.right_stick_x/power);
                right_back.setPower(-gamepad1.left_stick_y/power + gamepad1.left_stick_x/power - gamepad1.right_stick_x/power);

                //Wheel Telemetry
                telemetry.addData("LF POW", left_front.getPower());
                telemetry.addData("LB POW", left_back.getPower());
                telemetry.addData("RF POW", right_front.getPower());
                telemetry.addData("RB POW", right_back.getPower());

                //Arm Telemetry
                telemetry.addData("Arm Position", mainBoom.getCurrentPosition());
                telemetry.addData("Main Boom Power", Pa);
                telemetry.addData("Jib Boom Power", Pb);
                telemetry.addData("C 2 Var", c2);
                telemetry.addData("Main Boom Angle", angle1);
                telemetry.addData("Jib Boom Angle", angle2);
                telemetry.addData("AnglesAdded", abs(angle1) + angle2);
                telemetry.update();
            }
        }
    }
}