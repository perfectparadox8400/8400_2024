package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

@TeleOp(name = "!! Play Game")
public class PlayGame extends LinearOpMode {
    //ARM
    public DcMotor mainBoom1 = null;
    public DcMotor mainBoom2 = null;
    public DcMotor jibBoom = null;
    public Servo elbow = null;
    public Servo hand = null;

    public Servo launcher = null;

    //WHEELS
    public DcMotor right_front = null;
    public DcMotor left_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;

    @Override
    public void runOpMode() {

        //ARM
        mainBoom1 = hardwareMap.get(DcMotor.class, "arm1");
        mainBoom2 = hardwareMap.get(DcMotor.class, "arm2");
        jibBoom = hardwareMap.get(DcMotor.class, "arm3");
        elbow = hardwareMap.get(Servo.class, "elbow");
        hand = hardwareMap.get(Servo.class, "hand");
        launcher = hardwareMap.get(Servo.class, "launcher");
        mainBoom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainBoom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jibBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainBoom1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainBoom2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jibBoom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks = 282;

        double angle1;
        double angle2;

        double c1 = -.45;
        double c2 = -.2;
        double c3 = .1;

        double c1Tmp;
        double c2Tmp;
        double c3Tmp;

        double Pa = 0;
        double Pb = 0;

        double elbowTarget = 0;
        double handTarget = 1 ;




        double powerMod;

        boolean up_pressed = true;
        boolean down_pressed = true;
        boolean a_pressed = true;
        boolean b_pressed = true;

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
                angle1 = mainBoom1.getCurrentPosition() * 360/ticks;
                angle2 = jibBoom.getCurrentPosition() * 360/ticks;
                angle1 += 18;
                angle2 -= 180;

                if (gamepad2.dpad_left && (elbowTarget < 1)) {
                    if (down_pressed) {
                        elbowTarget += .05;
                        down_pressed = false;
                    }
                } else {
                    down_pressed = true;
                }

                if (gamepad2.dpad_right && (elbowTarget > 0)) {
                    if (up_pressed) {
                        elbowTarget -= .05;
                        up_pressed = false;
                    }
                } else {
                    up_pressed = true;
                }

                elbow.setPosition(elbowTarget);

                if (gamepad2.b && (handTarget < 1)) {
                    if (a_pressed) {
                        handTarget += .05;
                        a_pressed = false;
                    }
                } else {
                    a_pressed = true;
                }

                if (gamepad2.x && (handTarget > 0)) {
                    if (b_pressed) {
                        handTarget -= .05;
                        b_pressed = false;
                    }
                } else {
                    b_pressed = true;
                }

                hand.setPosition(handTarget);

                c1Tmp = c1;
                c2Tmp = c2;
                c3Tmp = c3;

                //Reset Encoders
                if (gamepad2.start) {
                    mainBoom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mainBoom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    jibBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mainBoom1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    mainBoom2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    jibBoom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }

                if (gamepad2.right_stick_button) {
                    launcher.setPosition(1);
                }

                //Kill Main Boom Motors
                if (gamepad2.left_stick_button) {
                    c1Tmp = 0;
                    c2Tmp = 0;
                }
                //Kill Jib Boom Motors
                if (gamepad2.right_stick_button) {
                    c3Tmp = 0;
                }

                Pa = (c1Tmp * cos(toRadians(angle1))) + (c2Tmp*cos(toRadians(angle1 + abs(angle2))));
                Pb = (c3Tmp * cos(toRadians(abs(angle1)+angle2)));
                //Pa = c1+c2;
                //Pb = c3;


                if (gamepad2.left_bumper) {
                    powerMod = .5;
                } else if (gamepad2.right_bumper) {
                    powerMod = 2;
                } else {
                    powerMod = 1;
                }

                if (gamepad2.dpad_up) {
                    Pa += .3;
                    Pa *= powerMod;
                } else if (gamepad2.dpad_down) {
                    Pa -= .3;
                    Pa *= powerMod;
                }
                if (gamepad2.y) {
                    Pb += .3;
                    Pb *= powerMod;
                } else if (gamepad2.a) {
                    Pb -= .3;
                    Pb *= powerMod;
                }

                mainBoom1.setPower(Pa);
                mainBoom2.setPower(Pa);
                jibBoom.setPower(Pb);

                //WHEELS
                power = 2.5;
                if (gamepad1.b){
                    power = 0.0000000000001;
                }
                if (gamepad1.y) {
                    power = 1;
                }

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
                telemetry.addData("Arm Position", mainBoom1.getCurrentPosition());
                telemetry.addData("Main Boom Power", Pa);
                telemetry.addData("Jib Boom Power", Pb);
                telemetry.addData("C 2 Var", c2);
                telemetry.addData("Main Boom Angle", angle1);
                telemetry.addData("Jib Boom Angle", angle2);
                telemetry.addData("AnglesAdded", abs(angle1) + angle2);
                telemetry.addData("Servo Target Position", elbowTarget);
                telemetry.update();
            }
        }
    }
}