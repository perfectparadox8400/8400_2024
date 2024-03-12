package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Check;

//DESIRED POWER = (MAX BATTERY VOLTS/CURRENT VOLTS) * GIVEN POWER


@TeleOp(name = "Arm Calibration")
public class ArmCalibration extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        mainBoom = hardwareMap.get(DcMotor.class, "main_arm");
        jibBoom = hardwareMap.get(DcMotor.class, "jib_arm");
        elbow1 = hardwareMap.get(Servo.class, "servo1");
        //elbow2 = hardwareMap.get(Servo.class, "servo2");
        grabberRight = hardwareMap.get(Servo.class, "servo3");
        grabberLeft = hardwareMap.get(Servo.class, "servo4");


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

        boolean rightBumperPing = true;
        boolean leftBumperPing = true;

        double rightTarget = 1;
        double leftTarget = 0;
        grabberRight.setPosition(rightTarget);
        grabberLeft.setPosition(leftTarget);
        elbow1.setPosition(.5);
        double mainBoomTicks = 28 * 125 ; //ticks per revolution
        double jibBoomTicks = 28 * 125;


        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
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

                if (rightBumperPing & !gamepad2.right_bumper) {
                    rightBumperPing = false;
                }

                if (gamepad2.right_bumper && rightBumperPressed && !rightBumperPing) {
                    rightTarget = 1;
                    rightBumperPressed = false;
                    rightBumperPing = true;
                }
                if (gamepad2.right_bumper && !rightBumperPressed && !rightBumperPing) {
                    rightTarget = 0;
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
                    leftTarget = 1;
                    leftBumperPressed = true;
                    leftBumperPing = true;
                }

                if (gamepad2.dpad_left) {
                    elbow1.setPosition(elbow1.getPosition() + .005);
                }
                if (gamepad2.dpad_right) {
                    elbow1.setPosition(elbow1.getPosition() - .005);
                }

                grabberRight.setPosition(rightTarget);
                grabberLeft.setPosition(leftTarget);

                c1Tmp = c1;
                c2Tmp = c2;

                Pa = (c1Tmp * cos(toRadians(angle1))) + (c2Tmp*cos(toRadians(angle1 + abs(angle2))));

                //Pb = (c3Tmp * cos(toRadians(abs(angle1)+angle2)));

                if (gamepad2.dpad_up) {
                    Pa = 1;
                } else if (gamepad2.dpad_down) {
                    Pa = -1;
                }

                if (gamepad2.a) {
                    Pb = 1;
                } else if (gamepad2.y) {
                    Pb = -1;
                } else {
                    Pb = 0;
                }

                mainBoom.setPower(Pa);
                jibBoom.setPower(Pb);
                telemetry.addData("Arm", angle1);
                telemetry.addData("Arm Power", Pa);
                telemetry.addData("Jib Boom", angle2);
                telemetry.addData("Jib Boom Power", Pb);
                telemetry.addData("Right Grabber Servo: ", rightTarget);

                telemetry.update();
                packet.put("Kp", Kp);
                packet.put("Kd", Kd);
                packet.put("Current Ticks", mainBoom.getCurrentPosition());
                packet.put("Main Boom Power", Pa);
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}