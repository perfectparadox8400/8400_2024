package org.firstinspires.ftc.teamcode.drive;


import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//DESIRED POWER = (MAX BATTERY VOLTS/CURRENT VOLTS) * GIVEN POWER

@TeleOp(name = "Arm Testing")
public class ArmTests extends LinearOpMode {
    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor elbow;
    private Servo wrist;
    private Servo hand;

    @Override
    public void runOpMode() {
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        elbow = hardwareMap.get(DcMotor.class, "arm3");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hand = hardwareMap.get(Servo.class, "hand");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Kp = 0;
        double Ki = 0;
        double Kd = 0;

        double reference = 0;
        double integralSum = 0;
        double lastError = 0;
        double encoderPosition;
        double error;
        double derivative;
        double out;
//        int target = 0;
//        int error = 0;
//        int last_error;
//        int d_error;
//        double kp = .1;
//        double kd = .1;
//        double power;
//
//        int mtarget = 0;
//        int merror = 0;
//        int mlast_error;
//        int md_error;
//        double mkp = .1;
//        double mkd = .1;
//        double mpower;

        double pwrist = 0;
        double phand = 0;
        ElapsedTime timer = new ElapsedTime();
        boolean y = true;
        boolean a = true;
        boolean x = true;
        boolean b = true;
        double power = 0;
        double w = 1;
        double d = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad2.dpad_up) {
                    reference -= .1;

                } else if (gamepad2.dpad_down) {
                    reference += .1;
                }

                if (gamepad2.y) {
//                    mtarget += 1;
                } else if (gamepad2.a) {
//                    mtarget -= 1;
                }

                if (gamepad2.x) {
                    pwrist = 0;
                } else if (gamepad2.b) {
                    pwrist = 1;
                }

                if (gamepad2.left_bumper) {
                    phand = 0;
                } else if (gamepad2.right_bumper) {
                    phand = 1;
                }

                if (gamepad1.y) {
                    if (y){
                        power += .005;
                        y = false;
                    }
                } else {
                    y = true;
                }
                if (gamepad1.a) {
                    if (a){
                        power -= .005;
                        a = false;
                    }
                } else {
                    a = true;
                }

                if (gamepad1.x) {
                    if (x){
                        w += .005;
                        x = false;
                    }
                } else {
                    x = true;
                }
                if (gamepad1.b) {
                    if (b){
                        w -= .005;
                        b = false;
                    }
                } else {
                    b = true;
                }

                encoderPosition = arm1.getCurrentPosition();

                error = reference - encoderPosition;

                derivative = (error - lastError) / timer.seconds();

                integralSum = integralSum + (error * timer.seconds());

                out = (-1 * 0.2 * cos(toRadians(encoderPosition*360/282))) + (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                lastError = error;

                timer.reset();

//                last_error = error;
//                mlast_error = merror;
//
//                error = target - arm1.getCurrentPosition();
//                merror = mtarget - elbow.getCurrentPosition();
//
//                d_error = error - last_error;
//                md_error = merror - mlast_error;
//
//                power = ((kp*error) + (kd*d_error));
//                mpower = ((mkp*merror) + (mkd*md_error));

                arm1.setPower(out);
                arm2.setPower(out);
                //elbow.setPower(-mpower);
                //arm1.setPower(power);
                //arm2.setPower(power);
                wrist.setPosition(pwrist);
                hand.setPosition(phand);

                telemetry.addData("Arm 1", arm1.getCurrentPosition());
                telemetry.addData("Arm 2", arm2.getCurrentPosition());
                telemetry.addData("Elbow", elbow.getCurrentPosition());
                telemetry.addData("Power", out);
                telemetry.addData("Error", error);
                telemetry.addData("Error Difference", derivative);
                telemetry.update();
                FtcDashboard dashboard = FtcDashboard.getInstance();
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Kp", Kp);
                packet.put("Kd", Kd);
                packet.put("Current Angle", (encoderPosition*360/282);
                packet.put("Power", power);
                packet.put("Target Position", reference);
                dashboard.sendTelemetryPacket(packet);

            }
        }
    }
}