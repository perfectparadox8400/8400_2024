package org.firstinspires.ftc.teamcode;
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

@TeleOp(name = "Arm Testing")
public class ArmTests extends LinearOpMode {
    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor arm3;
    private DcMotor elbow;
    private Servo wrist;
    private Servo hand;

    @Override
    public void runOpMode() {
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        elbow = hardwareMap.get(DcMotor.class, "arm3");
        wrist = hardwareMap.get(Servo.class, "elbow");
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
        double Kd = 0;
        double m_Kp = 0;
        double m_Kd = 0;

        double target = 0;
        double lastError = 0;
        double encoderAngle;
        double error = 0;
        double d_error;
        double out;
        double ticks = 282; //ticks per revolution
        double holdPower = 0.25; //power to hold the arm up at 0 degrees
        double extHoldPower = 0.5; //power to hold the arm up when main boom and jib boom are both fully extended at 0 degrees.
        double powerDiff = extHoldPower - holdPower; //Difference between power when arm is in and power when arm is out.
        double m_target = 0;
        double m_lastError = 0;
        double m_encoderAngle;
        double m_error = 0;
        double m_d_error = 0;
        double m_out;
        double m_holdPower = 0.2; //power to hold the middle arm up at 0 degrees
        double weight = 1;
        double weightMod;
        double armHolder;
        double elbowEditedAngle;
        
        double wrist_d = 0;
        double hand_d = 0;
        boolean y = true;
        boolean a = true;
        boolean x = true;
        boolean b = true;
        boolean z = true;
        boolean v = true;

        double power = 0.25;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
/*
                if (gamepad2.dpad_up) {
                    if (z){
                        target -= 0.5;
                        z = false;
                    }
                } else {
                    z = true;
                }

                if (gamepad2.dpad_down) {
                    if (v){
                        target += 0.5;
                        v = false;
                    }
                } else {
                    v = true;
                }
*/
                if (gamepad2.dpad_up) {
                    target -= 0.1;
                } else if (gamepad2.dpad_down) {
                    target += 0.1;
                }

                if (gamepad2.y) {
                    hand_d += 1;
                } else if (gamepad2.a) {
                    hand_d -= 1;
                }

                if (gamepad2.x) {
                    wrist_d = 0;
                } else if (gamepad2.b) {
                    wrist_d = 1;
                }

                if (gamepad1.y) {
                    if (y){
                        Kp += .005;
                        y = false;
                    }
                } else {
                    y = true;
                }

                if (gamepad1.a) {
                    if (a){
                        Kp -= .005;
                        a = false;
                    }
                } else {
                    a = true;
                }

                if (gamepad1.x) {
                    if (x){
                        Kd += .005;
                        x = false;
                    }
                } else {
                    x = true;
                }

                if (gamepad1.b) {
                    if (b){
                        Kd -= .005;
                        b = false;
                    }
                } else {
                    b = true;
                }

                lastError = error;
                m_lastError = m_error;


                encoderAngle = arm1.getCurrentPosition() * 360/ticks;
                m_encoderAngle = elbow.getCurrentPosition() * 360/ticks;

                error = target - encoderAngle;

                d_error = error - lastError;

                //m_error = m_target - m_encoderAngle;

                //m_d_error = m_error - m_lastError;

                m_encoderAngle -= 180;

                elbowEditedAngle = m_encoderAngle - encoderAngle;

                weightMod = -1 * powerDiff * cos(.5 * toRadians(m_encoderAngle));

                  armHolder = -1 * holdPower * cos(toRadians(encoderAngle));

                  if (!((weightMod * armHolder) > 0)) {
                      weightMod *= -1;
                  }

                if (encoderAngle != target) {
                    out = armHolder + weightMod + (Kp * error) + (Kd * d_error);
                } else {
                    out = armHolder + weightMod;                }


                //m_out = (m_holdPower * cos(toRadians(elbowEditedAngle))) + (m_Kp * m_error) + (m_Kd * m_d_error);


                arm1.setPower(out); //out
                arm2.setPower(out); //out
                //elbow.setPower(m_out); //power
                wrist.setPosition(wrist_d);
                hand.setPosition(hand_d);

                telemetry.addData("Arm", arm1.getCurrentPosition());
                //telemetry.addData("Elbow", m_encoderAngle);
                telemetry.addData("Power", out);
                telemetry.update();
                packet.put("Kp", Kp);
                packet.put("Kd", Kd);
                packet.put("Current Ticks", arm1.getCurrentPosition());
                packet.put("Main Boom Power", out);
                //packet.put("Jib Boom Power", m_out);
                //packet.put("edited angle", elbowEditedAngle);
                //packet.put("Jib Boom angle", m_encoderAngle);
                packet.put("Target Angle", target);
                //packet.put("Power modifier", CheckMeOut);
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}