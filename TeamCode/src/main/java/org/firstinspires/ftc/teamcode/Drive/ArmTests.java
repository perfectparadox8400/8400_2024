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
    private DcMotor arm3;
    private DcMotor elbow;
    private Servo wrist;
    private Servo hand;

    @Override
    public void runOpMode() {
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        elbow = hardwareMap.get(DcMotor.class, "arm4");
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
        double m_Kp = 0;
        double m_Ki = 0;
        double m_Kd = 0;

        double target = 0;
        double integralSum = 0;
        double lastError = 0;
        double encoderAngle;
        double error;
        double derivative;
        double out;
        double ticks = 282; //ticks per revolution
        double holdPower = 0.25; //power to hold the arm up at 0 degrees
        double m_target = 0;
        double m_integralSum = 0;
        double m_lastError = 0;
        double m_encoderAngle;
        double m_error;
        double m_derivative;
        double m_out;
        double m_holdPower = 0.2; //power to hold the middle arm up at 0 degrees
        double weight = 1;
        
        double wrist_d = 0;
        double hand_d = 0;
        boolean y = true;
        boolean a = true;
        boolean x = true;
        boolean b = true;

        double power = 0.25;

        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
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
                        power += .005;
                        x = false;
                    }
                } else {
                    x = true;
                }
                
                if (gamepad1.b) {
                    if (b){
                        power -= .005;
                        b = false;
                    }
                } else {
                    b = true;
                }

                encoderAngle = arm1.getCurrentPosition() * 360/ticks;
                m_encoderAngle = elbow.getCurrentPosition() * 360/ticks;
                error = target - encoderAngle;
                m_error = m_target - m_encoderAngle;
                derivative = (error - lastError) / timer.seconds();
                m_derivative = (m_error - m_lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                m_integralSum = m_integralSum + (m_error * timer.seconds());

                out = (-1 * holdPower * cos(toRadians(encoderAngle))) + (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                m_out = (-1 * m_holdPower * cos(toRadians(m_encoderAngle))) + (m_Kp * m_error) + (m_Ki * m_integralSum) + (m_Kd * m_derivative);

                lastError = error;
                m_lastError = m_error;
                timer.reset();

                arm1.setPower(-power); //out
                arm2.setPower(-power); //out
                elbow.setPower(0.2); //power
                wrist.setPosition(wrist_d);
                hand.setPosition(hand_d);

                telemetry.addData("Arm", encoderAngle);
                telemetry.addData("Elbow", m_encoderAngle);
                telemetry.addData("Power", out);
                telemetry.update();
                packet.put("Kp", Kp);
                packet.put("Kd", Kd);
                packet.put("Current Angle", encoderAngle);
                packet.put("Power", power);
                packet.put("Target Angle", target);
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}