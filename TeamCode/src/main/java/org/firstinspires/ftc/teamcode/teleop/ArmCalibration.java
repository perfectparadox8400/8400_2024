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

    private DcMotor elbow;
    private Servo wrist;
    private Servo hand;

    public DcMotor mainBoom = null;
    public DcMotor jibBoom = null;

    @Override
    public void runOpMode() {
        mainBoom = hardwareMap.get(DcMotor.class, "main_arm");
        jibBoom = hardwareMap.get(DcMotor.class, "jib_arm");

        mainBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mainBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jibBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jibBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jibBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean up_pressed = true;
        boolean down_pressed = true;
        boolean a_pressed = true;
        boolean b_pressed = true;

        double Kp = 0;
        double Kd = 0;
        double m_Kp = 0;
        double m_Kd = 0;

        double angle1;
        double angle2;
        //Power to hold Main Boom when Jib Boom is at 90 Degrees
        double c1 = .01;
        //Power to hold Main Boom when Jib Boom is fully extended
        double c2 = .05;
        //Power to hold Jib Boom when it is fully extended.
        double c3 = .01;

        double c1Tmp;
        double c2Tmp;
        double c3Tmp;

        double Pa = 0;
        double Pb = 0;


        double target = 0;
        double mainBoomTicks = 28 * 100 ; //ticks per revolution
        double jibBoomTicks = 28 * 36;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //ARM
                angle1 = mainBoom.getCurrentPosition() * 360/mainBoomTicks;
                angle2 = jibBoom.getCurrentPosition() * 360/jibBoomTicks;
                angle1 += 20;
                angle2 -= 170;



                c1Tmp = c1;
                c2Tmp = c2;
                c3Tmp = c3;


                Pa = (c1Tmp * cos(toRadians(angle1))) + (c2Tmp*cos(toRadians(angle1 + abs(angle2))));
                Pb = (c3Tmp * cos(toRadians(abs(angle1)+angle2)));

                if (gamepad2.dpad_up) {
                    Pa = -1;
                }
                if (gamepad2.dpad_down) {
                    Pa = 1;
                }
                if (gamepad2.a) {
                    Pb = 1;
                }
                if (gamepad2.y) {
                    Pb = -1;
                }

                mainBoom.setPower(Pa);
                jibBoom.setPower(Pb);
                telemetry.addData("Arm", angle1);
                telemetry.addData("Arm Power", Pa);
                telemetry.addData("Jib Boom", angle2);
                telemetry.addData("Jib Boom Power", Pb);

                telemetry.update();
                packet.put("Kp", Kp);
                packet.put("Kd", Kd);
                packet.put("Current Ticks", mainBoom.getCurrentPosition());
                packet.put("Main Boom Power", Pa);
                packet.put("Target Angle", target);
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}