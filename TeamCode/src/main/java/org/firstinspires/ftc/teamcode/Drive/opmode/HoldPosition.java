package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

@TeleOp(name = "Hold Arm Position")
public class HoldPosition extends LinearOpMode {
    private DcMotor mainBoom1;
    private DcMotor mainBoom2;
    private DcMotor jibBoom;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {
        mainBoom1 = hardwareMap.get(DcMotor.class, "arm1");
        mainBoom2 = hardwareMap.get(DcMotor.class, "arm2");
        jibBoom = hardwareMap.get(DcMotor.class, "arm3");
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
        double c3 = .2;

        double Pa = 0;
        double Pb = 0;

        boolean up_pressed = true;
        boolean down_pressed = true;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                angle1 = mainBoom1.getCurrentPosition() * 360/ticks;
                angle2 = jibBoom.getCurrentPosition() * 360/ticks;

                //This is for debugging - delete later.
                if (gamepad2.dpad_up && up_pressed) {
                    c2 -= .01;
                    up_pressed = false;
                } else if (!gamepad2.dpad_up){
                    up_pressed = true;
                }

                if (gamepad2.dpad_down && down_pressed) {
                    c2 += .01;
                    down_pressed = false;
                } else if (!gamepad2.dpad_down) {
                    down_pressed = true;
                }



                Pa = (c1 * cos(toRadians(angle1))) + (c2*cos(toRadians(angle1 + angle2)));
                Pb = (c3 * cos(toRadians(angle1+angle2)));
                //Pa = c1+c2;
                //Pb = c3;

                mainBoom1.setPower(Pa);
                mainBoom2.setPower(Pa);

                jibBoom.setPower(Pb);



                telemetry.addData("Arm Position", mainBoom1.getCurrentPosition());
                telemetry.addData("Main Boom Power", Pa);
                telemetry.addData("Jib Boom Power", Pb);
                telemetry.addData("C 2 Var", c2);
                telemetry.addData("Main Boom Angle", angle1);
                telemetry.addData("Jib Boom Angle", angle2);
                telemetry.addData("AnglesAdded", angle1 + angle2);

                telemetry.update();
            }
        }
    }
}