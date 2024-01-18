package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Testing Old")
public class ArmTestOld extends LinearOpMode {
    private DcMotor lowarm1;
    private DcMotor lowarm2;
    private DcMotor middlearm;
    private Servo elbow;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {
        lowarm1 = hardwareMap.get(DcMotor.class, "arm1");
        lowarm2 = hardwareMap.get(DcMotor.class, "arm2");
        middlearm = hardwareMap.get(DcMotor.class, "arm3");
        elbow = hardwareMap.get(Servo.class, "elbow");
        lowarm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowarm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middlearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowarm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowarm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middlearm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double target = 1;
        double error = 0;
        double last_error;
        double d_error;
        double kp = 0;
        double kd = 0;
        double power;
        double mtarget = 0;
        double merror = 0;
        double mlast_error;
        double md_error;
        double mkp = .1;
        double mkd = .1;
        double mpower;

        double pelbow = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                /*
                if (target >= 150)  {
                    target = 150;
                }

                if (target <= -5) {
                    target = -5;
                }
*/
                if (gamepad2.dpad_up) {
                    target += .1;
                }
                if (gamepad2.dpad_down) {
                    target -= .1;
                }

                if (gamepad2.y) {
                    kp += .001;
                } else if (gamepad2.a) {
                    kp -= .001;
                }

                if (gamepad2.x) {
                    kd += 0.001;
                } else if (gamepad2.b) {
                    kd -= 0.001;
                }

                last_error = error;
                mlast_error = merror;

                error = target - lowarm1.getCurrentPosition();
                merror = mtarget - middlearm.getCurrentPosition();

                d_error = error - last_error;
                md_error = merror - mlast_error;



                power = ((kp*error) + (kd*d_error));
                //mpower = ((mkp*merror) + (mkd*md_error));

                lowarm2.setPower(power);
                lowarm1.setPower(power);
                //middlearm.setPower(-mpower);
                //elbow.setPosition(pelbow);

                telemetry.addData("Low Arm 1", lowarm1.getCurrentPosition());
                telemetry.addData("Power", power);
                telemetry.addData("Error", error);
                telemetry.addData("Error Difference", d_error);
                telemetry.addData("Target", target);
                telemetry.addData("kd", kd);
                telemetry.addData("kp", kp);
                telemetry.update();
            }
        }
    }
}