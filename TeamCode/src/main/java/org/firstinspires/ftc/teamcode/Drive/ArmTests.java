package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Testing")
public class ArmTests extends LinearOpMode {
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
        lowarm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowarm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middlearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int target = 0;
        int error = 0;
        int last_error;
        int d_error;
        double kp = .1;
        double kd = .1;
        double power;
        int mtarget = 0;
        int merror = 0;
        int mlast_error;
        int md_error;
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
                    target -= 1;
                } else if (gamepad2.dpad_down) {
                    target += 1;
                }

                if (gamepad2.y) {
                    mtarget += 1;
                } else if (gamepad2.a) {
                    mtarget -= 1;
                }

                if (gamepad2.x) {
                    pelbow += 0.1;
                } else if (gamepad2.b) {
                    pelbow -= 0.1;
                }

                last_error = error;
                mlast_error = merror;

                error = target - lowarm2.getCurrentPosition();
                merror = mtarget - middlearm.getCurrentPosition();

                d_error = error - last_error;
                md_error = merror - mlast_error;

                power = ((kp*error) + (kd*d_error));
                mpower = ((mkp*merror) + (mkd*md_error));

                lowarm2.setPower(power);
                lowarm1.setPower(power);
                middlearm.setPower(-mpower);
                //elbow.setPosition(pelbow);

                telemetry.addData("Low Arm 1", lowarm1.getCurrentPosition());
                telemetry.addData("Low Arm 2", lowarm2.getCurrentPosition());
                telemetry.addData("Middle Arm 1", middlearm.getCurrentPosition());
                telemetry.addData("Power", mpower);
                telemetry.addData("Error", merror);
                telemetry.addData("Error Difference", md_error);
                telemetry.update();
            }
        }
    }
}