package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

        double pwrist = 0;
        double phand = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {\
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
                    pwrist = 0;
                } else if (gamepad2.b) {
                    pwrist = 1;
                }

                if (gamepad2.left_bumper) {
                    phand = 0;
                } else if (gamepad2.right_bumper) {
                    phand = 1;
                }

                last_error = error;
                mlast_error = merror;

                error = target - arm1.getCurrentPosition();
                merror = mtarget - elbow.getCurrentPosition();

                d_error = error - last_error;
                md_error = merror - mlast_error;

                power = ((kp*error) + (kd*d_error));
                mpower = ((mkp*merror) + (mkd*md_error));

                arm1.setPower(power);
                arm2.setPower(power);
                elbow.setPower(-mpower);
                wrist.setPosition(pwrist);
                hand.setPosition(phand);

                telemetry.addData("Arm 1", arm1.getCurrentPosition());
                telemetry.addData("Arm 2", arm2.getCurrentPosition());
                telemetry.addData("Elbow", elbow.getCurrentPosition());
                telemetry.addData("Power", mpower);
                telemetry.addData("Error", merror);
                telemetry.addData("Error Difference", md_error);
                telemetry.update();
            }
        }
    }
}