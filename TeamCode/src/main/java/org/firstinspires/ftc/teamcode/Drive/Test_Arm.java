package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.LoggingUtil;

//WE NEED TO PUSH THIS CODE TO THE BOT AUGHGHGHGH
@TeleOp(name = "Test Arm No Encode")
public class Test_Arm extends LinearOpMode {
    private DcMotor lowarm1;
    private DcMotor lowarm2;
    private DcMotor middlearm;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override


    public void runOpMode() {
        lowarm1 = hardwareMap.get(DcMotor.class, "arm1");
        lowarm2 = hardwareMap.get(DcMotor.class, "arm2");
        middlearm = hardwareMap.get(DcMotor.class, "arm3");
        lowarm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowarm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middlearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowarm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowarm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middlearm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    middlearm.setPower(1);
                } else if (gamepad2.a) {
                    middlearm.setPower(-1);
                } else {
                    middlearm.setPower(0);
                }
                if (gamepad2.dpad_up) {
                    lowarm1.setPower(1);
                    lowarm2.setPower(1);
                } else if (gamepad2.dpad_down) {
                    lowarm1.setPower(-1);
                    lowarm2.setPower(-1);
                } else {
                    lowarm1.setPower(0);
                    lowarm2.setPower(0);
                }
//                telemetry.addData("LF POW", left_front.getPower());
//                telemetry.addData("LB POW", left_back.getPower());
//                telemetry.addData("RF POW", right_front.getPower());
//                telemetry.addData("RB POW", right_back.getPower());
//                telemetry.update();
            }
        }
    }
}