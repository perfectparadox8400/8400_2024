package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.LoggingUtil;

@TeleOp(name = "Test Arm Encode")
public class Test_Arm_Encode extends LinearOpMode {
    private DcMotor lowarm1;
    private DcMotor lowarm2;
    private DcMotor middlearm;

    public int lowarmm1 = 0;
    public int lowarmm2 = 0;
    public int Marm = 0;
    public int tlowarmm1 = 0;
    public int tlowarmm2 = 0;
    public int tMarm = 0;



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
        lowarm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowarm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middlearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    if (Marm != tMarm) {
                        Marm = middlearm.getCurrentPosition();
                        tMarm = Marm;
                    }
                    Marm ++;
                    middlearm.setTargetPosition(Marm);
                    middlearm.setPower(0.5);
                } else if (gamepad2.a) {
                    if (Marm != tMarm) {
                        Marm = middlearm.getCurrentPosition();
                        tMarm = Marm;
                    }
                    Marm --;
                    middlearm.setTargetPosition(Marm);
                    middlearm.setPower(-0.5);
                } else {
                    middlearm.setTargetPosition(Marm);
                    middlearm.setPower(0.1);
                }
                
                if (gamepad2.dpad_up) {
                    if (lowarmm1 != tlowarmm1) {
                        lowarmm1 = lowarm1.getCurrentPosition();
                        tlowarmm1 = lowarmm1;
                    }
                    lowarmm1 ++;
                    if (lowarmm2 != tlowarmm2) {
                        lowarmm2 = lowarm2.getCurrentPosition();
                        tlowarmm2 = lowarmm2;
                    }
                    lowarmm2 ++;
                    lowarm1.setTargetPosition(lowarmm1);
                    lowarm2.setTargetPosition(lowarmm2);
                    lowarm1.setPower(0.5);
                    lowarm2.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    if (lowarmm1 != tlowarmm1) {
                        lowarmm1 = lowarm1.getCurrentPosition();
                        tlowarmm1 = lowarmm1;
                    }
                    lowarmm1 --;
                    if (lowarmm2 != tlowarmm2) {
                        lowarmm2 = lowarm2.getCurrentPosition();
                        tlowarmm2 = lowarmm2;
                    }
                    lowarmm2 --;
                    lowarm1.setTargetPosition(lowarmm1);
                    lowarm2.setTargetPosition(lowarmm2);
                    lowarm1.setPower(-0.5);
                    lowarm2.setPower(-0.5);
                } else {
                    lowarm1.setTargetPosition(lowarmm1);
                    lowarm2.setTargetPosition(lowarmm2);
                    lowarm1.setPower(0.1);
                    lowarm2.setPower(0.1);
                }

                if (gamepad2.b) {
                    middlearm.setPower(0);
                    lowarm1.setPower(0);
                    lowarm2.setPower(0);
                }

                telemetry.addData("Low Arm 1", lowarm1.getCurrentPosition());
                telemetry.addData("Low Arm 2", lowarm2.getCurrentPosition());
                telemetry.addData("Middle Arm 1", middlearm.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}