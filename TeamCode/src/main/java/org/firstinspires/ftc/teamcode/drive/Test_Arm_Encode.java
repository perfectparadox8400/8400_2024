package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.LoggingUtil;

@TeleOp(name = "Test Arm Encoders")
public class Test_Arm_Encode extends LinearOpMode {
    private DcMotor lowarm1;
    private DcMotor lowarm2;
    private DcMotor middlearm;

    private int lowArm1Value = 0;
    private int lowArm2Value = 0;
    private int middleArmValue = 0;
    private int tLowArm1Value = 0;
    private int tLowArm2Value = 0;
    private int tMiddleArmValue = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private final double INCREMENT_INTERVAL_MILLISECONDS = 1000;

    @Override
    public void runOpMode() {
        lowArm1 = hardwareMap.get(DcMotor.class, "arm1");
        lowArm2 = hardwareMap.get(DcMotor.class, "arm2");
        middleArm = hardwareMap.get(DcMotor.class, "arm3");

        // Set encoder modes and zero power behavior
        lowArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lowArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lowArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lowArm1Value = lowArm1.getCurrentPosition();
        lowArm2Value = lowArm2.getCurrentPosition();
        middleArmValue = middleArm.getCurrentPosition();

        waitForStart();
        if (opModeIsActive()) {
            timer.reset();
            while (opModeIsActive()) {
                middleArmValue = middleArm.getCurrentPosition();
                lowArm1Value = lowArm1.getCurrentPosition();
                lowArm2Value = lowArm2.getCurrentPosition();

                // Middle Arm Controls
                if (timer.seconds() >= INCREMENT_INTERVAL_MILLISECONDS) {
                    if (gamepad2.y || gamepad2.a) {
                        tMiddleArmValue += gamepad2.y ? 1 : 0; // Increment if Y is pressed
                        tMiddleArmValue -= gamepad2.a ? 1 : 0; // Decrement if A is pressed
                        middleArm.setTargetPosition(tMiddleArmValue);
                    }
                    if (gamepad2.dpad_up || gamepad2.dpad_down) {
                        tLowArm1Value += gamepad2.dpad_up ? 1 : 0; // Increment if Up is pressed
                        tLowArm1Value -= gamepad2.dpad_down ? 1 : 0; // Decrement if Down is pressed
                        tLowArm2Value += gamepad2.dpad_up ? 1 : 0; // Increment if Up is pressed
                        tLowArm2Value -= gamepad2.dpad_down ? 1 : 0; // Decrement if Down is pressed
                        lowArm1.setTargetPosition(tLowArm1Value);
                        lowArm2.setTargetPosition(tLowArm2Value);
                    }
                    timer.reset();
                }

                // Stop all motors if button B is pressed
                if (gamepad2.b) {
                    middleArm.setPower(0);
                    lowArm1.setPower(0);
                    lowArm2.setPower(0);
                    middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    middleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lowArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lowArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                telemetry.addData("Low Arm 1", lowArm1Value);
                telemetry.addData("Low Arm 2", lowArm2Value);
                telemetry.addData("Middle Arm 1", middleArmValue);
                telemetry.update();
            }
        }
    }
}