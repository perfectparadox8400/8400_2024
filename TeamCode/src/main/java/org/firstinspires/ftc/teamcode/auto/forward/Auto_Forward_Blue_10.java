package org.firstinspires.ftc.teamcode.auto.forward;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Go Forward Blue 10", group = "Old Auto")
public class Auto_Forward_Blue_10 extends LinearOpMode {
    private final ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 0.70588;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.2;
    static final double FEET_PER_METER = 3.28084;

    public DcMotor  left_front   = null;
    public DcMotor  right_front  = null;
    public DcMotor  right_back  = null;
    public DcMotor  left_back  = null;
    public Servo hand = null;

    public DcMotor jibBoom = null;

    @Override
    public void runOpMode() {
        right_front =  hardwareMap.get(DcMotor.class, "right_front");
        left_front =  hardwareMap.get(DcMotor.class, "left_front");
        left_back =  hardwareMap.get(DcMotor.class, "left_back");
        right_back =  hardwareMap.get(DcMotor.class, "right_back");
        jibBoom = hardwareMap.get(DcMotor.class, "arm3");
        hand = hardwareMap.get(Servo.class, "hand");
        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoder",  "Starting at %7d :%7d :%7d :%7d", left_front.getCurrentPosition(), right_front.getCurrentPosition(), right_back.getCurrentPosition(), right_back.getCurrentPosition());
        telemetry.update();

        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        hand.setPosition(1);
        jibBoom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            sleep(1000);
            encoderDrive(DRIVE_SPEED,  24,  24, 250);
            encoderDrive(DRIVE_SPEED,  4,  4, 500);
            encoderDrive(DRIVE_SPEED,  -4,  -4, 500);
            encoderDrive(TURN_SPEED,   -16, 16, 500);
            encoderDrive(DRIVE_SPEED,  85,  85, 500);
            hand.setPosition(0);
            jibBoom.setPower(-0.5);
            sleep(200);
            jibBoom.setPower(0);
        }

    }
    public void encoderDrive(double speed, double leftInches, double rightInches, long timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = left_front.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right_front.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            left_front.setTargetPosition(newLeftTarget);
            right_front.setTargetPosition(newRightTarget);
            left_back.setTargetPosition(newLeftTarget);
            right_back.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left_front.setPower(Math.abs(speed));
            right_front.setPower(Math.abs(speed));
            left_back.setPower(Math.abs(speed));
            right_back.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (left_front.isBusy() || right_front.isBusy() || left_back.isBusy() || right_back.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        left_front.getCurrentPosition(),
                        right_front.getCurrentPosition(),
                        left_back.getCurrentPosition(),
                        right_back.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            left_front.setPower(0);
            right_front.setPower(0);
            left_back.setPower(0);
            right_back.setPower(0);


            // Turn off RUN_TO_POSITION
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(timeoutS);   // optional pause after each move
        }
    }
}
