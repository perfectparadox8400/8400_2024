package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Go Forward")
public class Auto_Forward extends LinearOpMode {
    HardwarePushbot         robot   = new HardwarePushbot();
    private final ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 0.70588;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double FEET_PER_METER = 3.28084;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update(); robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoder",  "Starting at %7d :%7d :%7d :%7d", robot.left_front.getCurrentPosition(), robot.right_front.getCurrentPosition(), robot.right_back.getCurrentPosition(), robot.right_back.getCurrentPosition());
        telemetry.update();

        //encoderDrive(DRIVE_SPEED,  -34,  -34, 5.0);
    }
}
