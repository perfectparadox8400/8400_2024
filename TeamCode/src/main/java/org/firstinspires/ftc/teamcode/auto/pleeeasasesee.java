package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name="Auto Testing")
public class pleeeasasesee extends LinearOpMode {
    static final double COUNTS_PER_INCH = 0.00075775565;

    static final double COUNTS_PER_360 = 24000;
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.3;

    public DcMotor right_front = null;
    public DcMotor left_front = null;
    public DcMotor left_back = null;
    public DcMotor right_back = null;
    public Encoder par0, par1, perp;

    int offsetVar;

    @Override
    public void runOpMode()  {

        telemetry.setMsTransmissionInterval(50);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //WHEELS
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "left_back")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "right_back")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "center_encoder")));

        par0.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        encoderDrive(DRIVE_SPEED,  -12,  -12, 3000);
        encoderTurn(TURN_SPEED,  -90,  -1, 3000);
        encoderTurn(TURN_SPEED,  90,  1, 5);
        encoderDrive(DRIVE_SPEED,  12,  -12, 3000);
        encoderDrive(DRIVE_SPEED,  12,  12, 3000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderTurn(double speed, double degrees, int direction, long timeoutS) {
        double newTarget;
        double parr0 = par0.getPositionAndVelocity().position;
        double parr1 = par1.getPositionAndVelocity().position;
        double perrp = perp.getPositionAndVelocity().position;
        
        boolean go = true;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            
            // Determine new target position, and pass to motor controller
            degrees *= direction;

            //newTarget =  (COUNTS_PER_360 * -1 * (degrees/360));
            newTarget = (COUNTS_PER_360 * (degrees/360));

            if (direction == -1) {
                offsetVar = -825;
            } else {
                offsetVar = -830;
            }

            while (go) {
                telemetry.addData("Target", "Running at " + newTarget);
                telemetry.addData("Running", "Running? " + go);
                telemetry.update();

                go = ((newTarget - (offsetVar * (degrees/90))) > ((((parr0 - par0.getPositionAndVelocity().position) * -1) + (parr1 - par1.getPositionAndVelocity().position) + (( perrp - perp.getPositionAndVelocity().position) * -1))/3) * direction);
                if (go) {
                    left_back.setPower(speed * direction);
                    left_front.setPower(speed * direction);
                    right_back.setPower(speed * direction * -1);
                    right_front.setPower(speed * direction * -1);
                } else {
                    left_back.setPower(0);
                    left_front.setPower(0);
                    right_back.setPower(0);
                    right_front.setPower(0);
                }
            }


            try {
                Thread.sleep(timeoutS);
            } catch (Exception e) {
            }   // optional pause after each move
        }
    }
        public void encoderDrive(double speed, double leftInches, double rightInches, long timeoutS) {
            double newLeftTarget;
            double newRightTarget;
            boolean goLeft = true;
            boolean goRight = true;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = par0.getPositionAndVelocity().position + (leftInches/COUNTS_PER_INCH);
                newRightTarget = par1.getPositionAndVelocity().position + (rightInches/COUNTS_PER_INCH);

                while (goLeft || goRight) {
                    telemetry.addData("Path", "Running at " + par0.getPositionAndVelocity().position + " " + par1.getPositionAndVelocity().position);
                    telemetry.addData("Target", "Running at " + newLeftTarget + " " +  newRightTarget);
                    telemetry.addData("Running", "Runing? " + goLeft + " " +  goRight);
                    telemetry.update();
                    goLeft = ((newLeftTarget - 2300) > par0.getPositionAndVelocity().position);
                    goRight = ((newRightTarget - 2300) > par1.getPositionAndVelocity().position);

                    if (goLeft) {
                        left_back.setPower(speed);
                        left_front.setPower(speed);
                    } else {
                        left_back.setPower(0);
                        left_front.setPower(0);
                    }
                    if (goRight) {
                        right_back.setPower(speed);
                        right_front.setPower(speed);

                    } else {
                        right_back.setPower(0);
                        right_front.setPower(0);
                    }
                }


                try {
                    Thread.sleep(timeoutS);
                } catch (Exception e){}   // optional pause after each move
            }

        }
}