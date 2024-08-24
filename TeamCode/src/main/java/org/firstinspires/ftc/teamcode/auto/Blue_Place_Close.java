package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Blue Place Close")
public class Blue_Place_Close extends LinearOpMode {
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

    int location = 0;
//    OpenCvCamera webcam;
//    red.CubeDetectionPipeline pipeline;

    private Servo droneLauncher, elbow1, grabberRight, grabberLeft;
    public DcMotor mainBoom = null;
    public DcMotor jibBoom = null;

    @Override
    public void runOpMode()  {
        telemetry.setMsTransmissionInterval(50);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        //WHEELS
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);

        mainBoom = hardwareMap.get(DcMotor.class, "main_arm");
        jibBoom = hardwareMap.get(DcMotor.class, "jib_arm");
        elbow1 = hardwareMap.get(Servo.class, "servo1");
        grabberRight = hardwareMap.get(Servo.class, "servo3");
        grabberLeft = hardwareMap.get(Servo.class, "servo4");

        mainBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mainBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jibBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jibBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jibBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "left_back")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "right_back")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "center_encoder")));

        par0.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        double Kp = 0;
        double Kd = 0;
        double m_Kp = 0;
        double m_Kd = 0;

        double angle1;
        double angle2;
        //Power to hold Main Boom when Jib Boom is at 90 Degrees
        double c1 = .001;
        //Power to hold Main Boom when Jib Boom is fully extended
        double c2 = .005;
        //Power to hold Jib Boom when it is fully extended.
        //double c3 = .01;

        double c1Tmp;
        double c2Tmp;
        //double c3Tmp;

        double Pa = 0;
        double Pb = 0;

        double holdTorque;

        boolean rightBumperPing = true;
        boolean leftBumperPing = true;
        boolean b_pressed = false;

        double rightTarget = 1;
        double leftTarget = 0;
        grabberRight.setPosition(rightTarget);
        grabberLeft.setPosition(leftTarget);
        elbow1.setPosition(.5);
        double mainBoomTicks = 28 * 125 ; //ticks per revolution
        double jibBoomTicks = 28 * 125;

//        while (!isStarted() && !isStopRequested()) {
//            switch (red.CubeDetectionPipeline.getPosition()) {
//                case "Left":
//                    location = 1;
//                    telemetry.addData("Status", "Good");
//                    break;
//                case "Center":
//                    location = 2;
//                    telemetry.addData("Status", "Good");
//                    break;
//                case "Right":
//                    location = 3;
//                    telemetry.addData("Status", "Good");
//                    break;
//                default:
//                    location = 2;
//                    telemetry.addData("Status", "Unknown!");
//                    break;
//            }
//            telemetry.addData("Going", location);
//            telemetry.update();
//        }
        waitForStart();
        //webcam.stopStreaming();

        encoderDrive(DRIVE_SPEED,  6,  6,1, 500);
        encoderTurn(TURN_SPEED,  -90,  -1, 500);
        encoderDrive(DRIVE_SPEED,  37,  37,1, 500);
        encoderTurn(TURN_SPEED,  90,  1, 500);
        encoderDrive(DRIVE_SPEED,  16,  16,1, 500);
        encoderTurn(TURN_SPEED,  -97,  -1, 500);
        encoderDrive(DRIVE_SPEED,  2,  2,1, 500);
//        encoderTurn(TURN_SPEED,  90,  1, 500);
//        encoderDrive(DRIVE_SPEED,12,  12,1, 500);
//        if (location == 1) {
//            encoderTurn(TURN_SPEED,  -90,  -1, 500);
//        } else if (location == 2) {
//            encoderDrive(DRIVE_SPEED,  4,  4,1, 500);
//            encoderTurn(TURN_SPEED,  -90,  -1, 500);
//        } else if (location == 3) {
//            encoderDrive(DRIVE_SPEED,  8,  8,1, 500);
//            encoderTurn(TURN_SPEED,  -90,  -1, 500);
//        }
        jibBoom.setPower(-0.7);
        sleep(1000);
        jibBoom.setPower(0);
        sleep(500);
        grabberRight.setPosition(0);
        grabberLeft.setPosition(1);
        sleep(1000);
        jibBoom.setPower(0.7);
        sleep(1050);
        jibBoom.setPower(0);
        sleep(100);
        grabberRight.setPosition(1);
        grabberLeft.setPosition(0);
        encoderTurn(TURN_SPEED,  -95,  -1, 500);
        encoderDrive(DRIVE_SPEED,  24,  24,1, 500);
//        encoderTurn(TURN_SPEED,  -90,  -1, 500);
//        if (location == 1) {
//            encoderDrive(DRIVE_SPEED,  6,  6,1, 500);
//        } else if (location == 2) {
//            encoderDrive(DRIVE_SPEED,  9,  9,1, 500);
//        } else if (location == 3) {
//            encoderDrive(DRIVE_SPEED,  12,  12,1, 500);
//        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    //    public void setupCam() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        pipeline = new red.CubeDetectionPipeline();
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//    }
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
                offsetVar = -100;
            } else {
                offsetVar = -100;
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
    public void encoderDrive(double speed, double leftInches, double rightInches, double direction, long timeoutS) {
        double newLeftTarget;
        double newRightTarget;
        boolean goLeft = true;
        boolean goRight = true;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = (par0.getPositionAndVelocity().position + (leftInches/COUNTS_PER_INCH)*direction);
            newRightTarget = (par1.getPositionAndVelocity().position + (rightInches/COUNTS_PER_INCH)*direction);
            while (goLeft || goRight) {
                telemetry.addData("Path", "Running at " + par0.getPositionAndVelocity().position + " " + par1.getPositionAndVelocity().position);
                telemetry.addData("Target", "Running at " + newLeftTarget + " " +  newRightTarget);
                telemetry.addData("Running", "Runing? " + goLeft + " " +  goRight);
                telemetry.update();
                goLeft = ((newLeftTarget - 2300*direction)*direction > (par0.getPositionAndVelocity().position*direction));
                goRight = ((newRightTarget - 2300*direction)*direction > (par1.getPositionAndVelocity().position*direction));
                if (goLeft) {
                    left_back.setPower(speed*direction);
                    left_front.setPower(speed*direction);
                } else {
                    left_back.setPower(0);
                    left_front.setPower(0);
                }
                if (goRight) {
                    right_back.setPower(speed*direction);
                    right_front.setPower(speed*direction);
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