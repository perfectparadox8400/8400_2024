package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "testing!!!!!!!!!!!!!", group = "Autonomous")
public class testauto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        Action go;
        go = drive.actionBuilder(drive.pose)
                .lineToXConstantHeading(48)
                .turn(Math.toRadians(180))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        go
                )
        );
    }
}