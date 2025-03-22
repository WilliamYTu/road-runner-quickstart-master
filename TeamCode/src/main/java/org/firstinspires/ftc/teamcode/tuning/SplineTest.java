package org.firstinspires.ftc.teamcode.tuning;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(12, 12), Math.PI / 2)
                        .splineTo(new Vector2d(0, 24), Math.PI)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();
            //Actions.runBlocking(drive.moveToPoint(10, 20));
            // Actions.runBlocking(() -> Log.d("Test", "True"));
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(10)
                            //.splineTo(new Vector2d(12, 12), Math.PI / 2)
                            //.splineTo(new Vector2d(0, 24), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
