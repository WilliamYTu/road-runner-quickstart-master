package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Drawing;

import org.firstinspires.ftc.teamcode.TankDrive;
@TeleOp(name = "ForwardPushTest2", group = "test")
public class EncoderTickTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            float leftPosition = leftMotor.getCurrentPosition();
            float rightPosition = rightMotor.getCurrentPosition();
            float totalPosition = (leftPosition * -1) + rightPosition;
            /*
            telemetry.addData("Left Encoder", leftPosition);
            telemetry.addData("Right Encoder", rightPosition);
             */

            telemetry.addData("Encoder:", totalPosition);
            packet.put("Encoder Reading", totalPosition);
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }


    }


}
