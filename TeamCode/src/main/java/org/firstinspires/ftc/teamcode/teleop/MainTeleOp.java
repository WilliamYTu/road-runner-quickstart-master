package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.teleop.ProgrammingBoard4;

@TeleOp()
public class MainTeleOp extends OpMode{
    ProgrammingBoard4 board = new ProgrammingBoard4();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init(){
        board.init(hardwareMap);

    }
    @Override
    public void loop(){
        dashboard.sendTelemetryPacket(packet);
        double speedForward = -gamepad1.left_stick_y / 2.0;
        double turnSpeed = -gamepad1.right_stick_x / 2.0;


        if (gamepad1.b) {
            board.moveArm(1);
        } else if (gamepad1.a) {
            board.moveArm(-1);
        } else {
            board.moveArm(0);
        }


        if (gamepad1.y){
            board.moveIntake(1);
        } else if (gamepad1.x){
            board.moveIntake(-1);
        } else {
            board.moveIntake(0);
        }

        // Intake
        if (gamepad1.dpad_up){
            board.moveIntakeServo(2);
        } else if (gamepad1.dpad_down) {
            board.moveIntakeServo(-2);
        } else {
            board.moveIntakeServo(0);
        }
        // Claw
        if (gamepad1.dpad_right){
            board.moveClawServo(0.5);
        } else{
            board.moveClawServo(1);
        }


        if (gamepad1.left_stick_y >= 0.2 || gamepad1.left_stick_y <= -0.2){
            board.setMotorSpeed(-speedForward);
        } else {
            if (gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0) {
                board.turnRobot(-turnSpeed);
            }
            else {
                board.setMotorSpeed(0);
            }
        }
        telemetry.addData("Intake Motor:", board.getMotorPosition("intakeMotor"));

        telemetry.addData("Arm Motor:", board.getMotorRotations("armMotor"));
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("B button", gamepad1.b);

        packet.put("Left stick x", gamepad1.left_stick_x);
        packet.put("Left stick y", gamepad1.left_stick_y);
        packet.put("Right stick x", gamepad1.right_stick_x);
        packet.put("Right stick y", gamepad1.right_stick_y);
        packet.put("A button", gamepad1.a);
        packet.put("B button", gamepad1.b);

    }

}