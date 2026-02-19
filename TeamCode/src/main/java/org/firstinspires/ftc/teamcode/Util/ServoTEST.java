package org.firstinspires.ftc.teamcode.Util;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo Test Dashboard", group = "Test")
public class ServoTEST extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "Led");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(ServoTestConfig.servoPos);

            telemetry.addData("Servo Position", ServoTestConfig.servoPos);
            telemetry.update();
        }
    }
}