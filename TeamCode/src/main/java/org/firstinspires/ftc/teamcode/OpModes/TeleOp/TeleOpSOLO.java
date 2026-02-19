package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.GLOBALS;
import org.firstinspires.ftc.teamcode.Mechanisms.LED;
import org.firstinspires.ftc.teamcode.Mechanisms.Motoare;
import org.firstinspires.ftc.teamcode.Mechanisms.Servos;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop")

public class TeleOpSOLO extends LinearOpMode {
    enum teleopStates {Intake, Score}
    enum LauncherZone {SMALL, LARGE}

    LauncherZone currentZone = LauncherZone.SMALL;
    boolean squareWasPressed = false;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Motoare motoare = new Motoare(hardwareMap);
        Servos servos = new Servos(hardwareMap);
        LED led = new LED(hardwareMap);
        teleopStates currentState = teleopStates.Intake;

        double TOLERANTA = 0;

        waitForStart();
        while (opModeIsActive()) {


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            boolean squareNow = gamepad1.square;

            if (squareNow && !squareWasPressed) {
                if (currentZone == LauncherZone.SMALL) {
                    currentZone = LauncherZone.LARGE;
                    led.setMov();
                } else {
                    currentZone = LauncherZone.SMALL;
                    led.setVerde();
                }
            }
            squareWasPressed = squareNow;

            switch (currentZone) {
                case SMALL:
                    GLOBALS.TARGET_TPS = 1500;
                    led.setMov();
                    break;
                case LARGE:
                    GLOBALS.TARGET_TPS = 1200;
                    led.setVerde();
                    break;
            }

            switch (currentState){
                case Intake:
                    if(gamepad1.right_trigger !=0){
                        motoare.IntakeStart();
                        servos.setExtended();
                    }else{
                        motoare.IntakeStop();
                        servos.setExtended();
                    }
                    if(gamepad1.left_bumper){
                        motoare.startShooter();
                        servos.setRetracted();
                        sleep(300);
                        currentState = teleopStates.Score;
                    }
                    break;

                case Score:
                    if (gamepad1.left_trigger != 0 &&
                            motoare.getShooterTps() >= (GLOBALS.TARGET_TPS - TOLERANTA)) {

                        motoare.IntakeStart();
                        servos.setRetracted();
                        sleep(200);
                    } else {
                        motoare.IntakeStop();
                    }

                    if(gamepad1.left_bumper ){
                        motoare.stopShooter();
                        servos.setExtended();
                        sleep(300);
                        currentState = teleopStates.Intake;
                    }
                    break;



            }

            if(gamepad1.right_bumper)motoare.IntakeReverse();


            motoare.updateShooter();

            telemetry.addData("Zona shooter", currentZone);
            telemetry.addData("TARGET_TPS", GLOBALS.TARGET_TPS);
            telemetry.addData(" State", currentState);
            telemetry.addData("Shooter1 Velocity (tps)", motoare.getShooterTps());
            telemetry.addData("Shooter2 Velocity (tps)", -motoare.shooter2.getVelocity());
            telemetry.addData("TOLERANTA ", motoare.getShooterTps() >= (GLOBALS.TARGET_TPS - TOLERANTA));
            telemetry.update();

        }
    }
}

