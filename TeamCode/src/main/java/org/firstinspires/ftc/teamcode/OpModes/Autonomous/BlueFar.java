package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.AutoActions;
import org.firstinspires.ftc.teamcode.Mechanisms.GLOBALS;
import org.firstinspires.ftc.teamcode.Mechanisms.Motoare;
import org.firstinspires.ftc.teamcode.Mechanisms.Servos;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous
public class BlueFar extends LinearOpMode {

    Thread shooterThread;
    boolean shooterThreadRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Motoare motoare = new Motoare(hardwareMap);
        Servos servos = new Servos(hardwareMap);
        AutoActions autoActions = new AutoActions();

        double TOLERANTA = 0.0;
        long RUN_TIME_MS = 850;

        waitForStart();
        if (isStopRequested()) return;
        GLOBALS.TARGET_TPS = GLOBALS.FarVelAuto;

        motoare.startShooter();
        startShooterThread(motoare);

        //PRELOAD
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(8,0), Math.toRadians(20))
                        .build()
        );

        autoActions.shoot3(this, motoare, TOLERANTA, RUN_TIME_MS);
        motoare.stopShooter();
        stopShooterThread(motoare);

        //SET1 ALIGN
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(28,2),Math.toRadians(91))
                        .build()
        );

        GLOBALS.TARGET_TPS = GLOBALS.FarVelAuto2;

        //SET1 BILE
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
//                        .afterTime(0, () -> {
//                            servos.setExtended();
//                            motoare.IntakeStart();
//                        })
//                        .strafeToLinearHeading(new Vector2d(28,32),Math.toRadians(91))
//                        .afterTime(0.1, () -> {
//                            motoare.IntakeStop();
//                            servos.setRetracted();
//                        })
//                        .build()
//        );
//
        servos.setExtended();
        motoare.IntakeStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(28,32),Math.toRadians(91))
                        .build()
        );

        motoare.IntakeStop();
        servos.setRetracted();
        sleep(100);
        motoare.startShooter();
        startShooterThread(motoare);

        //SET1 SHOOT
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(8,0),Math.toRadians(20))
                        .build()
        );
        sleep(400);
        autoActions.shoot3(this, motoare, TOLERANTA, RUN_TIME_MS);
        motoare.stopShooter();
        stopShooterThread(motoare);

//
//
//        //GATE
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
//                        .strafeToLinearHeading(new Vector2d(-44,-20),Math.toRadians(-44))
//                        .build()
//        );

        sleep(2000);

    }



    private void startShooterThread(Motoare motoare) {
        shooterThreadRunning = true;
        shooterThread = new Thread(() -> {
            while (shooterThreadRunning && opModeIsActive()) {
                motoare.updateShooter();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException ignored) {}
            }
        });
        shooterThread.start();
    }

    private void stopShooterThread(Motoare motoare) {
        shooterThreadRunning = false;
        motoare.stopShooter();
    }
}

