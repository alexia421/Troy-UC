package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.GLOBALS;
import org.firstinspires.ftc.teamcode.Mechanisms.Motoare;
import org.firstinspires.ftc.teamcode.Mechanisms.Servos;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class AutoActions {

    public void resetPinpoint(LinearOpMode opMode, PinpointDrive drive, Pose2d newPose) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        opMode.sleep(50);

        drive.pinpoint.resetPosAndIMU();
        opMode.sleep(300);

        drive.pose = newPose;
        drive.pinpoint.setPosition(newPose);
    }

    public void shoot3(
            LinearOpMode opMode,
            Motoare motoare,
            double TOLERANTA,
            long RUN_TIME_MS
    ) {

        while (opMode.opModeIsActive()
                && motoare.getShooterTps() < (GLOBALS.TARGET_TPS - TOLERANTA)) {

            motoare.updateShooter();
            opMode.idle();
        }

        if (!opMode.opModeIsActive()) return;

        motoare.IntakeStart();

        long t0 = System.currentTimeMillis();
        while (opMode.opModeIsActive()
                && (System.currentTimeMillis() - t0) < RUN_TIME_MS) {

            motoare.updateShooter();
            opMode.idle();
        }

        motoare.IntakeStop();
    }

    public void shootClose(PinpointDrive drive,
                           LinearOpMode opMode,
                           int n,
                           Motoare motoare,
                           double TOLERANTA,
                           long FEED_TIME_MS,
                           long RECOVERY_TIME_MS) {

        for (int i = 0; i < n; i++) {

            while (opMode.opModeIsActive() &&
                    motoare.getShooterTps() < (GLOBALS.TARGET_TPS - TOLERANTA)) {

                motoare.updateShooter();
            }

            motoare.IntakeStart();

            long t0 = System.currentTimeMillis();
            while (opMode.opModeIsActive() && System.currentTimeMillis() - t0 < FEED_TIME_MS) {
                motoare.updateShooter();
                drive.updatePoseEstimate();
            }

            motoare.IntakeStop();

            long t1 = System.currentTimeMillis();
            while (opMode.opModeIsActive() && System.currentTimeMillis() - t1 < RECOVERY_TIME_MS) {
                motoare.updateShooter();
                drive.updatePoseEstimate();
            }
        }
    }

    }

