package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Motoare extends GLOBALS {
    public DcMotorEx shooter1, shooter2, intake;
    private final ElapsedTime timer = new ElapsedTime();
    private ShooterPID pid;
    private ShootState state = ShootState.STOPPED;

    public Motoare(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake   = hardwareMap.get(DcMotorEx.class, "perie");

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageSensor batt = hardwareMap.voltageSensor.iterator().next();
        pid = new ShooterPID(batt);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ------ INTAKE ------
    public void IntakeStart() {
        intake.setPower(1);
    }
    public void IntakeStop() {
        intake.setPower(0);
    }
    public void IntakeReverse() {
        intake.setPower(-1);
    }

    // ------ SHOOTER ------

    public double getShooterTpsRaw() {
        return shooter1.getVelocity();
    }

    public double getShooterTps() {
        return -shooter1.getVelocity();
    }

    public void startShooter() {
        pid.setTargetTps(TARGET_TPS);
        state = ShootState.BOOSTING;
        timer.reset();
    }

    public void stopShooter() {
        pid.setTargetTps(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = ShootState.STOPPED;
    }

    public void updateShooter() {
        if (state == ShootState.STOPPED) {
            pid.setTargetTps(0);
            shooter1.setPower(0);
            shooter2.setPower(0);

            pid.reset();

            return;
        }

        double dt = Math.max(0.001, timer.seconds());
        timer.reset();

        double currentTPS = -shooter1.getVelocity();

        double power = pid.update(currentTPS, dt);

        shooter1.setPower(power);
        shooter2.setPower(power);

        if (state == ShootState.BOOSTING && currentTPS >= MIN_VELOCITY) {
            state = ShootState.READY;
        }
    }

    public ShootState getState() {
        return state;
    }
}