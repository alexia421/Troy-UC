package org.firstinspires.ftc.teamcode.Math;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Motoare;

@Config
@TeleOp(name = "holdVel")
public class holdVel_TEST extends LinearOpMode {

    public static String  MOTOR_NAME    = "shooter1";
    public static String  MOTOR_NAME2   = "shooter2";

    public static String  VOLTAGE_NAME  = "Control Hub";
    public static String  VOLTAGE_NAME2 = "Expansion Hub";

    public static double  KP = 0.009;
    public static double  KI = 0.000003;
    public static double  KD = 0.0009;

    public static double  KS = 0.0007;
    public static double  KV = 0.0004; // original 0.1 / 0.02
    public static double  KA = 0.0003;

    public static double  ALPHA    = 0.4;   // velocity filter (0..1)
    public static double  SLEW     = 10;    // power slew
    public static double  MAX_I    = 0.4;   // integral clamp
    public static double  MAX_PWR  = 1.0;   // power clamp

    public static double  TARGET_TPS = 2500;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VelocityHoldTask task = new VelocityHoldTask(
                hardwareMap, MOTOR_NAME, MOTOR_NAME2,
                KP, KI, KD,
                KS, KV, KA,
                ALPHA, SLEW, MAX_I, MAX_PWR,
                TARGET_TPS, VOLTAGE_NAME, VOLTAGE_NAME2
        );

        DcMotorEx motor  = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME2);

        VoltageSensor battery = null;
        try {
            battery = hardwareMap.get(VoltageSensor.class, VOLTAGE_NAME);
        } catch (Exception ignored) {}

        ElapsedTime loop = new ElapsedTime();
        ElapsedTime telm = new ElapsedTime();
        double lastT = 0;

        waitForStart();
        loop.reset();
        telm.reset();

        while (opModeIsActive()) {

            // poți schimba target și gains din Dashboard
            task.setTargetTps(TARGET_TPS);
            task.set_pdif_values(KP, KI, KD, KS, KV, KA, SLEW, MAX_I, MAX_PWR);

            double now = loop.seconds();
            double dt  = now - lastT;
            if (dt <= 0) dt = 1e-3;

            task.step();

            // >>> folosim tps-ul "pozitiv" din task <<<
            double vel    = task.getCurrentTps();     // TPS (semn întors ca să fie pozitiv)
            double power  = motor.getPower();
            double error  = TARGET_TPS - vel;
            double volts  = (battery != null) ? battery.getVoltage() : 0.0;

            telemetry.addLine("=== Velocity Hold Tuning ===");
            telemetry.addData("Target (tps)", "%.0f", TARGET_TPS);
            telemetry.addData("Measured (tps)", "%.0f", vel);

            // Motor2: dacă vrei și aici plus când trage, întoarcem semnul
            telemetry.addData("Motor2 (tps)", "%.0f", -motor2.getVelocity());
            telemetry.addData("Motor2 power", "%.0f", motor2.getPower());

            telemetry.addData("Error (tps)", "%.0f", error);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Battery (V)", (battery != null) ? "%.2f" : "N/A", volts);

            telemetry.addLine("=== Gains / Limits ===");
            telemetry.addData("kP / kI / kD", "%.5f / %.5f / %.5f", KP, KI, KD);
            telemetry.addData("kS / kV / kA", "%.3f / %.6f / %.3f", KS, KV, KA);
            telemetry.addData("alpha / slew", "%.2f / %.2f", ALPHA, SLEW);
            telemetry.addData("maxI / maxPwr", "%.2f / %.2f", MAX_I, MAX_PWR);

            telemetry.addLine("=== Loop ===");
            telemetry.addData("dt (ms)", "%.1f", dt * 1000.0);
            telemetry.update();

            telm.reset();
            lastT = now;
        }

        task.stop();
    }
}