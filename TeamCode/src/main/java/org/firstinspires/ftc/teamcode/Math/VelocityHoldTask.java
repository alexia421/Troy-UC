package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityHoldTask {
    private final DcMotorEx motor;
    private final DcMotorEx motor2;
    private final PIDFVelocityController ctrl;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime;

    public static VoltageSensor pickVoltage(HardwareMap hw, String preferred) {
        try {
            if (preferred != null) return hw.get(VoltageSensor.class, preferred);
        } catch (Exception ignored) {}
        return hw.voltageSensor.iterator().hasNext()
                ? hw.voltageSensor.iterator().next()
                : null;
    }

    public static void enableBulk(HardwareMap hw) {
        for (LynxModule m : hw.getAll(LynxModule.class)) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public VelocityHoldTask(HardwareMap hw, String motorName, String motorName2,
                            double kP, double kI, double kD,
                            double kS, double kV, double kA,
                            double alpha, double slewRate, double maxI, double maxPower,
                            double targetTps, String voltageName, String voltageName2) {

        enableBulk(hw);

        motor  = hw.get(DcMotorEx.class, motorName);
        motor2 = hw.get(DcMotorEx.class, motorName2);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // shooter2 inversat mecanic (cum aveai și înainte)
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageSensor batt = pickVoltage(hw, voltageName);

        ctrl = new PIDFVelocityController(kP, kI, kD, kS, kV, kA, alpha, batt);
        ctrl.slewRate = slewRate;
        ctrl.maxI = maxI;
        ctrl.maxPower = maxPower;
        ctrl.setTargetTps(targetTps);
        ctrl.reset();

        timer.reset();
        lastTime = timer.seconds();
    }

    public void set_pdif_values(double kp, double ki, double kd,
                                double ks, double kv, double ka,
                                double slewRate, double maxI, double maxPower) {
        ctrl.kP = kp;
        ctrl.kI = ki;
        ctrl.kD = kd;
        ctrl.kS = ks;
        ctrl.kV = kv;
        ctrl.kA = ka;
        ctrl.slewRate = slewRate;
        ctrl.maxI = maxI;
        ctrl.maxPower = maxPower;
    }

    public void setTargetTps(double tps) { ctrl.setTargetTps(tps); }
    public double getTargetTps()         { return ctrl.getTargetTps(); }

    // viteza brută (așa cum vine din encoder, poate fi negativă)
    public double getCurrentTpsRaw() {
        return motor.getVelocity();
    }
    public double getCurrentTps() {
        return -motor.getVelocity();
    }

    public void step() {
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;

        double measuredTps = getCurrentTps();

        double power = ctrl.update(measuredTps, dt);
        motor.setPower(power);
        motor2.setPower(power);

        lastTime = now;
    }

    public void stop() {
        motor.setPower(0);
        motor2.setPower(0);
    }
}