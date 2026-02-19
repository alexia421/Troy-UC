package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ShooterPID {

    private double kp, ki, kd, ks, kv, ka;
    private double alpha, maxI, maxPwr, slew;

    private double targetTps;
    private double integral = 0;
    private double lastError = 0;
    private double lastTarget = 0;
    private double lastPower = 0;
    private double velFilt = 0;

    private VoltageSensor battery;

    public ShooterPID(VoltageSensor batt) {
        this.kp = GLOBALS.KP;
        this.ki = GLOBALS.KI;
        this.kd = GLOBALS.KD;
        this.ks = GLOBALS.KS;
        this.kv = GLOBALS.KV;
        this.ka = GLOBALS.KA;

        this.alpha = GLOBALS.ALPHA;
        this.maxI = GLOBALS.MAX_I;
        this.maxPwr = GLOBALS.MAX_PWR;
        this.slew = GLOBALS.SLEW;

        this.targetTps = 0;
        this.battery = batt;
    }

    public void setTargetTps(double tps) { this.targetTps = tps; }

    public double getTargetTps() { return targetTps; }
    public void reset(){
        integral=0;
        lastError=0;
        lastTarget=0;
        lastPower=0;
        velFilt=0;
    }

    public double update(double measuredTps, double dt) {

        velFilt += alpha * (measuredTps - velFilt);

        double error = targetTps - velFilt;
        double accel = (targetTps - lastTarget) / dt;

        double ff = (targetTps != 0 ? Math.signum(targetTps) * ks : 0)
                + kv * targetTps
                + ka * accel;

        integral += error * dt;
        integral = clamp(integral, -maxI, maxI);
        double d = (error - lastError) / dt;
        double output =
                ff + kp * error + ki * integral + kd * d;

        if (battery != null) {
            double volts = battery.getVoltage();
            if (volts > 0.1) output *= (12.0 / volts);
        }

        output = clamp(output, -maxPwr, maxPwr);

        double maxStep = slew * dt;
        output = clamp(output, lastPower - maxStep, lastPower + maxStep);

        lastError = error;
        lastPower = output;
        lastTarget = targetTps;

        return output;
    }

    private double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }
}