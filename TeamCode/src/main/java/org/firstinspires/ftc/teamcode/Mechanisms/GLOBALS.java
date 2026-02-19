package org.firstinspires.ftc.teamcode.Mechanisms;
import com.acmerobotics.dashboard.config.Config;

@Config
public class GLOBALS {

    public enum ShootState {
        STOPPED, BOOSTING, READY  }

    //-----------SHOOTER PID-----------------
    public static double KP = 0.009;//0.06
    public static double KI = 0.000003;//0.004
    public static double KD = 0.0008; //0.0009

    public static double KS = 0.0007;//0.00005
    public static double KV = 0.000358;
    public static double KA = 0.0003;

    public static double ALPHA = 0.4;
    public static double MAX_I = 0.4;
    public static double MAX_PWR = 1.0;
    public static double SLEW = 10;

    public static double TARGET_TPS = 1100;
    public final double MIN_VELOCITY = 900;

    public static double FarVelAuto = 1500;
    public static double FarVelAuto2 = 1550;


    public static double CloseVelAuto = 1200;

}
