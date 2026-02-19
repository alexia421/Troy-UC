package org.firstinspires.ftc.teamcode.Mechanisms;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Servos {
    private final Servo servoBlock;
    public double pos = 1.0;
    public static final double SERVO_EXTENDED = 0.78;
    public static final double SERVO_RETRACTED = 1;


    public Servos(HardwareMap hardwareMap) {
        servoBlock = hardwareMap.get(
                Servo.class, "servoBlock");

        servoBlock.setPosition(pos);
    }


    public double getPos() {
        return pos;
    }

    public void setRetracted(){
        servoBlock.setPosition(SERVO_RETRACTED);
    }

    public void setExtended(){
        servoBlock.setPosition(SERVO_EXTENDED);
    }
}

