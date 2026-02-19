package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Mechanisms.Servos.SERVO_EXTENDED;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LED {

    Servo Led;
    public static final double MOV = 0.7;
    public static final double VERDE = 0.5;

    public LED(HardwareMap hardwareMap){
        Led = hardwareMap.get(Servo.class, "Led");
    }
    public void setMov(){
        Led.setPosition(MOV);
    }

    public void setVerde(){

        Led.setPosition(VERDE);
    }
}
