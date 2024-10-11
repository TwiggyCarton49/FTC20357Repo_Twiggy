package org.firstinspires.ftc.teamcode.Toros.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class BatteryClass {
    private final VoltageSensor voltPrime;

    public BatteryClass(HardwareMap hardwareMap){
        voltPrime = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public double getBatteryPercent(){
        double volts = voltPrime.getVoltage();
        double battery = 0;
        if(volts > 12.00){
            battery = 100;
        } else if (volts <= 12) {
            battery = volts / 12.00 * 100;
        }
        return battery;
    }

}
