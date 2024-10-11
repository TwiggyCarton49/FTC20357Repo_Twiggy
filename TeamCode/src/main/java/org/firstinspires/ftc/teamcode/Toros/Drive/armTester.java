package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;

@TeleOp(name = "ArmTester")
public class armTester extends LinearOpMode {
    private DcMotor armPivot1,armPivot2,elevator,finalStage;
    @Override
    public void runOpMode() throws InterruptedException {
        armPivot1 = hardwareMap.get(DcMotorEx.class, "pivot1");
        armPivot2 = hardwareMap.get(DcMotorEx.class, "pivot2");
        elevator = hardwareMap.get(DcMotorEx.class, "elev");
        finalStage = hardwareMap.get(DcMotorEx.class, "final");

        if(opModeIsActive()){
            while (opModeIsActive()){
                double armY = gamepad2.left_stick_y;
                double armP = gamepad1.left_stick_x;

                BatteryClass battery = new BatteryClass(hardwareMap);
                telemetry.addData("Battery", battery.getBatteryPercent());
            }
        }
    }
}
