package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;

@TeleOp(name = "ArmTester")
public class armTester extends LinearOpMode {
    private DcMotor elevator,elevator2;
    @Override
    public void runOpMode() throws InterruptedException {
        elevator = hardwareMap.get(DcMotorEx.class, "elev");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elev2");
//        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
//        finalStage = hardwareMap.get(DcMotorEx.class, "final");
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                double elevatorControl = gamepad2.left_stick_y;
                double arm2 = gamepad2.right_stick_y;

                elevator.setPower(elevatorControl/2);
                elevator2.setPower(arm2/2);


                BatteryClass battery = new BatteryClass(hardwareMap);
                telemetry.addData("Battery", battery.getBatteryPercent());
            }
        }
    }
}
