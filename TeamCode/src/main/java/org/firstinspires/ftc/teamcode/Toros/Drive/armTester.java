package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Toros.Util.ArmClass;
import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;

@TeleOp(name = "ArmTester")
public class armTester extends LinearOpMode {
    private DcMotor joint1, joint2;
    @Override
    public void runOpMode() throws InterruptedException {
        ArmClass arm = new ArmClass(hardwareMap);
//        joint1 = hardwareMap.get(DcMotorEx.class, "joint1");
//        joint2 = hardwareMap.get(DcMotorEx.class, "joint2");
////        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
////        finalStage = hardwareMap.get(DcMotorEx.class, "final");
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
//                double joint1Control = gamepad2.left_stick_y;
//                double joint2Control = gamepad2.right_stick_y;
//
//                joint1.setPower(joint1Control/2);
//                joint2.setPower(joint2Control/2);
                 arm.runSlides();
                 arm.runPivot();


                BatteryClass battery = new BatteryClass(hardwareMap);
                telemetry.addData("Battery", battery.getBatteryPercent());
            }
        }
    }
}
