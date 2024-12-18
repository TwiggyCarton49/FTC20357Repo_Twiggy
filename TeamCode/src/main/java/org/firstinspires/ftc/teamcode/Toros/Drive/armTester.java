package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;

@TeleOp(name = "ArmTester")
public class armTester extends LinearOpMode {
    private DcMotor slideLeft, slideRight;
    @Override
    public void runOpMode() throws InterruptedException {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");//left
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");//right
        slideRight.setDirection(DcMotor.Direction.REVERSE);
//        finalStage = hardwareMap.get(DcMotorEx.class, "final");
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                double slideLeftControl = gamepad2.left_stick_y;
                double slideRightControl = gamepad2.right_stick_y;

                slideLeft.setPower(slideLeftControl);
                slideRight.setPower(slideRightControl);


                BatteryClass battery = new BatteryClass(hardwareMap);
                telemetry.addData("Battery", battery.getBatteryPercent());
                telemetry.addData("Slide: Left", slideLeft.getCurrentPosition());
                telemetry.addData("Slide: Right", slideRight.getCurrentPosition());

            }
        }
    }
}
//:3