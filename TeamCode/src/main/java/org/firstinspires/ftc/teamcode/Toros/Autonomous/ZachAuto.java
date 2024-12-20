package org.firstinspires.ftc.teamcode.Toros.Autonomous;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@TeleOp(name = "ZachAuto")
//Autonomous for first meet of 2024/25
public class ZachAuto extends LinearOpMode {


    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            slideLeft.setPower(0.3);
            //arm(800, 0.5);

        }
    }

    private void initHardware() {
        //Motors

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");


        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotor.Direction.REVERSE);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    private void arm(int targetPos, double speed) {

        int armPos = slideLeft.getCurrentPosition();

        if (armPos <= targetPos)
            slideLeft.setPower(speed);
        else {
            slideLeft.setPower(0);


            int armPos2 = slideRight.getCurrentPosition();

            if (armPos2 <= targetPos)
                slideRight.setPower(speed);
            else {
                slideRight.setPower(0);

            }
        }
    }
}