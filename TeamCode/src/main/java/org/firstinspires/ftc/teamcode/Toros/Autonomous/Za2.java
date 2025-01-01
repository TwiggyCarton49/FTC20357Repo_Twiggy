package org.firstinspires.ftc.teamcode.Toros.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "Za2")
public class Za2 extends LinearOpMode {

    private DcMotor slideLeft;
    private DcMotor slideRight;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            slideLeft.setPower(0.3);
            arm(1200, 0.5);  // Call arm method to move the arm to position
            //sleep(1000);
            //arm(500, 0.2);


            telemetry.addData("Target Position", "Left Slide" + " = " + slideLeft.getTargetPosition() + " | " + "Right Slide" + " = " + slideRight.getTargetPosition());
            telemetry.addData("Actual Position", "Left Slide" + " = " + slideLeft.getCurrentPosition() + " | " + "Right Slide" + " = " + slideRight.getCurrentPosition());
            telemetry.addData("Slide Power", "Left Slide" + " = " + slideLeft.getPower() + " | " + "Right Slide" + " = " + slideRight.getPower());
            telemetry.update();
        }
    }

    private void initHardware() {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void arm(int targetPos, double speed) {
        int armPos = slideLeft.getCurrentPosition();

        if (armPos < targetPos) {
            slideLeft.setPower(speed);
            slideRight.setPower(speed);
        } else if (armPos > targetPos) {
            slideLeft.setPower(-speed);
            slideRight.setPower(-speed);
        } else {
            slideLeft.setPower(0.01);
            slideRight.setPower(0.01);
        }
    }
}

