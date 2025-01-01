package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;



@TeleOp(name = "ArmTester")
public class armTester extends LinearOpMode {
    private DcMotorEx slideLeft, slideRight, pivot;
    private PIDController controller;
    private final double ticks_in_degrees = 1440 / 180;

    //public static double p = 0, i = 0, d = 0, f = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");//left
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");//right
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");//pivot
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if(opModeIsActive()){
            while (opModeIsActive()){
                double slideControl = gamepad2.right_stick_y;
                double pivotControl = gamepad2.left_stick_y;

                runPivot();
                slideLeft.setPower(slideControl);
                slideRight.setPower(slideControl);


                BatteryClass battery = new BatteryClass(hardwareMap);
                telemetry.addData("Battery", battery.getBatteryPercent());
                telemetry.addData("Left position:", slideLeft.getCurrentPosition());
                telemetry.addData("Left power:", slideLeft.getPower());
                telemetry.addData("Right position", slideRight.getCurrentPosition());
                telemetry.addData("Right Power", slideRight.getPower());
                telemetry.addData("pivot position", pivot.getCurrentPosition());
                telemetry.addData("pivot Power", pivot.getPower());
                telemetry.update();
            }
        }
    }
    private void runPivot() {
        double p = 0.001, i = 0, d = 0, f = 0;

        controller = new PIDController(p,i,d);
        int pivotPosition = pivot.getCurrentPosition();
        double target = 0;

//            if (target > -2100){
//                  f =  0.05;
//                } else if (target < -2100) {
//                    f = -0.05;
//                }
                /*Allows for our robot to hold the position of the arm when passing a certain point by
                multiplying the f value by -1 which allows for the arm to be perfectly stable no matter if it is behind or in front of the robot
                */

        //Useful for later
        double power = 0;




//                //Now the fun begins
        controller.setPID(p, i, d); // sets the terms
        double pid = controller.calculate(pivotPosition, target); /// Remember that very funny equation for PID. Well I told the computer to do my math homework

        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f; // Creates a number to get an angle related to the target and ticks and multiples by our f term
        power = pid + ff; // Gives the power to the motor

        // This below is a fun little statement that when the stick is not equal to 0 and is either below 1 or above -1 will give power to the arm divided by 2
        if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){
            power = gamepad2.left_stick_y * 0.5;
            target = pivotPosition;
        }

        // Arm power
        pivot.setPower(power);

//
//
    }
}



// centers at top, angle = stick.x