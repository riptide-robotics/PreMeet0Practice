package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TEST MOVE")
public class WilliamsCodeMoveBot extends LinearOpMode {
    DcMotor flWheelMotor;
    DcMotor frWheelMotor;
    DcMotor brWheelMotor;
    DcMotor blWheelMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        double y;
        double x;
        double rx;

        //Init Motors
        flWheelMotor = hardwareMap.dcMotor.get("flWheel");
        frWheelMotor = hardwareMap.dcMotor.get("frWheel");
        brWheelMotor = hardwareMap.dcMotor.get("brWheel");
        blWheelMotor = hardwareMap.dcMotor.get("blWheel");

        waitForStart();

        //Make all motors go in the same direction with same values imputed for power
        //Reverses the direction of the left motors
        flWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //While loop for when controlling robot
        while(opModeIsActive()) {

            //controller values inputed
            y = -gamepad1.left_stick_y;
            //for margin of error in movement
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            //scale values so the robot feels better to drive for our drivers <3
            double scale = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx), 1);

            //power set for each of our motos
            double flpower = (y-x+rx)/scale;
            double frpower = (y+x-rx)/scale;
            double blpower = (y+x+rx)/scale;
            double brpower = (y-x-rx)/scale;

            //setting the power of each of our motos
            flWheelMotor.setPower(flpower);
            frWheelMotor.setPower(frpower);
            brWheelMotor.setPower(blpower);
            blWheelMotor.setPower(brpower);
        }
    }
}
