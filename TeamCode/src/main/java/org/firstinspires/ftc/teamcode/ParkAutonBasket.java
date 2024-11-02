package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Park Auton Near Basket")
public class ParkAutonBasket extends LinearOpMode {

    private DcMotorEx flWheel, frWheel, blWheel, brWheel;

    public void runOpMode() throws InterruptedException {


        ElapsedTime time = new ElapsedTime();


        frWheel = hardwareMap.get(DcMotorEx.class, "frWheel");
        flWheel = hardwareMap.get(DcMotorEx.class, "flWheel");
        brWheel = hardwareMap.get(DcMotorEx.class, "brWheel");
        blWheel = hardwareMap.get(DcMotorEx.class, "blWheel");

        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);;
        frWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        waitForStart();

        while(opModeIsActive()) {

            double elapsedTime = time.seconds();
            double acceleration = (flWheel.getVelocity() - flWheel.getVelocity() - blWheel.getVelocity() - brWheel.getVelocity()) / elapsedTime;

            double distance = flWheel.getVelocity()*elapsedTime+(0.5*acceleration*(elapsedTime*elapsedTime));

            forward(10); // idk value but go forward about 2m
            rotateRight(10); // idk value but rotate 90deg
            forward(10); // idk value but go forward about 8m (towards park zone)
            rotateLeft(10); // idk value but rotate to face the park zone
            forward(10); // idk value but go into park zone and stop(about 2m)
        }
    }

    public void forward (double velocity){
        flWheel.setVelocity(velocity);
        frWheel.setVelocity(velocity);
        blWheel.setVelocity(velocity);
        brWheel.setVelocity(velocity);
        sleep(1000);
    }

    public void rotateRight(double velocity){
        flWheel.setVelocity(-velocity);
        frWheel.setVelocity(velocity);
        blWheel.setVelocity(-velocity);
        brWheel.setVelocity(velocity);
        sleep(1000);
    }

    public void rotateLeft(double velocity){
        flWheel.setVelocity(velocity);
        frWheel.setVelocity(-velocity);
        blWheel.setVelocity(velocity);
        brWheel.setVelocity(-velocity);
        sleep(1000);
    }

    public void backward(double velocity){
        flWheel.setVelocity(-velocity);
        frWheel.setVelocity(-velocity);
        blWheel.setVelocity(-velocity);
        brWheel.setVelocity(-velocity);
        sleep(1000);
    }
}
