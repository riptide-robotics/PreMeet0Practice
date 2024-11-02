package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "William Finite State Machine")
public class WilliamFSM extends LinearOpMode {
    //All motors


    //All servos


    public states currentState;
    public enum states {
        CLEANUP,
        DRIVE,
        HANG,
        LINKAGE
    }

    public void runOpMode() {
        // INIT of all variables

        waitForStart();

        while(opModeIsActive()) {


            switch(currentState) {
                case CLEANUP:

                    break;
                case DRIVE:

                    break;
                case HANG:

                    break;
                case LINKAGE:

                    break;
            }
        }
    }
}