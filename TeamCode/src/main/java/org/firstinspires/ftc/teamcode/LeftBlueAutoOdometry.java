package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot: LeftBlueAutoOdometry", group="Robot")

    public class LeftBlueAutoOdometry extends LinearOpMode{
    RobotHardware robot       = new RobotHardware();
    XyhVector pos = new XyhVector(91.44, 210.82, 90);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
//        ElapsedTime timer = new ElapsedTime();
//        robot.pathTimer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()){

        }
    }
}
