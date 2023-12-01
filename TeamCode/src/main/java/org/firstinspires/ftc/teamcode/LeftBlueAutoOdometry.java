package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot: LeftBlueAutoOdometry", group="Robot")

    public class LeftBlueAutoOdometry extends LinearOpMode{
    RobotHardware robot       = new RobotHardware();
    XyhVector pos = new XyhVector(91.44, 210.82, 90);
    DriveOD driveOD = new DriveOD(robot,telemetry);
    ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
//        ElapsedTime timer = new ElapsedTime();
//        robot.pathTimer = new ElapsedTime();
        waitForStart();
        robot.armServo.setPosition(robot.LONG_ARM);
        while (opModeIsActive()){
            //driveOD.foward(71,.5));
            double leftDistance = readSensor.distance(robot.leftDistanceSensor);
            double rightDistance = readSensor.distance(robot.rightDistanceSensor);
            if(leftDistance < robot.PROP_THRESHOLD){
//                drive.backward(5,.2);
//                gyroTurn.goodEnough(90);
//                drive.forward(10,.2);
//                claws.LeftClawOpen();
//                Thread.sleep(1000);
//                drive.backward(92,.4);
            }
            else if(rightDistance < robot.PROP_THRESHOLD){
//                drive.backward(10,.2);
//                strafe.right(33,.2);
//                claws.LeftClawOpen();
//                drive.backward(12,.2);
//                strafe.right(62,.4);
            }
            else{
//                claws.LeftClawOpen();
//                Thread.sleep(1000);
//                drive.backward(20,.2);
//                strafe.right(92,.4);
            }
        }
    }
}
