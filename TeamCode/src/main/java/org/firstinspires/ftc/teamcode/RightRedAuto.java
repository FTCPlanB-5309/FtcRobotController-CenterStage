package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

    @Autonomous(name = "RightRedAuto")
    public class RightRedAuto extends LinearOpMode {
        RobotHardware robot = new RobotHardware();
        Drive drive = new Drive(robot, telemetry, this);
        Strafe strafe = new Strafe(robot, telemetry, this);
        GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
        ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
        Claws claws = new Claws(robot,telemetry,this);

        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);
            robot.auto_init();
            waitForStart();
            drive.forward(77, .25);
            double leftDistance = readSensor.distance(robot.leftDistanceSensor);
            double rightDistance = readSensor.distance(robot.rightDistanceSensor);
            telemetry.addData("leftDistance: ",leftDistance);
            telemetry.addData("rightDistance: ",rightDistance);
            telemetry.update();
            if(leftDistance < robot.PROP_THRESHOLD){
                drive.backward(5,.2);
                gyroTurn.goodEnough(90);
                drive.forward(10,.2);
                claws.LeftClawOpen();
                Thread.sleep(1000);
                drive.backward(92,.4);
            }
            else if(rightDistance < robot.PROP_THRESHOLD){
                drive.backward(10,.2);
                strafe.right(33,.2);
                claws.LeftClawOpen();
                drive.backward(12,.2);
                strafe.right(62,.4);
            }
            else{
                claws.LeftClawOpen();
                Thread.sleep(1000);
                drive.backward(20,.2);
                strafe.right(92,.4);
            }
            Thread.sleep(30000);


        }
    }

