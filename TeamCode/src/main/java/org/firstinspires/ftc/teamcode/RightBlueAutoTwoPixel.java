package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "RightBlueAutoTwoPixel")

public class RightBlueAutoTwoPixel extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
    Claws claws = new Claws(robot, telemetry, this);
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    PropLocation propLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.auto_init();
        waitForStart();
        robot.armServo.setPosition(robot.LONG_ARM);
        robot.wristServo.setPosition(robot.WRIST_DROP_PIXEL);
        Thread.sleep(100);
        robot.armMotor.setTargetPosition(ARM_PIXEL_DROP);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(-1);
        drive.backward(70, .45);
        drive.backward(20,.25);
        double leftDistance = readSensor.distance(robot.leftDistanceSensor);
        double rightDistance = readSensor.distance(robot.rightDistanceSensor);
        telemetry.addData("leftDistance: ", leftDistance);
        telemetry.addData("rightDistance: ", rightDistance);
        telemetry.update();

        //left - not completed
        if (leftDistance < robot.PROP_THRESHOLD) {
            propLocation = PropLocation.LEFT;
            strafe.left(15, .2);
            drive.backward(5, .2);
            gyroTurn.goodEnough(-90);
            Thread.sleep(500);
            drive.forward(30, .2);
            drive.backward(12, .2);
            gyroTurn.goodEnough(-45);
            claws.LeftClawOpen();
            Thread.sleep(250);
            drive.backward(5, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
        }

        //right (same as RightRedLEFT) not completed
        else if (rightDistance < robot.PROP_THRESHOLD) {
            propLocation = PropLocation.RIGHT;
            strafe.left(37, .2);
            drive.backward(16, .2);
            claws.LeftClawOpen();
            drive.backward(10, .2);
            robot.wristServo.setPosition(robot.GRAB_WRIST);
            robot.armMotor.setTargetPosition(0);
        }

        //middle ROBOT IS STARTING BACKWARDS not completed
        else {
            propLocation = PropLocation.CENTER;
            drive.backward(34,.2);
            claws.LeftClawOpen();
            Thread.sleep(300);
            drive.backward(10, .2);
            gyroTurn.goodEnough(90);
            drive.backward(180,.3);
            robot.wristServo.setPosition(robot.WRIST_SCORE_PIXEL);
            robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
            robot.armServo.setPosition(robot.SHORT_ARM);
            strafe.right(67,.2);
        }

        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);


    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
}