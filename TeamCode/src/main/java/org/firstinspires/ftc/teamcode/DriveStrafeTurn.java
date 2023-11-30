package org.firstinspires.ftc.teamcode;

public class DriveStrafeTurn {
    double drive;
    double strafe;
    double turn;

    public DriveStrafeTurn (double drive, double strafe, double turn) {
        this.drive = drive;
        this.strafe = strafe;
        this.turn = turn;
    }

    public DriveStrafeTurn (DriveStrafeTurn driveStrafeTurn) {
        drive = driveStrafeTurn.drive;
        strafe = driveStrafeTurn.strafe;
        turn = driveStrafeTurn.turn;
    }
}
