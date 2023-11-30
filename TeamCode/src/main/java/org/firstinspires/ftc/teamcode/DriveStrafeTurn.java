package org.firstinspires.ftc.teamcode;

public class DriveStrafeTurn {

        public double drive;
        public double strafe;
        public double turn;

        public DriveStrafeTurn (double drive, double strafe, double turn){
            this.drive = drive;
            this.strafe = strafe;
            this.turn = turn;
        }

        public DriveStrafeTurn (org.firstinspires.ftc.teamcode.DriveStrafeTurn Intial){
            drive = Intial.drive;
            strafe = Intial.strafe;
            turn = Intial.turn;
        }
}
