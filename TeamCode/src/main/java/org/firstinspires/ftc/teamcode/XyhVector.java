package org.firstinspires.ftc.teamcode;

public class XyhVector {

    public double x;
    public double y;
    public double h;

    public XyhVector (double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public XyhVector (XyhVector Intial){
        x = Intial.x;
        y = Intial.y;
        h = Intial.h;
    }
}
