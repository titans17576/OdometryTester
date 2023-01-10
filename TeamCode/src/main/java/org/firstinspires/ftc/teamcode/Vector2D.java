package org.firstinspires.ftc.teamcode;

public class Vector2D {
    private double x;
    private double y;
    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Vector2D(Vector2D other){
        this.x = other.getX();
        this.y = other.getY();
    }

    public void add(Vector2D other) {
        x += other.getX();
        y += other.getY();
    }

    public void rotate(double heading){ //Rotates counterclockwise
        double x1 = x;
        double y1 = y;
        x = Math.cos(heading) * x1 - Math.sin(heading) * y1;
        y = Math.sin(heading) * x1 + Math.cos(heading) * y1;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    @Override
    public String toString() {
        return "Vector2D{" +
                "x=" + x +
                ", y=" + y +
                "}";
    }
}
