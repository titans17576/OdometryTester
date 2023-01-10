package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Odometry  {
    public DcMotorEx lEncoder;
    public DcMotorEx rEncoder;
    public DcMotorEx bEncoder;
    public Vector2D position;
    public double heading = 0;
    double startHeading = 0;

    public int lEncoderDirection = 1;
    public int rEncoderDirection = 1;
    public int bEncoderDirection = -1;

    public int lEncoderPrevious = 0;
    public int rEncoderPrevious = 0;
    public int bEncoderPrevious = 0;
    public int lEncoderCounts=0;
    public int rEncoderCounts=0;
    public int bEncoderCounts=0;

    public int deltalEncoder;
    public int deltarEncoder;
    public int deltabEncoder;
    public double deltaHeading;
    public double deltax;
    public double deltay;

    public double headingCorrection=0;


    final static double ENCODER_COUNTS_PER_INCH = 4096.0/(2.0*Math.PI*1.0);
    final static double RADIUS = 2.3465/2;  //6.4054;
    final static double BENCODER_RADIUS = 0.25; //6.758......1; //mm //.75
    Vector2D fieldCentricDelta;
    Vector2D robotCentricDelta;

    public Odometry(DcMotorEx lEncoder, DcMotorEx rEncoder, DcMotorEx bEncoder){
        this.lEncoder=lEncoder;
        this.rEncoder=rEncoder;
        this.bEncoder=bEncoder;

    }

    public void initialize() {

        resetAllEncoders();
        waitAllEncoders();
        setAllRunWithoutEncoders();

        position = new Vector2D(0,0);
    }
    public void setStartLocation(Vector2D startPosition, double startHeading){
        position = new Vector2D(startPosition);
        this.startHeading = Math.toRadians(startHeading);
        heading = normalizeRadians(heading + this.startHeading);
    }

    public void resetAllEncoders(){
        lEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void waitAllEncoders(){
        while(lEncoder.isBusy() || rEncoder.isBusy() || bEncoder.isBusy()){
        }
    }
    public void setAllRunWithoutEncoders(){
        lEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateEncoders(){
        lEncoderCounts = lEncoder.getCurrentPosition();
        rEncoderCounts = rEncoder.getCurrentPosition();
        bEncoderCounts = bEncoder.getCurrentPosition();
    }

    public int getlEncoderCounts() {
        return lEncoderCounts*lEncoderDirection;
    }

    public int getrEncoderCounts() {
        return rEncoderCounts*rEncoderDirection;
    }

    public int getbEncoderCounts() {
        return bEncoderCounts*bEncoderDirection;
    }

    public void updatePosition(){
        updateEncoders();

        deltalEncoder =  getlEncoderCounts() - lEncoderPrevious;
        deltarEncoder = getrEncoderCounts() - rEncoderPrevious;
        deltabEncoder = getbEncoderCounts() - bEncoderPrevious;

        lEncoderPrevious = getlEncoderCounts();
        rEncoderPrevious = getrEncoderCounts();
        bEncoderPrevious = getbEncoderCounts();

        deltaHeading = (deltarEncoder - deltalEncoder)/(2.0*RADIUS*ENCODER_COUNTS_PER_INCH);
        heading = normalizeRadians((getrEncoderCounts()-getlEncoderCounts())/(2.0*RADIUS*ENCODER_COUNTS_PER_INCH) + startHeading+headingCorrection);

        if(deltaHeading == 0){
            deltax = deltabEncoder;
            deltay = (deltalEncoder + deltarEncoder)/2;
        }else{
            double turnRadius = RADIUS*ENCODER_COUNTS_PER_INCH*(deltalEncoder + deltarEncoder)/(deltarEncoder - deltalEncoder);
            double strafeRadius = deltabEncoder/deltaHeading - BENCODER_RADIUS*ENCODER_COUNTS_PER_INCH;

            deltax = turnRadius*(Math.cos(deltaHeading) - 1) + strafeRadius*Math.sin(deltaHeading);
            deltay = turnRadius*Math.sin(deltaHeading) + strafeRadius*(1 - Math.cos(deltaHeading));
        }

        robotCentricDelta = new Vector2D(encoderToInch(deltax), encoderToInch(deltay));

        fieldCentricDelta = new Vector2D(encoderToInch(deltay), encoderToInch(-deltax));
        fieldCentricDelta.rotate(heading);
        position.add(fieldCentricDelta);
    }

    public Vector2D getRobotCentricDelta(){
        return robotCentricDelta;
    }
    public Vector2D getFieldCentricDelta(){
        return fieldCentricDelta;
    }
    public Vector2D currentPosition(){
        return position;
    }
    public double currentHeading(){
        return Math.toDegrees(heading);
    }
    public double relativeHeading(){ return Math.toDegrees(normalizeRadians(heading - startHeading)); }
    public void setHeading(double heading){
        this.heading = normalizeRadians(Math.toRadians(heading));
    }
    public void setHeadingCorrection(double correct){
        headingCorrection = normalizeRadians(correct-this.heading+headingCorrection);
    }
    public void setRelativeHeading(double relativeHeading){
        heading = normalizeRadians(Math.toRadians(relativeHeading) + startHeading);
    }

    public float encoderToInch(double encoder) {
        return (float)(encoder/ENCODER_COUNTS_PER_INCH);
    }

    public int inchToEncoder(float inches) {
        return (int) (inches * ENCODER_COUNTS_PER_INCH);
    }

    public double normalizeRadians(double angle){
        while(angle >= 2*Math.PI) {
            angle -= 2*Math.PI;
        }
        while(angle < 0.0) {
            angle += 2*Math.PI;
        }
        return angle;
    }
}