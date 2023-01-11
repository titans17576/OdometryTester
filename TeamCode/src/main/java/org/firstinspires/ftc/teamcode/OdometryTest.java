package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="OdometryTest")
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        robot R = new robot(hardwareMap);
        Odometry odo = new Odometry(R.leftMotor,R.rightMotor,R.backMotor);
        odo.initialize();
        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("left", R.leftMotor.getCurrentPosition());
            telemetry.addData("right", R.rightMotor.getCurrentPosition());
            telemetry.addData("back", R.backMotor.getCurrentPosition());
            odo.updatePosition();
            telemetry.addData("Pos", odo.currentPosition());
            telemetry.addData("Current Heading", odo.currentHeading());
            telemetry.addData("Relative Heading", odo.relativeHeading());

            telemetry.addData("RADIUS",Odometry.RADIUS);
            telemetry.addData("BENCODER_RADIUS" +
                    "",Odometry.BENCODER_RADIUS);
            telemetry.update();

        }
    }
}
