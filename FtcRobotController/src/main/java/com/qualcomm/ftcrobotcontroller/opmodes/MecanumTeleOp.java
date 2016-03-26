package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class MecanumTeleOp extends OpMode{
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;
    DcMotor shooterleft;
    DcMotor shooterright;
    DcMotor collector;
    Servo feeder;

    public MecanumTeleOp(){

    }
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorRearLeft = hardwareMap.dcMotor.get("RearLeft");
        motorRearRight = hardwareMap.dcMotor.get("RearRight");
        shooterleft = hardwareMap.dcMotor.get("shooterleft");
        shooterright = hardwareMap.dcMotor.get("shooterright");
        collector = hardwareMap.dcMotor.get("collector");
        feeder = hardwareMap.servo.get("feeder");

        feeder.setPosition(.5);

        collector.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void loop(){


        float frontleft = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
        float frontright = gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
        float rearleft = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
        float rearright = -gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;

        frontleft = Range.clip(frontleft, -1, 1);
        frontright = Range.clip(frontright, -1, 1);
        rearleft = Range.clip(rearleft, -1, 1);
        rearright = Range.clip(rearright, -1, 1);

        frontleft = (float)scaleInput(frontleft);
        frontright = (float)scaleInput(frontright);
        rearleft = (float)scaleInput(rearleft);
        rearright = (float)scaleInput(rearright);

        motorFrontLeft.setPower(frontleft);
        motorFrontRight.setPower(frontright);
        motorRearLeft.setPower(rearleft);
        motorRearRight.setPower(rearright);

        if(gamepad1.a){
            shooterleft.setPower(1);
            shooterright.setPower(1);
        }
        if(gamepad1.y){
            shooterleft.setPower(0);
            shooterright.setPower(0);
        }
        if(gamepad1.x){
            collector.setPower(1);
        }
        if(gamepad1.b){
            collector.setPower(0);
        }
        if(gamepad2.a) {
            feeder.setPosition(0);
        }
        else{
            feeder.setPosition(.5);
        }
        //******TELEMETRY******

        telemetry.addData("1 ", "Front Left Power: " + String.format("%.2f", frontleft));
        telemetry.addData("2 ", "Front Right Power: " + String.format("%.2f", rearleft));
        telemetry.addData("3 ", "Rear Left Power: " + String.format("%.2f", frontright));
        telemetry.addData("4 ", "Rear Right Power: " + String.format("%.2f", rearright));
        telemetry.addData("5 ", "Shooter Power: " + String.format("%.2f", shooterleft.getPower()));
        telemetry.addData("6 ", "Collector Power: " + String.format("%.2f", collector.getPower()));
        telemetry.addData("7 ", "Feeder Target Position: " + String.format("%.2f", feeder.getPosition()));
    }
    @Override
    public void stop(){

    }

    double scaleInput(double dVal)  {

        return  dVal * dVal * dVal;
    }

}
