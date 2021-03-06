package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

public class MecanumAutonomous extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;
    DcMotor shooterleft;
    DcMotor shooterright;
    DcMotor collector;
    Servo feeder;

    public void resetEncoders(){
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRearLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRearRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        if(motorFrontLeft.getCurrentPosition() == 0){
            step++;
        }
    }


    public void driveStraight(double driveDistance){
        double Distance = 90.48399 * driveDistance;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontLeft.setTargetPosition(roundedDistance);
        motorFrontRight.setTargetPosition(roundedDistance);
        motorRearLeft.setTargetPosition(roundedDistance);
        motorRearRight.setTargetPosition(roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorRearLeft.setPower(1);
        motorRearRight.setPower(1);

        if(Math.abs(motorFrontRight.getCurrentPosition()) >= Math.abs(motorFrontRight.getTargetPosition())){
            step++;
        }
    }
    public void driveDiagonalLeft(double driveDistance){
        double Distance = 90.48399 * driveDistance * 2;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontRight.setTargetPosition(roundedDistance);
        motorRearLeft.setTargetPosition(roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontRight.setPower(1);
        motorRearLeft.setPower(1);
        motorFrontLeft.setPower(0);
        motorRearRight.setPower(0);

        if(Math.abs(motorFrontRight.getCurrentPosition()) >= Math.abs(motorFrontRight.getTargetPosition())){
            step++;
        }
    }
    public void driveDiagonalRight(double driveDistance){
        double Distance = 90.48399 * driveDistance * 2;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontLeft.setTargetPosition(roundedDistance);
        motorRearRight.setTargetPosition(roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorRearRight.setPower(1);
        motorFrontRight.setPower(0);
        motorRearLeft.setPower(0);

        if(Math.abs(motorFrontLeft.getCurrentPosition()) >= Math.abs(motorFrontLeft.getTargetPosition())){
            step++;
        }
    }
    public void driveLeft(double driveDistance){
        double Distance = 90.48399 * driveDistance * 2;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontLeft.setTargetPosition(-roundedDistance);
        motorFrontRight.setTargetPosition(roundedDistance);
        motorRearLeft.setTargetPosition(roundedDistance);
        motorRearRight.setTargetPosition(-roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorRearLeft.setPower(1);
        motorRearRight.setPower(1);

        if(Math.abs(motorFrontRight.getCurrentPosition()) >= Math.abs(motorFrontRight.getTargetPosition())){
            step++;
        }
    }
    public void driveRight(double driveDistance){
        double Distance = 90.48399 * driveDistance * 2;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontLeft.setTargetPosition(roundedDistance);
        motorFrontRight.setTargetPosition(-roundedDistance);
        motorRearLeft.setTargetPosition(-roundedDistance);
        motorRearRight.setTargetPosition(roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorRearLeft.setPower(1);
        motorRearRight.setPower(1);

        if(Math.abs(motorFrontLeft.getCurrentPosition()) >= Math.abs(motorFrontLeft.getTargetPosition())){
            step++;
        }
    }
    public void turnLeft(double driveDistance){
        double Distance = 90.48399 * driveDistance;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontLeft.setTargetPosition(-roundedDistance);
        motorFrontRight.setTargetPosition(roundedDistance);
        motorRearLeft.setTargetPosition(-roundedDistance);
        motorRearRight.setTargetPosition(roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorRearLeft.setPower(1);
        motorRearRight.setPower(1);

        if(Math.abs(motorFrontRight.getCurrentPosition()) >= Math.abs(motorFrontRight.getTargetPosition())){
            step++;
        }
    }
    public void turnRight(double driveDistance){
        double Distance = 90.48399 * driveDistance;
        int roundedDistance = (int)Math.round(Distance);
        motorFrontLeft.setTargetPosition(roundedDistance);
        motorFrontRight.setTargetPosition(-roundedDistance);
        motorRearLeft.setTargetPosition(roundedDistance);
        motorRearRight.setTargetPosition(-roundedDistance);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorRearLeft.setPower(1);
        motorRearRight.setPower(1);

        if(Math.abs(motorFrontLeft.getCurrentPosition()) >= Math.abs(motorFrontLeft.getTargetPosition())){
            step++;
        }
    }

    int step;
    @Override
    public void init(){
        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorRearLeft = hardwareMap.dcMotor.get("RearLeft");
        motorRearRight = hardwareMap.dcMotor.get("RearRight");
        shooterleft = hardwareMap.dcMotor.get("shooterleft");
        shooterright = hardwareMap.dcMotor.get("shooterright");
        collector = hardwareMap.dcMotor.get("collector");
        feeder = hardwareMap.servo.get("feeder");

        feeder.setPosition(.5);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRearLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRearRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }
    @Override
    public void start(){
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        step = 1;
    }
    @Override
    public void loop(){
        switch (step){
            case 1:
                driveStraight(10);
                break;
            case 2:
                resetEncoders();
                break;
            case 3:
                turnLeft(10);
                break;
            case 4:
                resetEncoders();
                break;
            case 5:
                turnRight(10);
                break;
            case 6:
                resetEncoders();
                break;
            case 7:
                driveLeft(10);
                break;
            case 8:
                resetEncoders();
                break;
            case 9:
                driveRight(10);
                break;
            case 10:
                resetEncoders();
                break;
            case 11:
                driveDiagonalLeft(10);
                break;
            case 12:
                resetEncoders();
                break;
            case 13:
                driveDiagonalRight(10);
                break;
            case 14:
                resetEncoders();
                break;
            case 15:
                driveStraight(10);
                break;
            default:
                stop();
                break;

        }
        telemetry.addData("1 ", "Front Left TGTPOS: " + motorFrontLeft.getTargetPosition());
        telemetry.addData("2 ", "Front Right TGTPOS: " + motorFrontRight.getTargetPosition());
        telemetry.addData("3 ", "Rear Left TGTPOS: " + motorRearLeft.getTargetPosition());
        telemetry.addData("4 ", "Rear Right TGTPOS: " + motorRearRight.getTargetPosition());
        telemetry.addData("5 ", "Shooter Power: " + String.format("%.2f", shooterleft.getPower()));
        telemetry.addData("6 ", "Collector Power: " + String.format("%.2f", collector.getPower()));
        telemetry.addData("7 ", "Feeder Position: " + String.format("%.2f", feeder.getPosition()));
        telemetry.addData("8 ", "Case: " +  step);
    }

}
