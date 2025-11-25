package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Mechanism  {
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftShooter;
    private DcMotor rightShooter;
    private DcMotor frontIntake;
    private DcMotor middleIntake;
    private CRServo rightTrigger;
    private CRServo leftTrigger;
    private ElapsedTime runtime=new ElapsedTime();

    public void init (HardwareMap hardwareMap){
        rightFrontMotor =hardwareMap.dcMotor.get("rf");
        rightRearMotor =hardwareMap.dcMotor.get("rr");
        leftRearMotor =hardwareMap.dcMotor.get("lr");
        leftFrontMotor=hardwareMap.dcMotor.get("lf");

        leftShooter=hardwareMap.get(DcMotor.class,"ld");
        rightShooter=hardwareMap.get(DcMotor.class,"rd");
        frontIntake=hardwareMap.get(DcMotor.class,"fi");
       rightTrigger=hardwareMap.get(CRServo.class,"rt");
        leftTrigger=hardwareMap.get(CRServo.class,"lt");
        middleIntake=hardwareMap.get(DcMotor.class,"mi");

       leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

  public void intakeOn(){
        frontIntake.setPower(1);
        middleIntake.setPower(0.4);
  }

  public void intakeOff(){

        frontIntake.setPower(0);
        middleIntake.setPower(0);
  }

  public void triggerOff(){
        leftTrigger.setPower(0);
        rightTrigger.setPower(0);

  }
  public void triggerOn(){
        leftTrigger.setPower(-1);
        rightTrigger.setPower(1);
  }
  public void shooterOn(){
        leftShooter.setPower(-0.75);//
        rightShooter.setPower(0.75);

  }

  public void shooterOff(){
        leftShooter.setPower(0);
        rightShooter.setPower(0);
  }

  public void stopTime(double howLong) {
    runtime.reset();
    if(runtime.milliseconds()< howLong){
        //do nothing
    }

  }

  public void intakeOut(){
        frontIntake.setPower(-1);
  }




}
