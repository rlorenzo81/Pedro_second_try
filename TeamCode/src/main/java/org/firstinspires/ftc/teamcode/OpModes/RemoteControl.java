package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Mechanism;
@TeleOp

public class RemoteControl extends OpMode {
    Mechanism drive =new Mechanism();
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftRearMotor;
    private DcMotor leftFrontMotor;



    @Override
    public void init(){
        drive.init(hardwareMap);
        rightFrontMotor =hardwareMap.dcMotor.get("rf");
        rightRearMotor =hardwareMap.dcMotor.get("rr");
        leftRearMotor =hardwareMap.dcMotor.get("lr");
        leftFrontMotor=hardwareMap.dcMotor.get("lf");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override

    public void loop(){
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;


        leftFrontMotor.setPower(-v1);
        rightFrontMotor.setPower(-v2);
        leftRearMotor.setPower(-v3);
        rightRearMotor.setPower(-v4);

        if(gamepad2.a)
        {
            drive.intakeOn();
        }
        if(gamepad2.b){
            drive.intakeOff();
        }

        if(gamepad2.x){
            drive.triggerOn();
        }
        if(gamepad2.y){
            drive.triggerOff();
        }
        if(gamepad2.left_bumper){
            drive.shooterOn();
        }
        if(gamepad2.right_bumper){
            drive.shooterOff();
        }

    }

}
