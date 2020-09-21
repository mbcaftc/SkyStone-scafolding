package org.firstinspires.ftc.teamcode.DuVal.Lab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SingleServo_Callibrate extends OpMode {

    private Servo myServo = null;

    private double servoPos = 0.5;

    private double incVal = 0.001;

    @Override
    public void init () {
        myServo = hardwareMap.servo.get("servo");
        myServo.setPosition(servoPos);
    }

    @Override
    public void loop () {
        if (gamepad1.right_bumper) {
            servoPos += incVal;
            servoPos = Range.clip(servoPos,0,1);
            telemetry.addLine("Increase Servo Pos!");
        }
//        What about decrease servo position??

        myServo.setPosition(servoPos);
        updateTelemetry();
    }

    public void updateTelemetry () {
        telemetry.addLine("RB: increase, LB: Decrease");
        telemetry.addLine("x = set to .90, y = set to 0.10");
        telemetry.addData("TestS ervo Positiom: ", myServo.getPosition());
        telemetry.addData("Servo Variable Position: ", servoPos);
        telemetry.update();
    }
}
