package org.firstinspires.ftc.teamcode.DuVal.Lab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Class is used for testing a single motor, using trigger for variable power.
//WITHOUT

public class SingleMotor_ButtonToggle extends OpMode {
    private DcMotor motor = null;
    double r_trigger;
    double l_trigger;

    boolean forward;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        forward = true;

        telemetry.addLine("Press 'a' to engage motor.");
        telemetry.addLine("Press 'b' to stop motor.");
        telemetry.addLine("Press d-pad up for motor to go forward");
        telemetry.addLine("Press d-down up for motor to go reverse");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a == true && forward == true) {
            motor.setPower(1);
        }

//        Make motor go reverse? Valid values of motors are [-1, +1]


        if (gamepad1.dpad_up == true) {
            forward = true;
        }

        if (gamepad1.dpad_down == true) {
            forward = false;
        }

        update_telemetry();
    }

    public void update_telemetry () {
        telemetry.addData("Forward mode? ", forward);
        telemetry.addData("A being pressed? ", gamepad1.a);
    }
}
