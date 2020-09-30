package org.firstinspires.ftc.teamcode.DuVal.Lab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LaunchTest_Motor extends OpMode {
    DcMotor launcher = null;
    @Override
    public void init() {
        launcher = hardwareMap.dcMotor.get("launcher_motor");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addLine("caption: a pressed");
            launcher.setPower(1);

        }


    }
}
