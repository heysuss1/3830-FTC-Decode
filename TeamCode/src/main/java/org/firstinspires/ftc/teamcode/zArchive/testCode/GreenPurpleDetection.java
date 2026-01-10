package org.firstinspires.ftc.teamcode.zArchive.testCode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Green & Purple Detection", group = "Sensor")
public class GreenPurpleDetection extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor3");

        waitForStart();

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert normalized RGB (0–1) to HSV
            float[] hsv = new float[3];
            Color.RGBToHSV(
                    (int) (colors.red * 255),
                    (int) (colors.green * 255),
                    (int) (colors.blue * 255),
                    hsv
            );

            float hue = hsv[0];
            float saturation = hsv[1];
            float value = hsv[2];

            boolean isGreen = isGreen(hue, saturation, value);
            boolean isPurple = isPurple(hue, saturation, value);

            telemetry.addData("Hue", hue);
            telemetry.addData("Saturation", saturation);
            telemetry.addData("Value", value);
            telemetry.addData("Green Detected", isGreen);
            telemetry.addData("Purple Detected", isPurple);
            telemetry.update();
        }
    }

    private boolean isGreen(float hue, float sat, float val) {
        return (hue >= 80 && hue <= 150) &&
                (sat > 0.4) &&
                (val > 0.2);
    }

    private boolean isPurple(float hue, float sat, float val) {
        return (hue >= 260 && hue <= 320) &&
                (sat > 0.4) &&
                (val > 0.2);
    }
}