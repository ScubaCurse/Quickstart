package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class RobotPosition {

    public static void save(
            HardwareMap hardwareMap,
            double x,
            double y,
            double heading) {

        try {
            JSONObject json = new JSONObject();
            long timestamp = System.currentTimeMillis();
            json.put("stoppedX", x);
            json.put("stoppedY", y);
            json.put("stoppedHeading", heading);
            json.put("stoppedPositionSaved", true);
            json.put("timestamp", timestamp);
            File file = new File(
                    hardwareMap.appContext.getFilesDir(),
                    "RobotPosition.json"
            );

            FileWriter writer = new FileWriter(file);
            writer.write(json.toString(4));
            writer.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public JSONObject load(HardwareMap hardwareMap) {

        File file = new File(
                hardwareMap.appContext.getFilesDir(),
                "robotPosition.json"
        );

        try (BufferedReader reader = new BufferedReader(
                new FileReader(file))) {

            StringBuilder jsonString = new StringBuilder();
            String line;

            while ((line = reader.readLine()) != null) {
                jsonString.append(line);
            }

            return new JSONObject(jsonString.toString());

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}