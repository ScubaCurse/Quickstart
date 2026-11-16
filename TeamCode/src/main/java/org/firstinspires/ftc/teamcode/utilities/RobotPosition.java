
package org.firstinspires.ftc.teamcode.utilities;

import android.util.AtomicFile;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.charset.StandardCharsets;

public class RobotPosition {

    private static final String TAG = "RobotPosition";

    private static AtomicFile getPositionFile(HardwareMap hardwareMap) {

        File file = new File(
                hardwareMap.appContext.getFilesDir(),
                "RobotPosition.json"
        );

        return new AtomicFile(file);
    }

    public static void save(
            HardwareMap hardwareMap,
            double x,
            double y,
            double heading) {

        // Reject invalid coordinates
        if (!Double.isFinite(x) ||
                !Double.isFinite(y) ||
                !Double.isFinite(heading)) {

            Log.e(TAG, "Invalid position. Not saving.");
            return;
        }

        AtomicFile file = getPositionFile(hardwareMap);

        FileOutputStream stream = null;

        try {

            JSONObject json = new JSONObject();

            json.put("stoppedX", x);
            json.put("stoppedY", y);
            json.put("stoppedHeading", heading);
            json.put("stoppedPositionSaved", true);
            json.put("timestamp", System.currentTimeMillis());

            // Begin atomic write
            stream = file.startWrite();

            stream.write(
                    json.toString(4).getBytes(StandardCharsets.UTF_8)
            );

            // Finish and commit write
            file.finishWrite(stream);

            Log.d(
                    TAG,
                    "SAVED X=" + x +
                            " Y=" + y +
                            " HEADING=" + heading
            );

        } catch (Exception e) {

            if (stream != null) {
                file.failWrite(stream);
            }

            Log.e(TAG, "FAILED TO SAVE POSITION", e);
        }
    }

    public static JSONObject load(HardwareMap hardwareMap) {

        AtomicFile file = getPositionFile(hardwareMap);

        try {

            byte[] data = file.readFully();

            JSONObject json = new JSONObject(
                    new String(data, StandardCharsets.UTF_8)
            );

            if (!json.optBoolean("stoppedPositionSaved", false)) {
                Log.e(TAG, "Position not marked as saved.");
                return null;
            }

            double x = json.optDouble("stoppedX", Double.NaN);
            double y = json.optDouble("stoppedY", Double.NaN);
            double heading = json.optDouble("stoppedHeading", Double.NaN);

            if (!Double.isFinite(x) ||
                    !Double.isFinite(y) ||
                    !Double.isFinite(heading)) {

                Log.e(TAG, "Invalid coordinates in JSON.");
                return null;
            }

            Log.d(
                    TAG,
                    "LOADED X=" + x +
                            " Y=" + y +
                            " HEADING=" + heading
            );

            return json;

        } catch (Exception e) {

            Log.e(TAG, "FAILED TO LOAD POSITION", e);
            return null;
        }
    }
}
