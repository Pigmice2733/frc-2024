//LimelightHelpers v1.2.1 (March 1, 2023)

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

public class LimelightHelpers {

    public static class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    public static class LimelightTarget_Barcode {

    }

    public static class LimelightTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Classifier() {
        }
    }

    public static class LimelightTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Detector() {
        }
    }

    public static class Results {

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }

        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }

        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }

        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }

        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public Results() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];

        }
    }

    public static class LimelightResults {
        @JsonProperty("Results")
        public Results targetingResults;

        public LimelightResults() {
            targetingResults = new Results();
        }
    }

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    private static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(Units.degreesToRadians(inData[3]),
                        Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        return new Pose2d(
                new Translation2d(inData[0], inData[1]),
                new Rotation2d(Units.degreesToRadians(inData[5])));
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault()
                .getTable(sanitizeName(tableName));
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName,
            String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static double getLimelightNTDouble(String tableName,
            String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName,
            double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNTDoubleArray(String tableName,
            String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNTDoubleArray(String tableName,
            String entryName) {
        return getLimelightNTTableEntry(tableName, entryName)
                .getDoubleArray(new double[0]);
    }

    public static String getLimelightNTString(String tableName,
            String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    public static URL getLimelightURLString(String tableName, String request) {
        try {
            return new URL("http://" + sanitizeName(tableName) + ".local:5807/"
                    + request);
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
            return null;
        }
    }

    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    public static double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNTDouble(limelightName, "getpipe");
    }

    public static String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    /**
     * Switch to getBotPose
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName,
                "camerapose_targetspace");
    }

    public static double[] getCameraPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName,
                "camerapose_robotspace");
    }

    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName,
                "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName,
                "targetpose_robotspace");
    }

    public static double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    public static double getNeuralClassID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tclass");
    }

    public static Pose3d getBotPose3d(String limelightName) {
        return toPose3D(getBotPose(limelightName));
    }

    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        return toPose3D(getBotPose_wpiRed(limelightName));
    }

    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        return toPose3D(getBotPose_wpiBlue(limelightName));
    }

    public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
        return toPose3D(getBotPose_TargetSpace(limelightName));
    }

    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        return toPose3D(getCameraPose_TargetSpace(limelightName));
    }

    public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
        return toPose3D(getCameraPose_RobotSpace(limelightName));
    }

    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        return toPose3D(getTargetPose_CameraSpace(limelightName));
    }

    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        return toPose3D(getTargetPose_RobotSpace(limelightName));
    }

    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        return toPose2D(getBotPose_wpiBlue(limelightName));
    }

    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        return toPose2D(getBotPose_wpiRed(limelightName));
    }

    public static Pose2d getBotPose2d(String limelightName) {
        return toPose2D(getBotPose(limelightName));
    }

    /** Returns true if the "tv" value is 1.0 and false otherwise. */
    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    public static void setPipelineIndex(String limelightName,
            int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by
     * robot code.
     */
    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    public static void setStreamMode_PiPMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    public static void setStreamMode_PiPSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    public static void setCameraMode_Processor(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 0);
    }

    public static void setCameraMode_Driver(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 1);
    }

    private static double[] cropEntries = new double[4];
    private static double[] cprsEntries = new double[6];

    /**
     * Sets the crop window. The crop window in the UI must be completely open
     * for dynamic cropping to work.
     */
    public static void setCropWindow(String limelightName, double cropXMin,
            double cropXMax, double cropYMin, double cropYMax) {
        cropEntries[0] = cropXMin;
        cropEntries[1] = cropXMax;
        cropEntries[2] = cropYMin;
        cropEntries[3] = cropYMax;
        setLimelightNTDoubleArray(limelightName, "crop", cropEntries);
    }

    public static void setCameraPose_RobotSpace(String limelightName,
            double forward, double side, double up,
            double roll, double pitch, double yaw) {
        cprsEntries[0] = forward;
        cprsEntries[1] = side;
        cprsEntries[2] = up;
        cprsEntries[3] = roll;
        cprsEntries[4] = pitch;
        cprsEntries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set",
                cprsEntries);
    }

    public static void setPythonScriptData(String limelightName,
            double[] outgoingPythonData) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    public static double[] getPythonScriptData(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    /**
     * Asynchronously take snapshot.
     */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName,
            String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return synchTakeSnapshot(tableName, snapshotName);
        });
    }

    private static HttpURLConnection connection;

    private static boolean synchTakeSnapshot(String tableName,
            String snapshotName) {
        try {
            connection = (HttpURLConnection) getLimelightURLString(tableName,
                    "capturesnapshot").openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }
            if (connection.getResponseCode() == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    private static long start, end;
    private static LimelightResults results;
    private static ObjectMapper mapper;
    private static boolean profileJSON = false;

    /**
     * Parses Limelight's JSON results dump into a LimelightResults object.
     */
    public static LimelightResults getLatestResults(String limelightName) {
        start = System.nanoTime();
        results = new LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(
                    DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(limelightName),
                    LimelightResults.class);
        } catch (JsonProcessingException e) {
            System.err.println("LLjson error: " + e.getMessage());
        }

        end = System.nanoTime();
        results.targetingResults.latency_jsonParse = (end - start) * .000001; // milliseconds
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", (end - start) * .000001);
        }

        return results;
    }
}