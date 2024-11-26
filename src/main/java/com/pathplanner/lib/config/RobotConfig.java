package com.pathplanner.lib.config;

import org.json.simple.JSONObject;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class RobotConfig {
    private double robotWidth;
    private double robotLength;
    private boolean holonomicMode;
    private double defaultMaxVel;
    private double defaultMaxAccel;
    private double defaultMaxAngVel;
    private double defaultMaxAngAccel;
    private double defaultNominalVoltage;
    private double robotMass;
    private double robotMOI;
    private double robotTrackwidth;
    private double driveWheelRadius;
    private double driveGearing;
    private double maxDriveSpeed;
    private String driveMotorType;
    private double driveCurrentLimit;
    private double wheelCOF;
    private double flModuleX;
    private double flModuleY;
    private double frModuleX;
    private double frModuleY;
    private double blModuleX;
    private double blModuleY;
    private double brModuleX;
    private double brModuleY;
    private double bumperOffsetX;
    private double bumperOffsetY;

    public static RobotConfig fromGUISettings() throws IOException, ParseException {
        JSONParser parser = new JSONParser();
        JSONObject jsonObject = (JSONObject) parser.parse(new FileReader("path/to/settings.json"));

        RobotConfig config = new RobotConfig();

        config.robotWidth = getDoubleValue(jsonObject, "robotWidth", 0.9);
        config.robotLength = getDoubleValue(jsonObject, "robotLength", 0.9);
        config.holonomicMode = getBooleanValue(jsonObject, "holonomicMode", true);
        config.defaultMaxVel = getDoubleValue(jsonObject, "defaultMaxVel", 3.0005);
        config.defaultMaxAccel = getDoubleValue(jsonObject, "defaultMaxAccel", 3.0005);
        config.defaultMaxAngVel = getDoubleValue(jsonObject, "defaultMaxAngVel", 5450.0);
        config.defaultMaxAngAccel = getDoubleValue(jsonObject, "defaultMaxAngAccel", 7250.005);
        config.defaultNominalVoltage = getDoubleValue(jsonObject, "defaultNominalVoltage", 12.0);
        config.robotMass = getDoubleValue(jsonObject, "robotMass", 74.088);
        config.robotMOI = getDoubleValue(jsonObject, "robotMOI", 6.883);
        config.robotTrackwidth = getDoubleValue(jsonObject, "robotTrackwidth", 0.546);
        config.driveWheelRadius = getDoubleValue(jsonObject, "driveWheelRadius", 0.048);
        config.driveGearing = getDoubleValue(jsonObject, "driveGearing", 6.143);
        config.maxDriveSpeed = getDoubleValue(jsonObject, "maxDriveSpeed", 8.0);
        config.driveMotorType = getStringValue(jsonObject, "driveMotorType", "NEO");
        config.driveCurrentLimit = getDoubleValue(jsonObject, "driveCurrentLimit", 60.0);
        config.wheelCOF = getDoubleValue(jsonObject, "wheelCOF", 1.5);
        config.flModuleX = getDoubleValue(jsonObject, "flModuleX", 0.273);
        config.flModuleY = getDoubleValue(jsonObject, "flModuleY", 0.273);
        config.frModuleX = getDoubleValue(jsonObject, "frModuleX", 0.273);
        config.frModuleY = getDoubleValue(jsonObject, "frModuleY", -0.273);
        config.blModuleX = getDoubleValue(jsonObject, "blModuleX", -0.273);
        config.blModuleY = getDoubleValue(jsonObject, "blModuleY", 0.273);
        config.brModuleX = getDoubleValue(jsonObject, "brModuleX", -0.273);
        config.brModuleY = getDoubleValue(jsonObject, "brModuleY", -0.273);
        config.bumperOffsetX = getDoubleValue(jsonObject, "bumperOffsetX", 0.0);
        config.bumperOffsetY = getDoubleValue(jsonObject, "bumperOffsetY", 0.0);

        return config;
    }

    private static double getDoubleValue(JSONObject jsonObject, String key, double defaultValue) {
        Object value = jsonObject.get(key);
        return value != null ? ((Number) value).doubleValue() : defaultValue;
    }

    private static boolean getBooleanValue(JSONObject jsonObject, String key, boolean defaultValue) {
        Object value = jsonObject.get(key);
        return value != null ? (Boolean) value : defaultValue;
    }

    private static String getStringValue(JSONObject jsonObject, String key, String defaultValue) {
        Object value = jsonObject.get(key);
        return value != null ? (String) value : defaultValue;
    }
}
