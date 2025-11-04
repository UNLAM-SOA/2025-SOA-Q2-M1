package com.example.biciinteligenteapp;
public class ConfigMQTT {

    public static final String CLIENT_ID = "AndroidClient";

    // Servidor Ubidots
    public static final String MQTT_SERVER_UBIDOTS = "tcp://industrial.api.ubidots.com:1883";
    public static final String USER_NAME_UBIDOTS = "BBUS-HdFdBXCCMsjGsKwFNnLh7Y7vzLTasv";
    public static final String USER_PASS_UBIDOTS = "BBUS-HdFdBXCCMsjGsKwFNnLh7Y7vzLTasv";
    public static final String TOPIC_DATA_UBIDOTS = "/v1.6/devices/bici/data";
    public static final String TOPIC_CONTROL_UBIDOTS = "/v1.6/devices/bici";

    // Variables globales modificables
    public static String mqttServer;
    public static String userName;
    public static String userPass;
    public static String topicData;
    public static String topicControl;

    // Métodos para seleccionar Ubidots
    public static void useServerUBIDOTS() {
        mqttServer = MQTT_SERVER_UBIDOTS;
        userName = USER_NAME_UBIDOTS;
        userPass = USER_PASS_UBIDOTS;
        topicData = TOPIC_DATA_UBIDOTS;
        topicControl = TOPIC_CONTROL_UBIDOTS;
    }
}
