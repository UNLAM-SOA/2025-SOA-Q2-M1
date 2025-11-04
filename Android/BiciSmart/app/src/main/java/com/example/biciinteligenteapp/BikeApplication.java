package com.example.biciinteligenteapp;

import android.app.Application;
import android.util.Log;

public class BikeApplication extends Application {
    private static final String TAG = "BikeApplication";

    @Override
    public void onCreate() {
        super.onCreate();

        // Inicializar MQTT en un thread separado
        new Thread(() -> {
            try {
                Log.d(TAG, "Iniciando conexión MQTT...");

                MqttHandler mqttHandler = MqttHandler.getInstance(this);
                ConfigMQTT.useServerUBIDOTS();
                mqttHandler.connect(ConfigMQTT.mqttServer, ConfigMQTT.CLIENT_ID,
                        ConfigMQTT.userName, ConfigMQTT.userPass);

                // Esperar a que se conecte
                int intentos = 0;
                while (!mqttHandler.isConnected() && intentos < 10) {
                    Thread.sleep(500);
                    intentos++;
                }

                if (mqttHandler.isConnected()) {
                    mqttHandler.subscribe(ConfigMQTT.topicData);
                    Log.d(TAG, "MQTT conectado y suscrito exitosamente");
                } else {
                    Log.e(TAG, "No se pudo conectar a MQTT después de varios intentos");
                }

            } catch (InterruptedException e) {
                Log.e(TAG, "Error en inicialización MQTT: " + e.getMessage());
                e.printStackTrace();
            }
        }).start();
    }

    @Override
    public void onTrimMemory(int level) {
        super.onTrimMemory(level);
        if (level == TRIM_MEMORY_COMPLETE) {
            // Sistema está cerrando la app
            MqttHandler.getInstance(this).disconnect();
        }
    }

    @Override
    public void onTerminate() {
        super.onTerminate();
        // Desconectar cuando la app se cierre completamente
        MqttHandler.getInstance(this).disconnect();
    }
}