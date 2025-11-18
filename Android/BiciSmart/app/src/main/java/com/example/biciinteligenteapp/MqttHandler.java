package com.example.biciinteligenteapp;

import android.content.Context;
import android.content.Intent;
import android.util.Log;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.json.JSONObject;

import java.util.concurrent.atomic.AtomicBoolean;

public class MqttHandler implements MqttCallback {
    private static final String TAG = "MqttHandler";

    public static final String ACTION_DATA_RECEIVE = "com.example.intentservice.intent.action.DATA_RECEIVE";
    public static final String ACTION_CONNECTION_LOST = "com.example.intentservice.intent.action.CONNECTION_LOST";
    private final AtomicBoolean isReconnecting = new AtomicBoolean(false);
    private static MqttHandler instance;
    private MqttClient client;
    private final Context mContext;

    // Constructor privado para Singleton
    private MqttHandler(Context context) {
        // Usar ApplicationContext para evitar memory leaks
        this.mContext = context.getApplicationContext();
    }

    // Método getInstance para obtener la única instancia
    public static synchronized MqttHandler getInstance(Context context) {
        if (instance == null) {
            instance = new MqttHandler(context);
        }
        return instance;
    }

    public void connect(String brokerUrl, String clientId, String username, String password) {
        new Thread(() -> {
            try {
                // Si ya está conectado, no reconectar
                if (client != null && client.isConnected()) {
                    Log.d(TAG, "Cliente ya está conectado");
                    return;
                }

                MqttConnectOptions options = new MqttConnectOptions();
                options.setCleanSession(true);
                options.setUserName(username);
                options.setPassword(password.toCharArray());

                options.setKeepAliveInterval(60);
                options.setConnectionTimeout(30);
                options.setAutomaticReconnect(false);

                MemoryPersistence persistence = new MemoryPersistence();

                client = new MqttClient(brokerUrl, clientId, persistence);
                client.setCallback(this);
                client.connect(options);

                Log.d(TAG, "Conectado exitosamente a " + brokerUrl);

            } catch (MqttException e) {
                Log.e(TAG, "Error al conectar", e);
            }
        }).start();
    }

    public void reconnect(String brokerUrl, String clientId, String username, String password) {
        if (isReconnecting.get()) {
            return;
        }
        isReconnecting.set(true);
        new Thread(() -> {
            while (true) {
                try {
                    Log.d(TAG, "Intentando reconectar...");

                    MqttConnectOptions options = new MqttConnectOptions();
                    options.setCleanSession(true);
                    options.setUserName(username);
                    options.setPassword(password.toCharArray());
                    options.setKeepAliveInterval(60);
                    options.setConnectionTimeout(30);
                    options.setAutomaticReconnect(false);

                    MemoryPersistence persistence = new MemoryPersistence();

                    client = new MqttClient(brokerUrl, clientId, persistence);
                    client.setCallback(this);

                    client.connect(options);

                    Log.d(TAG, "Reconectado correctamente a " + brokerUrl);

                    // Resuscribir al reconectar
                    subscribe(ConfigMQTT.topicData);

                    isReconnecting.set(false);

                    return;

                } catch (Exception e) {
                    Log.e(TAG, "Error al reconectar: " + e.getMessage());
                }

                // Esperar antes del siguiente intento
                try {
                    //noinspection BusyWait
                    Thread.sleep(3000);
                } catch (InterruptedException ignored) {}
            }
        }).start();
    }

    public void disconnect() {
        new Thread(() -> {
            try {
                if (client != null && client.isConnected()) {
                    client.disconnect();
                    Log.d(TAG, "Desconectado exitosamente");
                }
            } catch (MqttException e) {
                Log.e(TAG, "Error al desconectar", e);
            }
        }).start();
    }

    public boolean isConnected() {
        return client != null && client.isConnected();
    }

    public void publish(String topic, String message) {
        new Thread(() -> {
            try {
                if (client != null && client.isConnected()) {
                    MqttMessage mqttMessage = new MqttMessage(message.getBytes());
                    mqttMessage.setQos(2);
                    client.publish(topic, mqttMessage);
                    Log.d(TAG, "Mensaje publicado en " + topic + ": " + message);
                } else {
                    Log.w(TAG, "No se puede publicar: cliente no conectado");
                }
            } catch (MqttException e) {
                Log.e(TAG, "Error al publicar", e);
            }
        }).start();
    }

    public void subscribe(String topic) {
        new Thread(() -> {
            try {
                if (client != null && client.isConnected()) {
                    client.subscribe(topic);
                    Log.d(TAG, "Suscrito a " + topic);
                } else {
                    Log.w(TAG, "No se puede suscribir: cliente no conectado");
                }
            } catch (MqttException e) {
                Log.e(TAG, "Error al suscribirse", e);
            }
        }).start();
    }

    @Override
    public void connectionLost(Throwable cause) {
        Log.e(TAG, "Conexión perdida: " + cause.getMessage());

        Intent i = new Intent(ACTION_CONNECTION_LOST);
        i.addCategory(Intent.CATEGORY_DEFAULT);
        mContext.sendBroadcast(i);
        reconnect(ConfigMQTT.mqttServer, ConfigMQTT.CLIENT_ID,
                ConfigMQTT.userName, ConfigMQTT.userPass);
    }

    @Override
    public void messageArrived(String topic, MqttMessage message) {
        String msgJson = message.toString();
        Log.d(TAG, "Mensaje recibido en " + topic + ": " + msgJson);

        try {
            JSONObject json = new JSONObject(msgJson);

            if (json.has("value")) {
                double value = json.getDouble("value");
                Log.d(TAG, "Valor extraído correctamente: " + value);

                // Enviar broadcast con el mensaje
                Intent i = new Intent(ACTION_DATA_RECEIVE);
                i.addCategory(Intent.CATEGORY_DEFAULT);
                i.putExtra("msg", value);
                mContext.sendBroadcast(i);

                Log.d(TAG, "Broadcast enviado exitosamente");
            } else {
                Log.w(TAG, "El mensaje no contiene el campo 'value'");
            }

        } catch (Exception e) {
            Log.e(TAG, "Error al procesar mensaje", e);
        }
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken token) {
        Log.d(TAG, "Entrega completada");
    }
}