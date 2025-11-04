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

public class MqttHandler implements MqttCallback {
    private static final String TAG = "MqttHandler";

    public static final String BROKER_URL = "tcp://industrial.api.ubidots.com:1883";
    public static final String CLIENT_ID = "mqttx_f9bfd3ww";
    public static final String USER = "BBUS-HdFdBXCCMsjGsKwFNnLh7Y7vzLTasv";
    public static final String PASS = "BBUS-HdFdBXCCMsjGsKwFNnLh7Y7vzLTasv";
    public static final String TOPIC_DATA = "/v1.6/devices/bici/data/lv";
    public static final String TOPIC_CONTROL = "/v1.6/devices/bici";

    public static final String ACTION_DATA_RECEIVE = "com.example.intentservice.intent.action.DATA_RECEIVE";
    public static final String ACTION_CONNECTION_LOST = "com.example.intentservice.intent.action.CONNECTION_LOST";

    private static MqttHandler instance;
    private MqttClient client;
    private Context mContext;

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

                // Opciones adicionales recomendadas
                options.setKeepAliveInterval(60);
                options.setConnectionTimeout(30);
                options.setAutomaticReconnect(true);

                // Set up the persistence layer
                MemoryPersistence persistence = new MemoryPersistence();

                client = new MqttClient(brokerUrl, clientId, persistence);
                client.setCallback(this);
                client.connect(options);

                Log.d(TAG, "Conectado exitosamente a " + brokerUrl);

            } catch (MqttException e) {
                Log.e(TAG, "Error al conectar: " + e.getMessage() + "  " + e.getCause());
                e.printStackTrace();
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
                Log.e(TAG, "Error al desconectar: " + e.getMessage());
                e.printStackTrace();
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
                Log.e(TAG, "Error al publicar: " + e.getMessage());
                e.printStackTrace();
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
                Log.e(TAG, "Error al suscribirse: " + e.getMessage());
                e.printStackTrace();
            }
        }).start();
    }

    public void unsubscribe(String topic) {
        new Thread(() -> {
            try {
                if (client != null && client.isConnected()) {
                    client.unsubscribe(topic);
                    Log.d(TAG, "Desuscrito de " + topic);
                }
            } catch (MqttException e) {
                Log.e(TAG, "Error al desuscribirse: " + e.getMessage());
                e.printStackTrace();
            }
        }).start();
    }

    @Override
    public void connectionLost(Throwable cause) {
        Log.e(TAG, "Conexión perdida: " + cause.getMessage());

        Intent i = new Intent(ACTION_CONNECTION_LOST);
        i.addCategory(Intent.CATEGORY_DEFAULT);
        mContext.sendBroadcast(i);
    }

    @Override
    /*
     * ESTE METODO SE EJECUTA EN UN THREAD SECUNDARIO, POR LO QUE NO PUEDE
     * ACTUALIZAR LA INTERFAZ DE USUARIO DIRECTAMENTE. POR LO QUE DEBE NOTIFICAR
     * AL MAIN THREAD PARA QUE LO MUESTRE EN LA INTERFAZ DE USUARIO
     */
    public void messageArrived(String topic, MqttMessage message) throws Exception {
        String msgJson = message.toString();
        Log.d(TAG, "Mensaje recibido en " + topic + ": " + msgJson);

        try {
            JSONObject json = new JSONObject(msgJson);

            // Validar que existe el campo "value" y obtenerlo como double
            if (json.has("value")) {
                double value = json.getDouble("value"); // Usar getDouble en lugar de getString
                Log.d(TAG, "Valor extraído correctamente: " + value);

                // Enviar broadcast con el mensaje completo
                Intent i = new Intent(ACTION_DATA_RECEIVE);
                i.addCategory(Intent.CATEGORY_DEFAULT);
                i.putExtra("msg", value);
                mContext.sendBroadcast(i);

                Log.d(TAG, "Broadcast enviado exitosamente");
            } else {
                Log.w(TAG, "El mensaje no contiene el campo 'value'");
            }

        } catch (Exception e) {
            Log.e(TAG, "Error al procesar mensaje: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken token) {
        Log.d(TAG, "Entrega completada");
    }

    // Método para limpiar recursos si es necesario
    public void destroy() {
        new Thread(() -> {
            disconnect();
            // Esperar un poco para que se complete la desconexión
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            instance = null;
        }).start();
    }
}