package com.example.biciinteligenteapp;

import android.animation.ObjectAnimator;
import android.annotation.SuppressLint;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.button.MaterialButton;

import org.json.JSONException;
import org.json.JSONObject;

public class TravelActivity extends AppCompatActivity {
    private MqttHandler mqttHandler;
    private TextView tvTimer, tvDistance, tvTripTitle;
    private ImageView imgCyclist;
    private MaterialButton btnEndTrip;
    public IntentFilter filterReceive;
    public IntentFilter filterConncetionLost;
    private ReceptorOperacion receiver = new ReceptorOperacion();
    private ConnectionLost connectionLost = new ConnectionLost();

    private static final String TAG = "TravelActivity";

    // Cronómetro
    private long startMillis;
    private final Handler handler = new Handler();
    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            long elapsed = System.currentTimeMillis() - startMillis;
            tvTimer.setText(formatElapsed(elapsed));
            handler.postDelayed(this, 1000);
        }
    };

    // Datos de viaje
    private double distanceKm = 0.0;

    // Configuración del usuario (rodado y peso)
    private double wheelInches = 0.0;
    private double userWeightKg = 0.0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_travel);

        mqttHandler = MqttHandler.getInstance(this);

        // Referencias UI
        tvTripTitle = findViewById(R.id.tvTripTitle);
        tvTimer = findViewById(R.id.tvTimer);
        tvDistance = findViewById(R.id.tvDistance);
        imgCyclist = findViewById(R.id.imgCyclist);
        btnEndTrip = findViewById(R.id.btnEndTrip);

        // Animación del ícono del ciclista (sube y baja suavemente)
        ObjectAnimator anim = ObjectAnimator.ofFloat(imgCyclist, "translationY", 0f, -15f, 0f);
        anim.setDuration(1600);
        anim.setRepeatCount(ObjectAnimator.INFINITE);
        anim.start();

        // Cargar ajustes del usuario
        loadUserSettings();

        // Iniciar cronómetro
        startMillis = System.currentTimeMillis();
        handler.post(tick);

        // Finalizar viaje → ir al resumen
        btnEndTrip.setOnClickListener(v -> finishTrip());

        configurarBroadcastReciever();

        Log.d(TAG, "TravelActivity iniciada. Rodado: " + wheelInches + " pulgadas, Peso: " + userWeightKg + " kg");
    }

    private void loadUserSettings() {
        SharedPreferences prefs = getSharedPreferences("UserSettings", MODE_PRIVATE);
        String wheel = prefs.getString("wheel", "");
        String weight = prefs.getString("weight", "");
        try {
            wheelInches = wheel.isEmpty() ? 26.0 : Double.parseDouble(wheel.replace(",", "."));
        } catch (Exception ignored) {
            wheelInches = 26.0;
        }
        try {
            userWeightKg = weight.isEmpty() ? 65.0 : Double.parseDouble(weight.replace(",", "."));
        } catch (Exception ignored) {
            userWeightKg = 65.0;
        }
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    private void configurarBroadcastReciever() {
        filterReceive = new IntentFilter(MqttHandler.ACTION_DATA_RECEIVE);
        filterConncetionLost = new IntentFilter(MqttHandler.ACTION_CONNECTION_LOST);

        filterReceive.addCategory(Intent.CATEGORY_DEFAULT);
        filterConncetionLost.addCategory(Intent.CATEGORY_DEFAULT);

        registerReceiver(receiver, filterReceive, Context.RECEIVER_EXPORTED );
        registerReceiver(connectionLost, filterConncetionLost, Context.RECEIVER_EXPORTED );

        Log.d(TAG, "Receivers registrados correctamente");
        Log.d(TAG, "Esperando broadcasts con action: " + MqttHandler.ACTION_DATA_RECEIVE);
    }

    public void onNewMessage(double wheel_turn_count) {
        Log.d(TAG, "onNewMessage llamado - Vueltas de rueda: " + wheel_turn_count);

        // Calcular la circunferencia de la rueda en metros
        double circumferenceMeters = inchesToMeters(wheelInches) * Math.PI;

        // Calcular distancia total basada en el contador acumulado del ESP32
        distanceKm = (circumferenceMeters * wheel_turn_count) / 1000.0;

        Log.d(TAG, "Circunferencia: " + circumferenceMeters + "m, Distancia calculada: " + distanceKm + " km");

        // Actualizar UI en el hilo principal
        runOnUiThread(() -> {
            tvDistance.setText(String.format("%.2f km", distanceKm));
            Log.d(TAG, "UI actualizada con distancia: " + distanceKm + " km");
        });
    }

    private double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private String formatElapsed(long ms) {
        long totalSec = ms / 1000;
        long h = totalSec / 3600;
        long m = (totalSec % 3600) / 60;
        long s = totalSec % 60;
        return String.format("%02d:%02d:%02d", h, m, s);
    }

    private void finishTrip() {
        handler.removeCallbacks(tick);
        mqttHandler.publish(ConfigMQTT.topicControl, "{\"trips\":0}");

        // Enviar datos al resumen
        Intent intent = new Intent(this, SummaryActivity.class);
        intent.putExtra("distanceKm", distanceKm);
        intent.putExtra("durationMs", System.currentTimeMillis() - startMillis);
        intent.putExtra("wheelInches", wheelInches);
        intent.putExtra("weightKg", userWeightKg);
        startActivity(intent);

        finish();
    }

    public class ConnectionLost extends BroadcastReceiver {
        public void onReceive(Context context, Intent intent) {
            Toast.makeText(getApplicationContext(), "Conexión Perdida", Toast.LENGTH_SHORT).show();
            new Thread(() -> {
                try {
                    Log.d(TAG, "Iniciando reconexión MQTT...");

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
                        Log.d(TAG, "MQTT reconectado y suscrito exitosamente");
                    } else {
                        Log.e(TAG, "No se pudo reconectar a MQTT después de varios intentos");
                    }

                } catch (InterruptedException e) {
                    Log.e(TAG, "Error en reconexión MQTT: " + e.getMessage());
                    e.printStackTrace();
                }
            }).start();
        }
    }

    public class ReceptorOperacion extends BroadcastReceiver {
        public void onReceive(Context context, Intent intent) {
            double msg = intent.getDoubleExtra("msg",0);
            Log.d(TAG, "Broadcast recibido: " + msg);
            onNewMessage(msg);
            /*
            try {
                JSONObject jsonObject = new JSONObject(msgJson);

                // El valor viene como double directamente en el JSON
                if (jsonObject.has("value")) {
                    double wheelTurns = jsonObject.getDouble("value");
                    Log.d(TAG, "Valor parseado correctamente: " + wheelTurns);
                    onNewMessage(wheelTurns);
                } else {
                    Log.e(TAG, "JSON no contiene campo 'value': " + msgJson);
                }

            } catch (JSONException e) {
                Log.e(TAG, "Error parseando JSON: " + e.getMessage());
                Log.e(TAG, "JSON recibido: " + msgJson);
                e.printStackTrace();
            }*/
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        handler.removeCallbacks(tick);

        // Desregistrar los receivers
        try {
            unregisterReceiver(receiver);
            unregisterReceiver(connectionLost);
            Log.d(TAG, "Receivers desregistrados correctamente");
        } catch (Exception e) {
            Log.e(TAG, "Error al desregistrar receivers: " + e.getMessage());
        }
    }
}