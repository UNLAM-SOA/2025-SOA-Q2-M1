package com.example.biciinteligenteapp;

import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.os.Bundle;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.button.MaterialButton;

public class BikeStatusActivity extends AppCompatActivity {
    private MqttHandler mqttHandler;
    private static final String PREFS_NAME = "BikePrefs";
    private static final String KEY_BIKE_STATE = "isBikeOn";

    private MaterialButton btnPower, btnStartTrip;
    private TextView tvBikeStatus;
    private ImageView imgLogo;
    private ImageButton btnLogout, btnSettings;
    private boolean isBikeOn = false;
    private SharedPreferences prefs;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_bike_status);

        mqttHandler = MqttHandler.getInstance(this);

        // Inicializar SharedPreferences
        prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);

        // Vincular vistas
        btnPower = findViewById(R.id.btnPower);
        btnStartTrip = findViewById(R.id.btnStartTrip);
        tvBikeStatus = findViewById(R.id.tvBikeStatus);
        imgLogo = findViewById(R.id.imgLogo);
        btnLogout = findViewById(R.id.btnLogout);
        btnSettings = findViewById(R.id.btnSettings);

        // Restaurar estado guardado de la bicicleta
        isBikeOn = prefs.getBoolean(KEY_BIKE_STATE, false);
        updateBikeStatus(isBikeOn);

        // Listener para cerrar sesión
        btnLogout.setOnClickListener(v -> {
            Toast.makeText(this, getString(R.string.msg_logged_out), Toast.LENGTH_SHORT).show();
            Intent intent = new Intent(BikeStatusActivity.this, LoginActivity.class);
            intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
            startActivity(intent);
            finish();
        });

        // Listener para abrir ajustes
        btnSettings.setOnClickListener(v -> {
            Intent intent = new Intent(this, SettingsActivity.class);
            startActivity(intent);
        });

        // Listener para alternar encendido/apagado
        btnPower.setOnClickListener(v -> toggleBikePower());

        // Listener para iniciar viaje
        btnStartTrip.setOnClickListener(v -> startTrip());
    }

    // Alterna el estado de encendido de la bicicleta
    private void toggleBikePower() {
        if (mqttHandler.isConnected()){
            isBikeOn = !isBikeOn;

            // Guardar estado en SharedPreferences
            prefs.edit().putBoolean(KEY_BIKE_STATE, isBikeOn).apply();

            updateBikeStatus(isBikeOn);

            if (isBikeOn) {
                Toast.makeText(this, getString(R.string.msg_bike_on), Toast.LENGTH_SHORT).show();
                // Enviar comando ON al ESP32
                mqttHandler.publish(ConfigMQTT.topicControl, "{\"control\":1}");
            } else {
                Toast.makeText(this, getString(R.string.msg_bike_off), Toast.LENGTH_SHORT).show();
                // Enviar comando OFF al ESP32
                mqttHandler.publish(ConfigMQTT.topicControl, "{\"control\":0}");
            }
        } else {
            Toast.makeText(this, "Error: sin conexión", Toast.LENGTH_SHORT).show();
            mqttHandler.reconnect(ConfigMQTT.mqttServer, ConfigMQTT.CLIENT_ID,
                    ConfigMQTT.userName, ConfigMQTT.userPass);
        }

    }

    // Actualiza la interfaz según el estado de la bicicleta
    private void updateBikeStatus(boolean isOn) {
        if (isOn) {
            // Bicicleta encendida - Mostrar botón ROJO para apagar
            tvBikeStatus.setText(getString(R.string.status_on));
            tvBikeStatus.setTextColor(Color.parseColor("#43A047"));
            imgLogo.setColorFilter(Color.parseColor("#43A047"));

            btnPower.setText(getString(R.string.btn_turn_off));
            btnPower.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#E53935"))); // Rojo

            btnStartTrip.setEnabled(true);
            btnStartTrip.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#43A047")));
        } else {
            // Bicicleta apagada - Mostrar botón VERDE para encender
            tvBikeStatus.setText(getString(R.string.status_off));
            tvBikeStatus.setTextColor(Color.parseColor("#212121"));
            imgLogo.setColorFilter(Color.parseColor("#9E9E9E"));

            btnPower.setText(getString(R.string.btn_turn_on));
            btnPower.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#43A047"))); // Verde

            btnStartTrip.setEnabled(false);
            btnStartTrip.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#BDBDBD")));
        }
    }

    // Inicia un nuevo viaje si la bicicleta está encendida
    private void startTrip() {
        if (mqttHandler.isConnected()){
            if (!isBikeOn) {
                Toast.makeText(this, getString(R.string.msg_turn_on_first), Toast.LENGTH_SHORT).show();
            } else {
                mqttHandler.publish(ConfigMQTT.topicControl, "{\"trips\":1}");
                Toast.makeText(this, getString(R.string.msg_trip_started), Toast.LENGTH_SHORT).show();
                Intent intent = new Intent(this, TravelActivity.class);
                startActivity(intent);
                finish();
            }
        } else {
            Toast.makeText(this, "Error: sin conexión", Toast.LENGTH_SHORT).show();
            mqttHandler.reconnect(ConfigMQTT.mqttServer, ConfigMQTT.CLIENT_ID,
                    ConfigMQTT.userName, ConfigMQTT.userPass);
        }
    }
}