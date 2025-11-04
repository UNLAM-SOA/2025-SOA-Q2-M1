package com.example.biciinteligenteapp;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.widget.ImageButton;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.button.MaterialButton;
import com.google.android.material.textfield.TextInputEditText;
import com.google.android.material.textfield.TextInputLayout;

public class SettingsActivity extends AppCompatActivity {

    private TextInputLayout tilWeight, tilWheel;
    private TextInputEditText txtWeight, txtWheel;
    private MaterialButton btnSaveSettings;
    private ImageButton btnBack;

    // Valores por defecto sino se ingresan valores
    private static final double DEFAULT_WEIGHT_KG = 70.0;
    private static final double DEFAULT_WHEEL_INCHES = 26.0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        // Referencias UI
        tilWeight = findViewById(R.id.tilWeight);
        tilWheel = findViewById(R.id.tilWheel);
        txtWeight = findViewById(R.id.txtWeight);
        txtWheel = findViewById(R.id.txtWheel);
        btnSaveSettings = findViewById(R.id.btnSaveSettings);
        btnBack = findViewById(R.id.btnBack);

        // Cargar valores previos (si existen o usar los predeterminados)
        SharedPreferences prefs = getSharedPreferences("UserSettings", MODE_PRIVATE);

        String savedWeight = prefs.getString("weight", "");
        String savedWheel = prefs.getString("wheel", "");

        if (savedWeight.isEmpty() || savedWheel.isEmpty()) {
            // Aplicar valores por defecto
            txtWeight.setText(String.valueOf(DEFAULT_WEIGHT_KG));
            txtWheel.setText(String.valueOf(DEFAULT_WHEEL_INCHES));

            Toast.makeText(this,
                    "Se aplicaron valores por defecto (peso y rodado).",
                    Toast.LENGTH_SHORT).show();
        } else {
            txtWeight.setText(savedWeight);
            txtWheel.setText(savedWheel);
        }

        // Botón volver atrás
        btnBack.setOnClickListener(v -> finish());

        // Botón guardar
        btnSaveSettings.setOnClickListener(v -> saveSettings());
    }

    // Guarda los valores en SharedPreferences
    private void saveSettings() {
        String weight = txtWeight.getText() != null ? txtWeight.getText().toString().trim() : "";
        String wheel = txtWheel.getText() != null ? txtWheel.getText().toString().trim() : "";

        if (weight.isEmpty() || wheel.isEmpty()) {
            Toast.makeText(this, getString(R.string.error_empty_fields), Toast.LENGTH_SHORT).show();
            return;
        }

        // Guardar en SharedPreferences
        SharedPreferences.Editor editor = getSharedPreferences("UserSettings", MODE_PRIVATE).edit();
        editor.putString("weight", weight);
        editor.putString("wheel", wheel);
        editor.apply();

        Toast.makeText(this, getString(R.string.msg_saved), Toast.LENGTH_SHORT).show();
        finish();
    }
}
