package com.example.biciinteligenteapp;

import android.content.Intent;
import android.os.Bundle;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.button.MaterialButton;

import java.util.Locale;

public class SummaryActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_summary);

        // Referencias UI
        TextView tvTime = findViewById(R.id.tvTime);
        TextView tvDistance = findViewById(R.id.tvDistance);
        TextView tvCalories = findViewById(R.id.tvCalories);
        MaterialButton btnBackToHome = findViewById(R.id.btnBackToHome);

        // Datos recibidos del viaje
        double distanceKm = getIntent().getDoubleExtra("distanceKm", 0.0);
        long durationMs = getIntent().getLongExtra("durationMs", 0L);
        double weightKg = getIntent().getDoubleExtra("weightKg", 0.0);

        // Formatear datos
        String timeFormatted = formatElapsed(durationMs);
        String distanceFormatted = String.format(Locale.US, "%.2f km", distanceKm);
        String caloriesFormatted = String.format(Locale.US, "%.0f kcal", estimateCalories(distanceKm, durationMs, weightKg));

        tvTime.setText(getString(R.string.summary_time_label, timeFormatted));
        tvDistance.setText(getString(R.string.summary_distance_label, distanceFormatted));
        tvCalories.setText(getString(R.string.summary_calories_label, caloriesFormatted));

        // Botón volver al inicio
        btnBackToHome.setOnClickListener(v -> {
            Intent intent = new Intent(this, BikeStatusActivity.class);
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(intent);
            finish();
        });
    }

    // Estima las calorías quemadas (valor aproximado basado en peso y distancia)
    private double estimateCalories(double distanceKm, double durationMs, double weightKg) {
        if (distanceKm <= 0 || durationMs <= 0 || weightKg <= 0) {
            return 0.0;
        }

        // Duración en horas
        double hours = durationMs / 3600000.0;

        // Velocidad promedio en km/h
        double avgSpeed = distanceKm / hours;

        // Determinar MET según velocidad
        double met;
        if (avgSpeed < 10.0) {
            met = 3.5;
        } else if (avgSpeed < 12.0) {
            met = 4.0;
        } else if (avgSpeed < 14.0) {
            met = 6.0;
        } else if (avgSpeed < 16.0) {
            met = 8.0;
        } else if (avgSpeed < 19.0) {
            met = 10.0;
        } else if (avgSpeed < 22.0) {
            met = 12.0;
        } else if (avgSpeed < 25.0) {
            met = 14.0;
        } else if (avgSpeed < 30.0) {
            met = 16.0;
        } else {
            met = 18.0;
        }

        // Cálculo final de calorías quemadas
        return met * weightKg * hours;
    }


    // Formatea el tiempo transcurrido
    private String formatElapsed(long ms) {
        long totalSec = ms / 1000;
        long h = totalSec / 3600;
        long m = (totalSec % 3600) / 60;
        long s = totalSec % 60;
        return String.format(Locale.US, "%02d:%02d:%02d", h, m, s);
    }
}
