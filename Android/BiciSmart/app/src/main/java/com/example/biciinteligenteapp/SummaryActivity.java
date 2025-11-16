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
        String caloriesFormatted = String.format(Locale.US, "%.0f kcal", estimateCalories(distanceKm, weightKg));

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
    private double estimateCalories(double distanceKm, double weightKg) {
        // Fórmula base: 35 kcal/km para 70kg, ajustado por peso
        double baseRate = 35.0 * (weightKg / 70.0);
        return distanceKm * baseRate;
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
