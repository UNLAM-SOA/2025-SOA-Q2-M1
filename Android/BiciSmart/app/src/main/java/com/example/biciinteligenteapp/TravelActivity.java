package com.example.biciinteligenteapp;

import android.animation.ObjectAnimator;
import android.annotation.SuppressLint;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.ContextCompat;

import com.google.android.material.button.MaterialButton;

import android.Manifest;
import android.content.pm.PackageManager;
import android.location.Location;

import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.Polyline;
import org.osmdroid.views.overlay.mylocation.GpsMyLocationProvider;
import org.osmdroid.views.overlay.mylocation.IMyLocationProvider;
import org.osmdroid.views.overlay.mylocation.MyLocationNewOverlay;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class TravelActivity extends AppCompatActivity {
    private MqttHandler mqttHandler;
    private TextView tvTimer, tvDistance;
    public IntentFilter filterReceive;
    public IntentFilter filterConncetionLost;
    private final messageReceiver receiver = new messageReceiver();
    private final ConnectionLost connectionLost = new ConnectionLost();

    private static final String TAG = "TravelActivity";

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

    private double distanceKm = 0.0;
    private double wheelInches = 0.0;
    private double userWeightKg = 0.0;

    private MapView mapView;
    private MyLocationNewOverlay myLocationOverlay;
    private Polyline routeLine;
    private final List<GeoPoint> routePoints = new ArrayList<>();

    private final ScheduledExecutorService mapExecutor =
            Executors.newSingleThreadScheduledExecutor();

    private final BlockingQueue<GeoPoint> pendingPoints =
            new LinkedBlockingQueue<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_travel);

        mqttHandler = MqttHandler.getInstance(this);

        tvTimer = findViewById(R.id.tvTimer);
        tvDistance = findViewById(R.id.tvDistance);
        ImageView imgCyclist = findViewById(R.id.imgCyclist);
        MaterialButton btnEndTrip = findViewById(R.id.btnEndTrip);

        ObjectAnimator anim = ObjectAnimator.ofFloat(imgCyclist, "translationY", 0f, -15f, 0f);
        anim.setDuration(1600);
        anim.setRepeatCount(ObjectAnimator.INFINITE);
        anim.start();

        loadUserSettings();

        mapView = findViewById(R.id.mapView);
        if (mapView != null) {
            mapView.setMultiTouchControls(true);
            mapView.getController().setZoom(17.0);
            mapView.setTileSource(org.osmdroid.tileprovider.tilesource.TileSourceFactory.MAPNIK);

            routeLine = new Polyline();
            //noinspection deprecation
            routeLine.setWidth(8f);
            //noinspection deprecation
            routeLine.setColor(ContextCompat.getColor(this, android.R.color.holo_blue_dark));
            mapView.getOverlayManager().add(routeLine);

            ActivityResultLauncher<String> requestFinePermission = registerForActivityResult(
                    new ActivityResultContracts.RequestPermission(),
                    granted -> {
                        if (granted) enableMyLocationAndCenter();
                    });

            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    == PackageManager.PERMISSION_GRANTED) {
                enableMyLocationAndCenter();
            } else {
                requestFinePermission.launch(Manifest.permission.ACCESS_FINE_LOCATION);
            }
        }

        startMillis = System.currentTimeMillis();
        handler.post(tick);

        btnEndTrip.setOnClickListener(v -> finishTrip());

        setupBroadcastReceiver();

        Log.d(TAG, "TravelActivity iniciada. Rodado: " + wheelInches + " pulgadas, Peso: " + userWeightKg + " kg");
    }

    private void enableMyLocationAndCenter() {
        if (mapView == null) return;

        GpsMyLocationProvider gpsProvider = new GpsMyLocationProvider(this);
        gpsProvider.setLocationUpdateMinTime(2000);
        gpsProvider.setLocationUpdateMinDistance(2f);

        // Captura puntos sin cargar UI
        myLocationOverlay = new MyLocationNewOverlay(gpsProvider, mapView) {
            @Override
            public void onLocationChanged(final Location location, final IMyLocationProvider source) {
                super.onLocationChanged(location, source);
                if (location != null) {
                    pendingPoints.add(new GeoPoint(location.getLatitude(), location.getLongitude()));
                }
            }
        };

        if (!mapView.getOverlays().contains(myLocationOverlay)) {
            mapView.getOverlays().add(myLocationOverlay);
        }

        myLocationOverlay.enableMyLocation();
        myLocationOverlay.enableFollowLocation();

        myLocationOverlay.runOnFirstFix(() -> runOnUiThread(() -> {
            GeoPoint me = myLocationOverlay.getMyLocation();
            if (me != null) mapView.getController().animateTo(me);
        }));
    }

    private void startBackgroundMapProcessor() {
        mapExecutor.scheduleWithFixedDelay(() -> {
            List<GeoPoint> batch = new ArrayList<>();
            pendingPoints.drainTo(batch);

            if (batch.isEmpty()) return;

            synchronized (routePoints) {
                routePoints.addAll(batch);
            }

            runOnUiThread(() -> {
                synchronized (routePoints) {
                    routeLine.setPoints(new ArrayList<>(routePoints));
                }
                if (mapView != null) mapView.invalidate();
            });

        }, 0, 1, TimeUnit.SECONDS);
    }

    private void loadUserSettings() {
        SharedPreferences prefs = getSharedPreferences("UserSettings", MODE_PRIVATE);
        String wheel = prefs.getString("wheel", "");
        String weight = prefs.getString("weight", "");

        wheelInches = Double.parseDouble(wheel.replace(",", "."));
        userWeightKg = Double.parseDouble(weight.replace(",", "."));

    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    private void setupBroadcastReceiver() {
        filterReceive = new IntentFilter(MqttHandler.ACTION_DATA_RECEIVE);
        filterConncetionLost = new IntentFilter(MqttHandler.ACTION_CONNECTION_LOST);

        filterReceive.addCategory(Intent.CATEGORY_DEFAULT);
        filterConncetionLost.addCategory(Intent.CATEGORY_DEFAULT);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(receiver, filterReceive, Context.RECEIVER_EXPORTED);
            registerReceiver(connectionLost, filterConncetionLost, Context.RECEIVER_EXPORTED);
        } else {
            registerReceiver(receiver, filterReceive);
            registerReceiver(connectionLost, filterConncetionLost);
        }
    }

    public void onNewMessage(double wheel_turn_count) {
        double circumferenceMeters = inchesToMeters(wheelInches) * Math.PI;
        distanceKm = (circumferenceMeters * wheel_turn_count) / 1000.0;

        runOnUiThread(() ->
                tvDistance.setText(String.format(Locale.getDefault(), "Distancia: %.2f km", distanceKm))
        );
    }

    private double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private String formatElapsed(long ms) {
        long totalSec = ms / 1000;
        long h = totalSec / 3600;
        long m = (totalSec % 3600) / 60;
        long s = totalSec % 60;
        return String.format(Locale.US, "%02d:%02d:%02d", h, m, s);
    }

    private void finishTrip() {
        handler.removeCallbacks(tick);
        mqttHandler.publish(ConfigMQTT.topicControl, "{\"trips\":0}");

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
        }
    }

    public class messageReceiver extends BroadcastReceiver {
        public void onReceive(Context context, Intent intent) {
            double msg = intent.getDoubleExtra("msg",0);
            onNewMessage(msg);
        }
    }

    @Override protected void onResume() {
        super.onResume();
        if (mapView != null) mapView.onResume();
        handler.post(tick);
        startBackgroundMapProcessor();
    }

    @Override protected void onPause() {
        super.onPause();
        if (mapView != null) mapView.onPause();
        handler.removeCallbacks(tick);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        handler.removeCallbacks(tick);
        try {
            unregisterReceiver(receiver);
            unregisterReceiver(connectionLost);
        } catch (Exception ignored) {}

        mapExecutor.shutdownNow();
    }
}
