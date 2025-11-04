package com.example.biciinteligenteapp;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;

import com.google.android.material.textfield.TextInputEditText;
import com.google.android.material.textfield.TextInputLayout;

public class LoginActivity extends AppCompatActivity {

    // --- Constantes de credenciales (hardcodeadas) --- ADMIN ADMIN
    private static final String VALID_USER = "admin";
    private static final String VALID_PASS = "admin";

    // --- Referencias UI ---
    private TextInputLayout tilUser, tilPass;
    private TextInputEditText txtUser, txtPass;
    private Button btnLogin;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_login);

        // Inicialización de vistas
        tilUser = findViewById(R.id.tilUser);
        tilPass = findViewById(R.id.tilPass);
        txtUser = findViewById(R.id.txtUser);
        txtPass = findViewById(R.id.txtPass);
        btnLogin = findViewById(R.id.btnLogin);

        // Acción de teclado "done" en el campo contraseña
        txtPass.setOnEditorActionListener((v, actionId, event) -> {
            if (actionId == EditorInfo.IME_ACTION_DONE) {
                attemptLogin();
                return true;
            }
            return false;
        });

        // Click en botón login
        btnLogin.setOnClickListener(v -> attemptLogin());
    }

    // Valida las credenciales y gestiona el inicio de sesióN
    private void attemptLogin() {
        // Ocultar teclado
        View view = this.getCurrentFocus();
        if (view != null) {
            InputMethodManager imm = (InputMethodManager) getSystemService(INPUT_METHOD_SERVICE);
            imm.hideSoftInputFromWindow(view.getWindowToken(), 0);
        }

        // Limpiar errores previos
        tilUser.setError(null);
        tilPass.setError(null);

        String user = txtUser.getText() != null ? txtUser.getText().toString().trim() : "";
        String pass = txtPass.getText() != null ? txtPass.getText().toString().trim() : "";

        // Validaciones
        if (user.isEmpty()) {
            tilUser.setError("Ingrese el usuario");
            return;
        }
        if (pass.isEmpty()) {
            tilPass.setError("Ingrese la contraseña");
            return;
        }

        // Evitar doble clic rápido
        btnLogin.setEnabled(false);
        new Handler(Looper.getMainLooper()).postDelayed(() -> btnLogin.setEnabled(true), 1000);

        // Verificar credenciales
        if (user.equals(VALID_USER) && pass.equals(VALID_PASS)) {
            Toast.makeText(this, "¡Bienvenido!", Toast.LENGTH_SHORT).show();

            // Ir a la siguiente pantalla
            Intent intent = new Intent(this, BikeStatusActivity.class);
            startActivity(intent);
            finish(); // Evita volver atrás
        } else {
            tilUser.setError("Usuario o contraseña incorrectos");
            tilPass.setError("Usuario o contraseña incorrectos");
            Toast.makeText(this, "Credenciales incorrectas", Toast.LENGTH_SHORT).show();
        }
    }
}
