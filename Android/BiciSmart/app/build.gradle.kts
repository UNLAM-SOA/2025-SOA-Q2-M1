plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.example.biciinteligenteapp"
    compileSdk = 36

    defaultConfig {
        applicationId = "com.example.biciinteligenteapp"
        minSdk = 31
        targetSdk = 36
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    // ✅ Compatibilidad con Java 11
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    // ✅ (Opcional, muy útil) Para usar ViewBinding
    buildFeatures {
        viewBinding = true
    }
}

dependencies {
    // 🟢 Cliente MQTT (antiguo pero estable con AndroidX patch)
    implementation("org.eclipse.paho:org.eclipse.paho.client.mqttv3:1.2.5")
    implementation("org.eclipse.paho:org.eclipse.paho.android.service:1.1.1")

    // 🩵 FIX: Añade LocalBroadcastManager compatible con AndroidX
    implementation("androidx.localbroadcastmanager:localbroadcastmanager:1.1.0")

    // 🧩 Librerías AndroidX y Material Design
    implementation(libs.appcompat)
    implementation(libs.material)
    implementation(libs.activity)
    implementation(libs.constraintlayout)

    // 🚀 Navegación (si la usás)
    implementation(libs.navigation.fragment)
    implementation(libs.navigation.ui)

    // 🧪 Pruebas
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)
    androidTestImplementation(libs.espresso.core)
}
