#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"

// ---- Modell paraméterek (K karakterisztika illesztve) ----
static const float R_eq     = 5.5f;     // ohm, ekvivalens ellenállás
static const float R_th     = 1.0f;     // K/W, termikus ellenállás
static const float C_th     = 47.09f;    // J/K, termikus kapacitás
//static const float T_trip   = 150.0f;   // °C, kioldási hőmérséklet
//static const float T_reset  = 80.0f;    // °C, visszakapcsolási hőmérséklet

// Névleges áram
//static const float In = 5.0f; // A

// Szimulált állapotváltozók
static float deltaT = 0.0f;  // °C, relatív melegedés
static bool motor_on = true;


// Bimetál szimulációs task
bool motor_protect_func(float I,float T_trip,float T_reset,int looptime_ms) {
    const float dt = looptime_ms/1000; // másodperc, diszkrét időlépés

        // Differenciaegyenlet diszkrét alakja
        float dT = (I*I*R_eq - deltaT / R_th) * (dt / C_th);
        deltaT += dT;

        // Védelem logika
        if (motor_on && deltaT >= T_trip) {
            motor_on = false;
        } else if (!motor_on && deltaT <= T_reset) {
            motor_on = true;
        }

        ESP_LOGI("DEBUG_TASK", "I=%.2f A, ΔT=%.1f °C, motor=%d, T_trip:%f, T_reset:%f", I, deltaT, motor_on,T_trip,T_reset);
        return (motor_on);
    }
