#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include "pump_protection.h"



// Névleges áram
//static const float In = 5.0f; // A

// Szimulált állapotváltozók
static float deltaT = 0.0f;  // °C, relatív melegedés
static bool motor_prot_on = true;


// Bimetál szimulációs task
bool motor_protect_func(float I,float T_trip,float T_reset,int looptime_ms,bool pumpstatus,float *T_max) {
    const float dt = looptime_ms/1000; // másodperc, diszkrét időlépés

        // Differenciaegyenlet diszkrét alakja
        float dT = (I*I*R_eq - deltaT / R_th) * (dt / C_th);
        deltaT += dT;
        if (deltaT>*T_max) *T_max=deltaT;

        // Védelem logika
        if (motor_prot_on && deltaT >= T_trip) {
            motor_prot_on = false;
        } else if (!motor_prot_on && deltaT <= T_reset) {
            motor_prot_on = true;
        }

        if (pumpstatus) ESP_LOGI("DEBUG_TASK", "I=%.2f A, ΔT=%.1f °C, motor=%d, T_trip:%f, T_reset:%f", I, deltaT, motor_prot_on,T_trip,T_reset);
        return (motor_prot_on);
    }
