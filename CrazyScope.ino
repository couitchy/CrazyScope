/*
 * ==================================================================================================
 *  Crazy Scope - Mini oscilloscope Wi-Fi
 * ==================================================================================================
 *   - ESP32 DEV MODULE (ESP32-WROOM-32)
 *   - CH1 sur GPIO34 (ADC1_CH6) - entree analogique 0..3.3V
 *   - CH2 sur GPIO35 (ADC1_CH7) - entree analogique 0..3.3V
 *   - Etage analogique à configurer dans l'interface d'administration
 *
 *  Bibliothèques requises sous Arduino IDE :
 *    - ArduinoJson         (Benoit Blanchon)
 *    - ESP Async WebServer (ESP32Async)
 *    - Async TCP           (ESP32Async)
 *
 *  Partition Scheme : "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)"
 *
 *  Auteur : Couitchy pour l'architecture, Opus 4.8 pour l'implémentation
 * ==================================================================================================
 */
 
#include <WiFi.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Update.h>

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/dac_cosine.h"

// ============================================================================
//  CONFIGURATION MATERIELLE
// ============================================================================

#define LED_PIN              2
#define ADC_CH1_CHANNEL      ADC_CHANNEL_6   // GPIO34
#define ADC_CH2_CHANNEL      ADC_CHANNEL_7   // GPIO35
#define ADC_UNIT_USED        ADC_UNIT_1
#define ADC_ATTEN            ADC_ATTEN_DB_12 // pleine echelle 0..3.3V
#define ADC_BIT_WIDTH        ADC_BITWIDTH_12

// ============================================================================
//  RESEAU
// ============================================================================

#define AP_SSID_PREFIX       "CrazyScope-"
#define AP_PASSWORD          ""              // AP ouvert par defaut
#define MDNS_NAME            "crazyscope"    // -> http://crazyscope.local
#define STA_CONNECT_TIMEOUT  15000           // ms

// ============================================================================
//  ACQUISITION
// ============================================================================

// Throttle d'envoi : la pile Wi-Fi+AsyncTCP de l'ESP32 sature autour de
// 8-10 paquets/sec, peu importe la taille. On envoie donc moins de paquets,
// mais plus gros (samples_per_frame jusqu'a 1024).
#define FRAME_THROTTLE_MS 100

// Bornes physiques du generateur DAC cosine (horloge 8 MHz / 65536 par step)
// Fmin theorique ~122 Hz (step=1), Fmax pratique limitee au filtre de sortie.
#define TEST_SIG_FREQ_MIN  130
#define TEST_SIG_FREQ_MAX  100000

// Buffer DMA : doit etre un multiple de SOC_ADC_DIGI_DATA_BYTES_PER_CONV (4)
// On taille large pour absorber les jitters Wi-Fi
// Buffers DMA de l'ADC continu.
// max_store_buf_size : taille du buffer circulaire interne. Doit etre assez
// grand pour absorber les latences quand le Wi-Fi sature brievement le CPU
// (sinon overrun -> samples perdus -> glitchs visibles dans la trace).
// A 800 kHz x 4 octets/sample = 3.2 MB/s, 32 ko = ~10 ms de marge.
#define DMA_RING_BUFFER_SIZE 32768
#define DMA_FRAME_SIZE       2048

// Nombre maxi de samples effectifs envoyes par canal et par trame WebSocket
// 1024 permet d'avoir ~10 ms de signal a 100 kHz dans une trame, soit
// 10 cycles complets a 1 kHz - parfait pour visualisation.
#define MAX_SAMPLES_PER_FRAME 1024

// Presets de qualite d'acquisition (cf. table dans le README)
// Chaque preset definit le Fs *effectif par canal* et l'oversampling associe.
// Le Fs ADC physique total est = Fs_eff * OS * nb_canaux_actifs.
// On reste sous ~800 kHz pour garder de la marge avec Wi-Fi actif.
struct QualityPreset {
    const char* name;
    uint32_t fs_effective_hz; // Fs sortant par canal
    uint8_t  os_2ch;          // oversampling si 2 voies actives
    uint8_t  os_1ch;          // oversampling si 1 seule voie
};

static const QualityPreset PRESETS[] = {
    { "speed",     100000,  4,  8  }, // Vitesse  : bande utile 50 kHz
    { "standard",   50000,  8, 16  }, // Standard : bande utile 25 kHz, meilleur SNR
    { "precision",  25000, 16, 32  }, // Precision: bande utile 12 kHz, SNR x4..x5
    { "fastest",   500000,  1,  1  }  // HF mono-canal : bande utile 250 kHz
};
#define NB_PRESETS (sizeof(PRESETS)/sizeof(PRESETS[0]))

// ============================================================================
//  ETAT GLOBAL
// ============================================================================

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

adc_continuous_handle_t adc_handle = NULL;
adc_cali_handle_t       cali_handle = NULL;
bool                    cali_enabled = false;

// Configuration courante (persistee dans /config.json)
struct AppConfig {
    String   wifi_ssid;
    String   wifi_password;
    uint8_t  quality_idx = 0;       // index dans PRESETS[]
    bool     ch1_enabled = true;
    bool     ch2_enabled = true;
    uint16_t samples_per_frame = 256;
    uint16_t decimation = 1;        // facteur de decimation temporelle
                                    // (base de temps lente : moyenne plus de
                                    //  samples ADC par point affiche)
    // Etage analogique entre V_sig et l'ADC :
    //   V_sig --[Rdiv_sig]-- ADC --[Rdiv_gnd]-- GND
    //                         |
    //                       [Roffset]
    //                         |
    //                       Voffset (ex: 3.3V)
    // Convention de stockage:  > 0  -> valeur en kOhm
    //                         == 0  -> court-circuit (R=0)
    //                          < 0  -> circuit ouvert (R=infini)
    // Defaults : pas de front-end analogique (mesure brute 0..3.3V via
    // entree directe). Pour un test de base, l'utilisateur peut connecter
    // directement un signal entre 0 et 3.3V sur GPIO34/GPIO35.
    float    r_div_sig_kohm =  0.0f;   // court-circuit (entree directe)
    float    r_div_gnd_kohm = -1.0f;   // circuit ouvert
    float    r_offset_kohm  = -1.0f;   // circuit ouvert
    float    v_offset_v     =  3.3f;
    // Generateur de signal de test (sinusoide via le DAC cosine du SoC)
    // GPIO25 (DAC_CHAN_0). Borne par TEST_SIG_FREQ_MIN/MAX (cf constantes)
    bool     test_sig_enabled = false;
    uint32_t test_sig_freq_hz = 1000;
    String   ui_state = "{}";       // etat UI brut, opaque pour le firmware
};
AppConfig cfg;

// Etat acquisition (modifie depuis WS, lu depuis la task d'acquisition)
volatile bool g_adc_running       = false;
volatile bool g_reconfigure_adc   = false;
volatile bool g_client_connected  = false;
// Drapeaux pour reporter les ecritures LittleFS hors du contexte callback WS
// (sinon on gele la pile AsyncTCP sur Core 0 pendant l'ecriture flash et
//  les clients se font kicker sur timeout).
volatile bool g_save_config_pending  = false;
// Restart differe : on ne redemarre pas dans le callback HTTP (sinon la
// reponse n'a pas le temps de partir et le client croit a une erreur).
volatile bool     g_restart_pending = false;
volatile uint32_t g_restart_at_ms   = 0;

// Statistiques calculees a chaque trame (pour overlay UI)
struct FrameStats {
    uint32_t fs_effective_hz;
    uint8_t  oversampling;
    uint16_t nb_samples;
    uint8_t  flags; // bit0=CH1, bit1=CH2
};

TaskHandle_t acquisitionTaskHandle = NULL;

// ============================================================================
//  PROTOTYPES
// ============================================================================

void loadConfig();
void saveConfig();

bool startWifiStation();
void startWifiAP();
void setupMDNS();

void setupWebServer();
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len);
void handleWsCommand(AsyncWebSocketClient* client, uint8_t* data, size_t len);

bool initAdcContinuous();
void deinitAdcContinuous();
void acquisitionTask(void* arg);
void sendBinaryFrame(const uint16_t* ch1, const uint16_t* ch2,
                     const FrameStats& stats);

// ============================================================================
//  setup()
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("==== Crazy Scope - boot ===="));

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // --- LittleFS ----------------------------------------------------------
    if (!LittleFS.begin(true)) {
        Serial.println(F("[FS] Echec montage LittleFS"));
    } else {
        Serial.printf("[FS] LittleFS monte (%u / %u octets)\n",
                      (unsigned)LittleFS.usedBytes(),
                      (unsigned)LittleFS.totalBytes());
    }
    loadConfig();

    // --- Wi-Fi -------------------------------------------------------------
    bool sta_ok = false;
    if (cfg.wifi_ssid.length() > 0) {
        sta_ok = startWifiStation();
    }
    if (!sta_ok) {
        Serial.println(F("[WiFi] Bascule en mode AP"));
        startWifiAP();
    } else {
        digitalWrite(LED_PIN, HIGH); // STA OK : LED fixe
    }
    setupMDNS();

    // --- Calibration ADC ---------------------------------------------------
    // ESP32 classique (WROOM-32) : seul le line-fitting est disponible.
    // Curve-fitting est reserve aux puces recentes (S3, C3...).
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_USED,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle)
        == ESP_OK) {
        cali_enabled = true;
        Serial.println(F("[ADC] Calibration line-fitting OK"));
    } else {
        Serial.println(F("[ADC] Calibration indispo (eFuses absents ?)"));
    }

    // --- Web server / WebSocket -------------------------------------------
    setupWebServer();

    // --- Generateur DAC cosinus (signal de test) --------------------------
    applyTestSignal();

    // --- Task acquisition (Core 1) ----------------------------------------
    // Task acquisition (Core 1, priorite elevee pour limiter la preemption
    // par le scheduler quand le Wi-Fi est actif sur Core 0)
    xTaskCreatePinnedToCore(
        acquisitionTask, "acq", 8192, NULL, 10,
        &acquisitionTaskHandle, 1);

    Serial.println(F("==== Boot termine ===="));
}

// ============================================================================
//  loop()
// ============================================================================

void loop() {
    // Tout est event-driven (Async). On en profite pour faire des taches
    // legeres : nettoyage WS, sauvegardes differees, blink LED si AP.
    ws.cleanupClients();

    // Ecritures LittleFS deportees ici (Core 1 par defaut pour Arduino loop),
    // pour ne PAS bloquer la pile AsyncTCP qui tourne sur Core 0.
    if (g_save_config_pending) {
        g_save_config_pending = false;
        saveConfig();
    }

    // Redemarrage differe (laisse le temps a la reponse HTTP de partir)
    if (g_restart_pending && (int32_t)(millis() - g_restart_at_ms) >= 0) {
        Serial.println(F("[SYS] Redemarrage..."));
        delay(100);
        ESP.restart();
    }

    static uint32_t last_blink = 0;
    if (WiFi.getMode() & WIFI_AP) {
        if (millis() - last_blink > 500) {
            last_blink = millis();
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    }
    delay(10);
}

// ============================================================================
//  CONFIG (LittleFS /config.json)
// ============================================================================

void loadConfig() {
    // ----- /config.json : parametres systeme -----------------------------
    if (LittleFS.exists("/config.json")) {
        File f = LittleFS.open("/config.json", "r");
        if (f) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, f);
            f.close();
            if (!err) {
                cfg.wifi_ssid         = (const char*)(doc["wifi_ssid"]     | "");
                cfg.wifi_password     = (const char*)(doc["wifi_password"] | "");
                cfg.quality_idx       = doc["quality_idx"]       | 0;
                cfg.ch1_enabled       = doc["ch1_enabled"]       | true;
                cfg.ch2_enabled       = doc["ch2_enabled"]       | true;
                cfg.samples_per_frame = doc["samples_per_frame"] | 256;
                cfg.decimation        = doc["decimation"]        | 1;
                if (cfg.decimation < 1) cfg.decimation = 1;
                // Etage analogique (defaults conformes au schema kit ESP32)
                cfg.r_div_sig_kohm = doc["r_div_sig_kohm"] |  0.0f;
                cfg.r_div_gnd_kohm = doc["r_div_gnd_kohm"] | -1.0f;
                cfg.r_offset_kohm  = doc["r_offset_kohm"]  | -1.0f;
                cfg.v_offset_v     = doc["v_offset_v"]     |  3.3f;
                cfg.test_sig_enabled = doc["test_sig_enabled"] | false;
                cfg.test_sig_freq_hz = doc["test_sig_freq_hz"] | 1000;
                if (cfg.quality_idx >= NB_PRESETS) cfg.quality_idx = 0;
                if (cfg.samples_per_frame > MAX_SAMPLES_PER_FRAME)
                    cfg.samples_per_frame = MAX_SAMPLES_PER_FRAME;
                // ui_state : extraction puis re-serialisation en String brute
                JsonVariant ui = doc["ui_state"];
                if (!ui.isNull()) {
                    String s;
                    serializeJson(ui, s);
                    cfg.ui_state = s;
                }
            } else {
                Serial.printf("[CFG] Erreur JSON config.json : %s\n", err.c_str());
            }
        }
    } else {
        Serial.println(F("[CFG] Aucun /config.json, valeurs par defaut"));
    }

    if (cfg.ui_state.length() == 0) cfg.ui_state = "{}";

    Serial.printf("[CFG] charge : SSID='%s' quality=%s CH1=%d CH2=%d\n",
                  cfg.wifi_ssid.c_str(),
                  PRESETS[cfg.quality_idx].name,
                  cfg.ch1_enabled, cfg.ch2_enabled);
}

void saveConfig() {
    // Construction du JSON avec un placeholder pour ui_state, puis
    // substitution brute (contourne le bug ArduinoJson v7 sur les
    // sous-documents qui ne sont pas correctement deep-copies).
    JsonDocument doc;
    doc["wifi_ssid"]         = cfg.wifi_ssid;
    doc["wifi_password"]     = cfg.wifi_password;
    doc["quality_idx"]       = cfg.quality_idx;
    doc["ch1_enabled"]       = cfg.ch1_enabled;
    doc["ch2_enabled"]       = cfg.ch2_enabled;
    doc["samples_per_frame"] = cfg.samples_per_frame;
    doc["decimation"]        = cfg.decimation;
    doc["r_div_sig_kohm"]    = cfg.r_div_sig_kohm;
    doc["r_div_gnd_kohm"]    = cfg.r_div_gnd_kohm;
    doc["r_offset_kohm"]     = cfg.r_offset_kohm;
    doc["v_offset_v"]        = cfg.v_offset_v;
    doc["test_sig_enabled"]  = cfg.test_sig_enabled;
    doc["test_sig_freq_hz"]  = cfg.test_sig_freq_hz;
    doc["ui_state"]          = "@@UI_STATE_PLACEHOLDER@@";

    String out;
    if (serializeJson(doc, out) == 0) {
        Serial.println(F("[CFG] Echec serialisation /config.json"));
        return;
    }
    out.replace("\"@@UI_STATE_PLACEHOLDER@@\"",
                cfg.ui_state.length() > 0 ? cfg.ui_state : String("{}"));

    File f = LittleFS.open("/config.json", "w");
    if (!f) {
        Serial.println(F("[CFG] Impossible d'ouvrir /config.json"));
        return;
    }
    f.print(out);
    f.close();
}

// ============================================================================
//  GENERATEUR DE SIGNAL DE TEST (DAC cosinus)
// ============================================================================
//
// L'ESP32 dispose de 2 DAC 8 bits (GPIO25 = DAC_CHAN_0, GPIO26 = DAC_CHAN_1)
// avec un generateur cosinus integre, autonome (aucun CPU une fois lance).
// On utilise GPIO25. L'utilisateur peut relier ce GPIO a l'entree de CH1 ou
// CH2 (avant le pont diviseur, cote Vsig) pour verifier l'oscilloscope avec
// un signal connu.
//
// Note materielle : le generateur cosinus partage son horloge entre les
// deux canaux DAC. Comme on n'utilise qu'un canal, aucun conflit.

static dac_cosine_handle_t g_dac_handle = NULL;

void stopTestSignal() {
    if (g_dac_handle != NULL) {
        dac_cosine_stop(g_dac_handle);
        dac_cosine_del_channel(g_dac_handle);
        g_dac_handle = NULL;
    }
}

void startTestSignal(uint32_t freq_hz) {
    stopTestSignal();
    dac_cosine_config_t c = {};
    c.chan_id = DAC_CHAN_0;                     // GPIO25
    c.freq_hz = freq_hz;
    c.clk_src = DAC_COSINE_CLK_SRC_DEFAULT;     // RTC_8M (8 MHz)
    c.atten   = DAC_COSINE_ATTEN_DEFAULT;       // 0 dB -> amplitude max
    c.phase   = DAC_COSINE_PHASE_0;
    c.offset  = 0;
    c.flags.force_set_freq = false;
    esp_err_t r = dac_cosine_new_channel(&c, &g_dac_handle);
    if (r != ESP_OK) {
        Serial.printf("[DAC] dac_cosine_new_channel echec : %s\n",
                      esp_err_to_name(r));
        g_dac_handle = NULL;
        return;
    }
    r = dac_cosine_start(g_dac_handle);
    if (r != ESP_OK) {
        Serial.printf("[DAC] dac_cosine_start echec : %s\n",
                      esp_err_to_name(r));
        dac_cosine_del_channel(g_dac_handle);
        g_dac_handle = NULL;
        return;
    }
    Serial.printf("[DAC] signal de test : %u Hz sur GPIO25\n",
                  (unsigned)freq_hz);
}

void applyTestSignal() {
    // Borne defensive
    if (cfg.test_sig_freq_hz < TEST_SIG_FREQ_MIN)
        cfg.test_sig_freq_hz = TEST_SIG_FREQ_MIN;
    if (cfg.test_sig_freq_hz > TEST_SIG_FREQ_MAX)
        cfg.test_sig_freq_hz = TEST_SIG_FREQ_MAX;
    if (cfg.test_sig_enabled) startTestSignal(cfg.test_sig_freq_hz);
    else                      stopTestSignal();
}

// ============================================================================
//  WI-FI
// ============================================================================

bool startWifiStation() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MDNS_NAME);
    WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_password.c_str());

    Serial.printf("[WiFi] Connexion a '%s'", cfg.wifi_ssid.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - t0 < STA_CONNECT_TIMEOUT) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] STA OK, IP = %s\n",
                      WiFi.localIP().toString().c_str());
        return true;
    }
    Serial.println(F("[WiFi] Echec connexion STA"));
    return false;
}

void startWifiAP() {
    WiFi.mode(WIFI_AP);
    // softAPmacAddress() fonctionne meme avant softAP() et donne le bon MAC
    // (contrairement a WiFi.macAddress() qui peut retourner 00:00:... si STA
    //  n'a pas encore ete initialise).
    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    char ssid[32];
    snprintf(ssid, sizeof(ssid), "%s%02X%02X",
             AP_SSID_PREFIX, mac[4], mac[5]);
    WiFi.softAP(ssid, strlen(AP_PASSWORD) > 0 ? AP_PASSWORD : NULL);
    Serial.printf("[WiFi] AP '%s' demarre, IP = %s\n",
                  ssid,
                  WiFi.softAPIP().toString().c_str());
}

void setupMDNS() {
    if (MDNS.begin(MDNS_NAME)) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[mDNS] http://%s.local\n", MDNS_NAME);
    } else {
        Serial.println(F("[mDNS] echec"));
    }
}

// ============================================================================
//  WEB SERVER
// ============================================================================

void setupWebServer() {
    // --- WebSocket --------------------------------------------------------
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // --- Pages statiques --------------------------------------------------
    server.serveStatic("/", LittleFS, "/")
          .setDefaultFile("index.html");

    // --- API REST : etat / config -----------------------------------------
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* r) {
        JsonDocument d;
        d["mode"]      = (WiFi.getMode() & WIFI_AP) ? "AP" : "STA";
        d["ip"]        = (WiFi.getMode() & WIFI_AP)
                         ? WiFi.softAPIP().toString()
                         : WiFi.localIP().toString();
        d["ssid"]      = (WiFi.getMode() & WIFI_AP)
                         ? WiFi.softAPSSID() : WiFi.SSID();
        d["rssi"]      = WiFi.RSSI();
        d["uptime_ms"] = (uint32_t)millis();
        d["fs_total"]  = (uint32_t)LittleFS.totalBytes();
        d["fs_used"]   = (uint32_t)LittleFS.usedBytes();
        d["heap_free"] = (uint32_t)ESP.getFreeHeap();
        d["cali_ok"]   = cali_enabled;
        d["sketch"]    = "CrazyScope v1.0";
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    server.on("/api/wifi", HTTP_POST,
        [](AsyncWebServerRequest* r) { /* handled in body */ },
        NULL,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len,
           size_t index, size_t total) {
            JsonDocument d;
            if (deserializeJson(d, data, len) != DeserializationError::Ok) {
                r->send(400, "text/plain", "JSON invalide");
                return;
            }
            cfg.wifi_ssid     = (const char*)(d["ssid"]     | "");
            cfg.wifi_password = (const char*)(d["password"] | "");
            g_save_config_pending = true;
            r->send(200, "application/json",
                    "{\"ok\":true,\"msg\":\"Configuration enregistree, "
                    "redemarrage dans 2 s...\"}");
            // Restart differe : la reponse ci-dessus doit pouvoir partir
            // avant le reboot, sinon le client croit a une erreur.
            g_restart_pending = true;
            g_restart_at_ms   = millis() + 2000;
        });

    // --- Liste des reseaux Wi-Fi visibles (pour la page admin) ------------
    server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest* r) {
        int n = WiFi.scanComplete();
        if (n == -2) {
            WiFi.scanNetworks(true);
            r->send(202, "application/json",
                    "{\"status\":\"scanning\"}");
            return;
        }
        if (n == -1) {
            r->send(202, "application/json",
                    "{\"status\":\"scanning\"}");
            return;
        }
        JsonDocument d;
        JsonArray arr = d["networks"].to<JsonArray>();
        for (int i = 0; i < n; ++i) {
            JsonObject o = arr.add<JsonObject>();
            o["ssid"]    = WiFi.SSID(i);
            o["rssi"]    = WiFi.RSSI(i);
            o["secured"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
        WiFi.scanDelete();
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    // --- Etage analogique : lecture des parametres ------------------------
    server.on("/api/analog", HTTP_GET, [](AsyncWebServerRequest* r) {
        JsonDocument d;
        d["r_div_sig_kohm"] = cfg.r_div_sig_kohm;
        d["r_div_gnd_kohm"] = cfg.r_div_gnd_kohm;
        d["r_offset_kohm"]  = cfg.r_offset_kohm;
        d["v_offset_v"]     = cfg.v_offset_v;
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    // --- Etage analogique : ecriture des parametres -----------------------
    // Convention: > 0 -> kOhm ; == 0 -> court-circuit ; < 0 -> circuit ouvert
    server.on("/api/analog", HTTP_POST,
        [](AsyncWebServerRequest* r) { /* handled in body */ },
        NULL,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len,
           size_t /*index*/, size_t /*total*/) {
            JsonDocument d;
            if (deserializeJson(d, data, len) != DeserializationError::Ok) {
                r->send(400, "text/plain", "JSON invalide");
                return;
            }
            if (d["r_div_sig_kohm"].is<float>())
                cfg.r_div_sig_kohm = d["r_div_sig_kohm"];
            if (d["r_div_gnd_kohm"].is<float>())
                cfg.r_div_gnd_kohm = d["r_div_gnd_kohm"];
            if (d["r_offset_kohm"].is<float>())
                cfg.r_offset_kohm  = d["r_offset_kohm"];
            if (d["v_offset_v"].is<float>())
                cfg.v_offset_v     = d["v_offset_v"];
            g_save_config_pending = true;
            // Notification temps reel aux clients oscilloscope connectes
            JsonDocument push;
            push["type"]            = "analog_config";
            push["r_div_sig_kohm"]  = cfg.r_div_sig_kohm;
            push["r_div_gnd_kohm"]  = cfg.r_div_gnd_kohm;
            push["r_offset_kohm"]   = cfg.r_offset_kohm;
            push["v_offset_v"]      = cfg.v_offset_v;
            String s;
            serializeJson(push, s);
            ws.textAll(s);
            r->send(200, "application/json",
                    "{\"ok\":true,\"msg\":\"Etage analogique enregistre\"}");
        });

    // --- Signal de test (DAC cosinus) : lecture ---------------------------
    server.on("/api/testsig", HTTP_GET, [](AsyncWebServerRequest* r) {
        JsonDocument d;
        d["enabled"]  = cfg.test_sig_enabled;
        d["freq_hz"]  = cfg.test_sig_freq_hz;
        d["freq_min"] = TEST_SIG_FREQ_MIN;
        d["freq_max"] = TEST_SIG_FREQ_MAX;
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    // --- Signal de test : ecriture ----------------------------------------
    server.on("/api/testsig", HTTP_POST,
        [](AsyncWebServerRequest* r) { /* handled in body */ },
        NULL,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len,
           size_t /*index*/, size_t /*total*/) {
            JsonDocument d;
            if (deserializeJson(d, data, len) != DeserializationError::Ok) {
                r->send(400, "text/plain", "JSON invalide");
                return;
            }
            if (d["enabled"].is<bool>())
                cfg.test_sig_enabled = d["enabled"];
            if (d["freq_hz"].is<int>()) {
                uint32_t f = d["freq_hz"];
                if (f < TEST_SIG_FREQ_MIN) f = TEST_SIG_FREQ_MIN;
                if (f > TEST_SIG_FREQ_MAX) f = TEST_SIG_FREQ_MAX;
                cfg.test_sig_freq_hz = f;
            }
            applyTestSignal();
            g_save_config_pending = true;
            r->send(200, "application/json",
                    "{\"ok\":true,\"msg\":\"Signal de test mis a jour\"}");
        });

    // --- Upload de fichiers vers LittleFS (page admin) --------------------
    server.on("/api/upload", HTTP_POST,
        [](AsyncWebServerRequest* r) { r->send(200, "text/plain", "OK"); },
        [](AsyncWebServerRequest* r, String filename, size_t index,
           uint8_t* data, size_t len, bool final) {
            static File uf;
            if (!index) {
                if (!filename.startsWith("/")) filename = "/" + filename;
                Serial.printf("[UP] %s\n", filename.c_str());
                uf = LittleFS.open(filename, "w");
            }
            if (uf) uf.write(data, len);
            if (final && uf) uf.close();
        });

    // --- Liste / suppression de fichiers ---------------------------------
    server.on("/api/files", HTTP_GET, [](AsyncWebServerRequest* r) {
        JsonDocument d;
        JsonArray arr = d["files"].to<JsonArray>();
        File root = LittleFS.open("/");
        File f = root.openNextFile();
        while (f) {
            JsonObject o = arr.add<JsonObject>();
            o["name"] = String("/") + f.name();
            o["size"] = (uint32_t)f.size();
            f = root.openNextFile();
        }
        d["total"] = (uint32_t)LittleFS.totalBytes();
        d["used"]  = (uint32_t)LittleFS.usedBytes();
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    server.on("/api/delete", HTTP_POST, [](AsyncWebServerRequest* r) {
        if (!r->hasParam("file", true)) {
            r->send(400, "text/plain", "param 'file' requis");
            return;
        }
        String name = r->getParam("file", true)->value();
        if (!name.startsWith("/")) name = "/" + name;
        if (LittleFS.remove(name)) {
            r->send(200, "application/json", "{\"ok\":true}");
        } else {
            r->send(500, "application/json", "{\"ok\":false}");
        }
    });

    // --- OTA firmware update ---------------------------------------------
    server.on("/api/firmware", HTTP_POST,
        [](AsyncWebServerRequest* r) {
            bool ok = !Update.hasError();
            AsyncWebServerResponse* resp = r->beginResponse(200,
                "application/json",
                ok ? "{\"ok\":true}" : "{\"ok\":false}");
            resp->addHeader("Connection", "close");
            r->send(resp);
            delay(500);
            if (ok) ESP.restart();
        },
        [](AsyncWebServerRequest* r, String fn, size_t idx,
           uint8_t* data, size_t len, bool final) {
            if (!idx) {
                Serial.printf("[OTA] start %s\n", fn.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
            }
            if (Update.write(data, len) != len) Update.printError(Serial);
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] termine, %u octets\n",
                                  (unsigned)(idx + len));
                } else {
                    Update.printError(Serial);
                }
            }
        });

    server.onNotFound([](AsyncWebServerRequest* r) {
        r->send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.println(F("[HTTP] serveur demarre sur :80"));
}

// ============================================================================
//  WEBSOCKET
// ============================================================================

void onWsEvent(AsyncWebSocket* /*s*/, AsyncWebSocketClient* client,
               AwsEventType type, void* /*arg*/, uint8_t* data, size_t len) {
    switch (type) {
    case WS_EVT_CONNECT: {
        Serial.printf("[WS] client #%u connecte (%s)\n", client->id(),
                      client->remoteIP().toString().c_str());
        g_client_connected = true;
        // Construction du hello : ArduinoJson pour la base, puis injection
        // brute de cfg.ui_state (qui est deja du JSON valide).
        JsonDocument d;
        d["type"]              = "hello";
        d["quality_idx"]       = cfg.quality_idx;
        d["ch1_enabled"]       = cfg.ch1_enabled;
        d["ch2_enabled"]       = cfg.ch2_enabled;
        d["samples_per_frame"] = cfg.samples_per_frame;
        d["decimation"]        = cfg.decimation;
        d["r_div_sig_kohm"]    = cfg.r_div_sig_kohm;
        d["r_div_gnd_kohm"]    = cfg.r_div_gnd_kohm;
        d["r_offset_kohm"]     = cfg.r_offset_kohm;
        d["v_offset_v"]        = cfg.v_offset_v;
        d["test_sig_enabled"]  = cfg.test_sig_enabled;
        d["test_sig_freq_hz"]  = cfg.test_sig_freq_hz;
        JsonArray qa = d["presets"].to<JsonArray>();
        for (size_t i = 0; i < NB_PRESETS; ++i) {
            JsonObject o = qa.add<JsonObject>();
            o["name"]   = PRESETS[i].name;
            o["fs"]     = PRESETS[i].fs_effective_hz;
            o["os_2ch"] = PRESETS[i].os_2ch;
            o["os_1ch"] = PRESETS[i].os_1ch;
        }
        d["ui_state"] = "@@UI_STATE_PLACEHOLDER@@";
        String out;
        serializeJson(d, out);
        // Substitution du placeholder par l'objet brut
        out.replace("\"@@UI_STATE_PLACEHOLDER@@\"",
                    cfg.ui_state.length() > 0 ? cfg.ui_state : String("{}"));
        client->text(out);
        break;
    }
    case WS_EVT_DISCONNECT:
        Serial.printf("[WS] client #%u parti\n", client->id());
        if (ws.count() == 0) g_client_connected = false;
        break;
    case WS_EVT_ERROR:
        Serial.printf("[WS] erreur client #%u\n", client->id());
        break;
    case WS_EVT_PONG:
        break;
    case WS_EVT_DATA:
        handleWsCommand(client, data, len);
        break;
    default:
        break;
    }
}

void handleWsCommand(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
    JsonDocument d;
    if (deserializeJson(d, data, len) != DeserializationError::Ok) return;
    const char* type = d["type"] | "";

    if (strcmp(type, "set_config") == 0) {
        bool changed = false;       // declenche la sauvegarde config
        bool need_reconfig = false; // declenche la reconfig ADC (couteux)
        if (d["quality_idx"].is<int>()) {
            uint8_t q = d["quality_idx"];
            if (q < NB_PRESETS && q != cfg.quality_idx) {
                cfg.quality_idx = q; changed = true; need_reconfig = true;
            }
        }
        if (d["ch1_enabled"].is<bool>()) {
            bool v = d["ch1_enabled"].as<bool>();
            if (v != cfg.ch1_enabled) {
                cfg.ch1_enabled = v; changed = true; need_reconfig = true;
            }
        }
        if (d["ch2_enabled"].is<bool>()) {
            bool v = d["ch2_enabled"].as<bool>();
            if (v != cfg.ch2_enabled) {
                cfg.ch2_enabled = v; changed = true; need_reconfig = true;
            }
        }
        if (d["samples_per_frame"].is<int>()) {
            uint16_t n = d["samples_per_frame"];
            if (n > 0 && n <= MAX_SAMPLES_PER_FRAME &&
                n != cfg.samples_per_frame) {
                // Changer la taille de trame ne touche PAS la config ADC :
                // pas de reconfig, juste la sauvegarde.
                cfg.samples_per_frame = n; changed = true;
            }
        }
        if (d["decimation"].is<int>()) {
            uint32_t dec = d["decimation"];
            if (dec >= 1 && dec <= 65535 && dec != cfg.decimation) {
                // Decimation = nb de samples ADC moyennes par point affiche
                // (en plus de l'oversampling du preset). Pas de reconfig ADC.
                cfg.decimation = (uint16_t)dec; changed = true;
            }
        }
        // Garde-fou : au moins 1 voie active
        if (!cfg.ch1_enabled && !cfg.ch2_enabled) {
            cfg.ch1_enabled = true; changed = true; need_reconfig = true;
        }
        if (need_reconfig) g_reconfigure_adc = true;
        if (changed)       g_save_config_pending = true;
    }
    else if (strcmp(type, "save_ui_state") == 0) {
        // Stockage opaque de l'etat UI (sliders, V/div, etc.)
        JsonVariant ui = d["state"];
        if (!ui.isNull()) {
            String s;
            serializeJson(ui, s);
            cfg.ui_state = s;
            g_save_config_pending = true;
        }
    }
    else if (strcmp(type, "ping") == 0) {
        client->text("{\"type\":\"pong\"}");
    }
}

void sendBinaryFrame(const uint16_t* ch1, const uint16_t* ch2,
                     const FrameStats& stats) {
    if (ws.count() == 0) return;
    // Garde-fou heap : AsyncWebSocket autorise jusqu'a 32 messages en file
    // par client. En bicanal a 1024 samples (~4 ko/trame), un hoquet reseau
    // peut donc monopoliser >100 ko de heap et faire lacher la connexion.
    // Si la RAM libre passe sous le seuil, on saute la trame : AsyncTCP a
    // ainsi le temps de purger sa file et le heap remonte.
    if (ESP.getFreeHeap() < 90000) return;
    if (!ws.availableForWriteAll()) return;

    // Trame : header 12 octets + samples
    //   [0]    u8  version (=1)
    //   [1]    u8  flags (bit0=CH1, bit1=CH2)
    //   [2..3] u16 nb_samples par canal (LE)
    //   [4..7] u32 fs_effective_hz       (LE)
    //   [8]    u8  oversampling
    //   [9]    u8  reserved
    //   [10..11] u16 reserved
    //   [12..] samples CH1 (u16 LE) puis CH2 (u16 LE)
    const uint16_t N = stats.nb_samples;
    const uint8_t  flags = stats.flags;
    const size_t   ch_count = ((flags & 1) ? 1 : 0) + ((flags & 2) ? 1 : 0);
    const size_t   payload  = 12 + ch_count * N * 2;

    AsyncWebSocketMessageBuffer* buf = ws.makeBuffer(payload);
    if (!buf) return;
    uint8_t* p = buf->get();
    p[0] = 1;
    p[1] = flags;
    p[2] = N & 0xFF;          p[3] = (N >> 8) & 0xFF;
    uint32_t fs = stats.fs_effective_hz;
    p[4] = fs & 0xFF;         p[5] = (fs >> 8) & 0xFF;
    p[6] = (fs >> 16) & 0xFF; p[7] = (fs >> 24) & 0xFF;
    p[8] = stats.oversampling;
    p[9] = 0;
    p[10] = 0; p[11] = 0;
    size_t off = 12;
    if (flags & 1) {
        memcpy(p + off, ch1, N * 2);
        off += N * 2;
    }
    if (flags & 2) {
        memcpy(p + off, ch2, N * 2);
    }
    // Envoi a tous les clients connectes.
    // binaryAll() gere en interne la saturation et la gestion du buffer.
    ws.binaryAll(buf);
}

// ============================================================================
//  ACQUISITION ADC continuous
// ============================================================================

bool initAdcContinuous() {
    deinitAdcContinuous();

    const QualityPreset& q = PRESETS[cfg.quality_idx];
    uint8_t nb_ch = (cfg.ch1_enabled ? 1 : 0) + (cfg.ch2_enabled ? 1 : 0);
    if (nb_ch == 0) return false;

    // Mode HF : strictement mono-canal
    bool is_hf = (cfg.quality_idx == 3);
    if (is_hf && nb_ch == 2) {
        // Le client n'aurait pas du, mais on tombe en mono CH1 par defaut
        nb_ch = 1;
    }

    uint8_t os = (nb_ch == 2) ? q.os_2ch : q.os_1ch;
    uint32_t fs_total = q.fs_effective_hz * os * nb_ch;

    Serial.printf("[ADC] preset=%s nb_ch=%u OS=%u Fs_total=%u Hz\n",
                  q.name, nb_ch, os, (unsigned)fs_total);

    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = DMA_RING_BUFFER_SIZE,
        .conv_frame_size    = DMA_FRAME_SIZE,
    };
    if (adc_continuous_new_handle(&hcfg, &adc_handle) != ESP_OK) {
        Serial.println(F("[ADC] new_handle ECHEC"));
        return false;
    }

    adc_digi_pattern_config_t pattern[2] = {};
    uint8_t pn = 0;
    if (cfg.ch1_enabled || is_hf) {
        pattern[pn].atten     = ADC_ATTEN;
        pattern[pn].channel   = ADC_CH1_CHANNEL;
        pattern[pn].unit      = ADC_UNIT_USED;
        pattern[pn].bit_width = ADC_BIT_WIDTH;
        pn++;
    }
    if (cfg.ch2_enabled && !is_hf) {
        pattern[pn].atten     = ADC_ATTEN;
        pattern[pn].channel   = ADC_CH2_CHANNEL;
        pattern[pn].unit      = ADC_UNIT_USED;
        pattern[pn].bit_width = ADC_BIT_WIDTH;
        pn++;
    }

    adc_continuous_config_t dcfg = {
        .pattern_num    = pn,
        .adc_pattern    = pattern,
        .sample_freq_hz = fs_total,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    if (adc_continuous_config(adc_handle, &dcfg) != ESP_OK) {
        Serial.println(F("[ADC] config ECHEC"));
        return false;
    }
    if (adc_continuous_start(adc_handle) != ESP_OK) {
        Serial.println(F("[ADC] start ECHEC"));
        return false;
    }
    g_adc_running = true;
    return true;
}

void deinitAdcContinuous() {
    if (adc_handle) {
        adc_continuous_stop(adc_handle);
        adc_continuous_deinit(adc_handle);
        adc_handle = NULL;
    }
    g_adc_running = false;
}

void acquisitionTask(void* /*arg*/) {
    static uint8_t  raw[DMA_FRAME_SIZE];
    static uint16_t ch1_out[MAX_SAMPLES_PER_FRAME];
    static uint16_t ch2_out[MAX_SAMPLES_PER_FRAME];
    // Accumulateurs 32 bits pour l'oversampling+decimation (somme avant moyennage)
    static uint32_t acc_ch1[MAX_SAMPLES_PER_FRAME];
    static uint32_t acc_ch2[MAX_SAMPLES_PER_FRAME];
    static uint32_t cnt_ch1[MAX_SAMPLES_PER_FRAME];
    static uint32_t cnt_ch2[MAX_SAMPLES_PER_FRAME];

    for (;;) {
        if (!g_client_connected) {
            // Pas de client : on coupe l'ADC pour economiser
            if (g_adc_running) deinitAdcContinuous();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (!g_adc_running || g_reconfigure_adc) {
            g_reconfigure_adc = false;
            if (!initAdcContinuous()) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

        const QualityPreset& q = PRESETS[cfg.quality_idx];
        uint8_t nb_ch = (cfg.ch1_enabled ? 1 : 0) + (cfg.ch2_enabled ? 1 : 0);
        bool is_hf = (cfg.quality_idx == 3);
        if (is_hf) nb_ch = 1;
        uint8_t os = (nb_ch == 2) ? q.os_2ch : q.os_1ch;
        uint16_t N = cfg.samples_per_frame;
        if (N > MAX_SAMPLES_PER_FRAME) N = MAX_SAMPLES_PER_FRAME;
        uint16_t decim = cfg.decimation < 1 ? 1 : cfg.decimation;
        // Nombre total de samples ADC moyennes par point affiche
        uint32_t os_total = (uint32_t)os * decim;

        FrameStats stats;
        // Fs effectif reel = Fs preset / decimation (chaque point couvre
        // decim fois plus de temps)
        stats.fs_effective_hz = q.fs_effective_hz / decim;
        if (stats.fs_effective_hz == 0) stats.fs_effective_hz = 1;
        stats.oversampling    = os;
        stats.nb_samples      = N;
        stats.flags           = (cfg.ch1_enabled || is_hf ? 1 : 0)
                              | (cfg.ch2_enabled && !is_hf ? 2 : 0);

        // Init accumulateurs
        memset(acc_ch1, 0, sizeof(uint32_t) * N);
        memset(acc_ch2, 0, sizeof(uint32_t) * N);
        memset(cnt_ch1, 0, sizeof(uint32_t) * N);
        memset(cnt_ch2, 0, sizeof(uint32_t) * N);

        uint32_t target_raw_per_ch = (uint32_t)N * os_total;
        uint32_t got_ch1 = 0, got_ch2 = 0;

        // Boucle de remplissage de la trame
        while ((cfg.ch1_enabled || is_hf ? got_ch1 < target_raw_per_ch : true)
            && (cfg.ch2_enabled && !is_hf ? got_ch2 < target_raw_per_ch : true)
            && g_client_connected && !g_reconfigure_adc) {

            uint32_t n_read = 0;
            esp_err_t r = adc_continuous_read(adc_handle, raw,
                                              DMA_FRAME_SIZE, &n_read, 100);
            if (r != ESP_OK || n_read == 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            for (uint32_t i = 0; i < n_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t* s =
                    (adc_digi_output_data_t*)&raw[i];
                uint8_t ch  = s->type1.channel;
                uint16_t v  = s->type1.data;
                if (ch == ADC_CH1_CHANNEL && (cfg.ch1_enabled || is_hf)) {
                    uint32_t slot = got_ch1 / os_total;
                    if (slot < N) {
                        acc_ch1[slot] += v;
                        cnt_ch1[slot]++;
                        got_ch1++;
                    }
                } else if (ch == ADC_CH2_CHANNEL &&
                           cfg.ch2_enabled && !is_hf) {
                    uint32_t slot = got_ch2 / os_total;
                    if (slot < N) {
                        acc_ch2[slot] += v;
                        cnt_ch2[slot]++;
                        got_ch2++;
                    }
                }
            }
            // Sortie anticipee si plus de client
            if (!g_client_connected) break;
        }
        if (!g_client_connected || g_reconfigure_adc) continue;

        // Moyennage final + conversion raw -> mV via calibration ADC.
        // L'ADC du ESP32 classique est notoirement non-lineaire ; la
        // calibration "line fitting" utilise les coefficients eFuse pour
        // corriger les ecarts. Sans elle, on a typiquement 10% d'erreur.
        for (uint16_t i = 0; i < N; ++i) {
            uint32_t raw1 = cnt_ch1[i] ? acc_ch1[i] / cnt_ch1[i] : 0;
            uint32_t raw2 = cnt_ch2[i] ? acc_ch2[i] / cnt_ch2[i] : 0;
            int mv1 = 0, mv2 = 0;
            if (cali_enabled) {
                adc_cali_raw_to_voltage(cali_handle, (int)raw1, &mv1);
                adc_cali_raw_to_voltage(cali_handle, (int)raw2, &mv2);
            } else {
                mv1 = (int)((raw1 * 3300) / 4095);
                mv2 = (int)((raw2 * 3300) / 4095);
            }
            if (mv1 < 0)       mv1 = 0;
            if (mv1 > 65535)   mv1 = 65535;
            if (mv2 < 0)       mv2 = 0;
            if (mv2 > 65535)   mv2 = 65535;
            ch1_out[i] = (uint16_t)mv1;
            ch2_out[i] = (uint16_t)mv2;
        }

        sendBinaryFrame(ch1_out, ch2_out, stats);
        vTaskDelay(pdMS_TO_TICKS(FRAME_THROTTLE_MS));
    }
}
