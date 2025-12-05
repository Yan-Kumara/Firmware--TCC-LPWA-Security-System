#include <Arduino.h>
#include <Wire.h>                 // Biblioteca para comunicação I2C
#include <Keypad.h>             //Biblioteca para o teclado matricial
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>//Bibliotecas do expansor 
#include <Adafruit_MCP23XXX.h>

#define SIMULATION_MODE 1 // 1 == simulação - 0 == funcionamento comum (envio p/modem)
#define KEYPAD_BUZZER_CHANNEL 0 // Canal PWM para o feedback do teclado
#define KEYPAD_BUZZER_RESOLUTION 8
#define NOTE_KEYPRESS 523 // Frequencia do buzzer do teclado
// --- Definições de Hardware ---
#if SIMULATION_MODE == 0
  #define MODEM_SERIAL Serial1
  #define MODEM_RX_PIN 26
  #define MODEM_TX_PIN 12
#endif
HardwareSerial cameraSerial(2); 

// GPIOs CONECTADOS AO ESP32
const int SMOKE_SENSOR_PIN = 32; // sensor de fumaça (depreciado)
const int KEYPAD_BUZZER_PIN = 18; // Buzzer para o teclado
const int SIREN_RELAY_PIN = 25;  // Sirene principal do alarme
const int RED_LED_PIN = 5; // led desliga
const int BLUE_LED_PIN = 19; // led ligado
const int SMOKE_THRESHOLD = 1800;
const long BUTTON_LOCKOUT_MS = 300;

// Objeto do MCP23017 
Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;
#define MCP1_ADDRESS 0x20
#define MCP2_ADDRESS 0x21

//Mapeamento dos Sensores (no mcp)
#define SENSOR_PIR_1_PIN   8  //GPB0
#define SENSOR_VIB_1_PIN   9  //GPB1
#define SENSOR_VIB_2_PIN   10 //GPB2
#define SENSOR_MAG_1_PIN   11 //GPB3
#define SENSOR_MAG_2_PIN   12 //GPB4
#define SENSOR_MAG_3_PIN   13 //GPB5

// Configurações do teclado matricial

const char MASTER_PASSWORD[7] = "123456"; // Senha de 6 dígitos + terminador nulo
const byte ROWS = 4; 
const byte COLS = 4; 
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// Pinos do MCP onde o teclado está conectado (GPA0-GPA7)
byte rowPins[ROWS] = {0, 1, 2, 3}; 
byte colPins[COLS] = {4, 5, 6, 7};
String enteredPassword = "";
unsigned long lastKeypadActivity = 0;
const long KEYPAD_TIMEOUT_MS = 5000; // reseta senha após 5 segundos


// * Configurações da Rede e MQTT *
const char apn[]             = "timbrasil.br"; //APN pode variar
const char* mqtt_broker      = "10.0.0.105";
const int   mqtt_port        = 1883;
const char* mqtt_client_id   = "esp32-tcc-yan-01";

// * Tópicos MQTT *
const char* topic_events       = "tcc/seguranca/eventos";
const char* topic_camera_start = "tcc/seguranca/camera/start";
const char* topic_camera_chunk = "tcc/seguranca/camera/chunk";
const char* topic_camera_end   = "tcc/seguranca/camera/end";

// * Constantes da Câmera *
#define CAMERA_READ_TIMEOUT 15000
#define CHUNK_SIZE 1024

// Classe "ponte" para fazer o Keypad funcionar com o MCP23017
class Keypad_MCP23017 : public Keypad {
public:
  Keypad_MCP23017(Adafruit_MCP23X17 *mcp, char *userKeymap, byte *row, byte *col, byte numRows, byte numCols)
    : Keypad(userKeymap, row, col, numRows, numCols), _mcp(mcp) {}

  void pin_mode(byte pinNum, byte mode) { _mcp->pinMode(pinNum, mode); }
  void pin_write(byte pinNum, boolean level) { _mcp->digitalWrite(pinNum, level); }
  int  pin_read(byte pinNum) { return _mcp->digitalRead(pinNum); }
private:
  Adafruit_MCP23X17 *_mcp;
};

//Cria o objeto do keypad com a ponte
Keypad_MCP23017 customKeypad = Keypad_MCP23017(&mcp1, makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Variáveis de Estado
bool isSmokeAlarmActive = false;
bool isSystemArmed = false;
unsigned long lastPressTime = 0;
bool cameraTriggered = false;
bool isMqttConnected = false;
unsigned long lastMqttCheck = 0;
uint16_t lastMcp1States = 0; // Guarda o último estado dos 16 pinos do MCP1

// função de envios comando AT
#if SIMULATION_MODE == 0 // apenas se fora do modo simulação
String sendCommand(const String& cmd, const char* expectedResponse, unsigned long timeout) {
    String response = "";
    while(MODEM_SERIAL.available()) { MODEM_SERIAL.read(); }
    Serial.print("Enviando -> ");
    Serial.println(cmd);
    MODEM_SERIAL.println(cmd);
    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        if (MODEM_SERIAL.available()) {
            char c = MODEM_SERIAL.read();
            response += c;
            if (response.indexOf(expectedResponse) != -1) {
                Serial.print("Recebido <- ");
                Serial.println(response);
                return response;
            }
        }
    }
    Serial.print("TIMEOUT! Resposta parcial <- ");
    Serial.println(response);
    return response;
}
#endif

// Função de feedback do teclado e gerenciamento do teclado

void beepKeyPress() {
  ledcWrite(KEYPAD_BUZZER_CHANNEL, 255); 
  delay(25);
  ledcWrite(KEYPAD_BUZZER_CHANNEL, 0);
}

void beepSuccess() {
  ledcWrite(KEYPAD_BUZZER_CHANNEL, 255);
  delay(500);
  ledcWrite(KEYPAD_BUZZER_CHANNEL, 0);
}

void beepError() {
  for (int i = 0; i < 3; i++) {
    ledcWrite(KEYPAD_BUZZER_CHANNEL, 255);
    delay(100);
    ledcWrite(KEYPAD_BUZZER_CHANNEL, 0);
    delay(50);
  }
}

void resetPasswordEntry() {
  enteredPassword = "";
  Serial.println("Input da senha resetado.");
}

void manageKeypad() {
  // Verifica o timeout
  if (enteredPassword.length() > 0 && (millis() - lastKeypadActivity > KEYPAD_TIMEOUT_MS)) {
    Serial.println("Timeout do teclado!");
    beepError();
    resetPasswordEntry();
  }

  char key = customKeypad.getKey();
    // Para debuggin: Imprime qualquer tecla detectada
  if (key) {
    Serial.print("Tecla detectada: ");
    Serial.println(key);    
    beepKeyPress();
    lastKeypadActivity = millis();

    if (key == '*') {
      Serial.println("Tecla '*' pressionada. Limpando senha.");
      resetPasswordEntry();
    } 
    else if (key == '#') {
      Serial.println("Tecla '#' pressionada. Verificando senha...");
      if (enteredPassword.equals(MASTER_PASSWORD)) {
        Serial.println("Senha correta!");
        beepSuccess();
        isSystemArmed = !isSystemArmed;
        if (isSystemArmed) Serial.println(">>> SISTEMA ARMADO <<<");
        else Serial.println(">>> SISTEMA DESARMADO <<<");
      } else {
        Serial.println("Senha incorreta!");
        beepError();
      }
      resetPasswordEntry();
    }
    else if (enteredPassword.length() < 6) {
      // Adiciona apenas dígitos numéricos à senha
      if (isDigit(key)) {
        enteredPassword += key;
        Serial.print("Senha atual: ");
        Serial.println(enteredPassword);
      } else {
        Serial.println("Tecla invalida para senha (A-D). Ignorando.");
      }
    }
  }
}

bool publishMqttMessage(const char* topic, const String& payload) { //função de publicação  MQTT
  #if SIMULATION_MODE == 1 // simulação
    Serial.println('\n');
    Serial.printf("[MQTT-SIM] Publicando no topico: '%s'\n", topic);
    Serial.printf("[MQTT-SIM] Tamanho do Payload: %d bytes\n", payload.length());
    Serial.print("[MQTT-SIM] Payload (previa): ");
    Serial.println(payload.substring(0, 60) + (payload.length() > 60 ? "..." : ""));
    Serial.println("\n");
    delay(100);
    return true;
  #else // função de envio real
    if (!isMqttConnected) {
        Serial.printf("ERRO: MQTT desconectado. Nao foi possivel publicar no topico '%s'\n", topic);
        return false;
    }
    String cmd = "AT+QMTPUB=0,0,0,0,\"" + String(topic) + "\"";
    String response = sendCommand(cmd, ">", 5000);
    if (response.indexOf('>') != -1) {
        Serial.printf("Enviando payload para o topico '%s'...\n", topic);
        MODEM_SERIAL.print(payload);
        delay(100);
        MODEM_SERIAL.write(0x1A);
        response = sendCommand("", "OK", 20000);
        if (response.indexOf("OK") != -1) {
            Serial.println("Publicacao confirmada!");
            return true;
        }
    }
    Serial.println("Falha na publicacao MQTT.");
    return false;
  #endif
}

// Função de Registro de Eventos
void logEvent(const char* sensorType, int pin, const char* eventType, int analogValue = -1) {
  unsigned long now = millis();
  String payload = "{";
  payload += "\"sensorType\":\"" + String(sensorType) + "\",";
  payload += "\"pin\":" + String(pin) + ",";
  payload += "\"event\":\"" + String(eventType) + "\",";
  payload += "\"timestamp\":" + String(now);
  if (analogValue != -1) { payload += ",\"value\":" + String(analogValue); }
  payload += "}";
  Serial.printf("\n[LOG] Evento: %s no pino %d - %s\n", sensorType, pin, eventType);
  publishMqttMessage(topic_events, payload);
}

String readBase64FromCamera() { // função para obter dados da ESPCAM
    Serial.println("Aguardando dados da camera...");
    String base64Data = "";
    bool receivingData = false;
    unsigned long startTime = millis();

    while (millis() - startTime < CAMERA_READ_TIMEOUT) {
        if (cameraSerial.available()) {
            String line = cameraSerial.readStringUntil('\n');
            line.trim(); // Remove caracteres de nova linha e espaços

            if (line.startsWith("B64_START")) {
                Serial.println("Marcador B64_START recebido. Lendo dados...");
                receivingData = true;
                continue; // Pula para a próxima iteração para começar a ler os dados
            }

            if (receivingData) {
                if (line.startsWith("B64_END")) {
                    Serial.println("Marcador B64_END recebido. Leitura concluida.");
                    return base64Data;
                } else {
                    // Anexa a linha de dados (que pode ser um pedaço da imagem)
                    base64Data += line;
                }
            }
        }
    }
    
    Serial.println("TIMEOUT! Nao foi possivel ler a imagem da camera.");
    return "";
}


void triggerCameraAndUpload() { // comunica com a ESPCAM e envia os dados
    Serial.println("\n--- INICIANDO PROCESSO DE CAPTURA E UPLOAD (SIMULADO) ---");
    cameraSerial.println("SNAP");
    String imageB64 = readBase64FromCamera();
    if (imageB64.length() == 0) {
        Serial.println("ERRO: Nao ha dados de imagem para enviar.");
        return;
    }
    Serial.printf("Imagem recebida. Tamanho da string Base64: %d bytes\n", imageB64.length());

    if (!publishMqttMessage(topic_camera_start, "start")) return;
    delay(100);

    int totalChunks = (imageB64.length() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    for (int i = 0; i < imageB64.length(); i += CHUNK_SIZE) {
        String chunk = imageB64.substring(i, i + CHUNK_SIZE);
        Serial.printf("Preparando chunk %d/%d...\n", (i/CHUNK_SIZE) + 1, totalChunks);
        if (!publishMqttMessage(topic_camera_chunk, chunk)) {
            publishMqttMessage(topic_camera_end, "error");
            return;
        }
        delay(100);
    }
    
    publishMqttMessage(topic_camera_end, "end");
    Serial.println("--- UPLOAD (SIMULADO) CONCLUIDO ---");
}


//Funções de Gerenciamento de sensores

void manageSmokeSensor() {// Gerencia o sensor de fumaça (depreciado)
  int currentValue = analogRead(SMOKE_SENSOR_PIN);
  if (currentValue > SMOKE_THRESHOLD && !isSmokeAlarmActive) {
    logEvent("Fumaca MQ-7", SMOKE_SENSOR_PIN, "ALERTA DE FUMACA", currentValue);
    isSmokeAlarmActive = true;
  } else if (currentValue <= SMOKE_THRESHOLD && isSmokeAlarmActive) {
    logEvent("Fumaca MQ-7", SMOKE_SENSOR_PIN, "Nivel normalizado", currentValue);
    isSmokeAlarmActive = false;
  }
}

// Funções de Estado dos LED
void updateStatusLEDs() {
  if (isSystemArmed) {
    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  } else {
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
  }
}

void manageCameraCommunication() {  // Debugging, print de dados da câmera
  while (cameraSerial.available()) {

    Serial.write(cameraSerial.read());

  }
}


#if SIMULATION_MODE == 0 // Fora da simulação - Conecta à rede através dos AT
  bool connectToNetwork() {
      Serial.println("--- Conectando a Rede Celular ---");
      for (int i = 0; i < 30; i++) {
          String response = sendCommand("AT+CEREG?", "+CEREG:", 2000);
          if (response.indexOf(",1") != -1 || response.indexOf(",5") != -1) {
              Serial.println("Registrado na rede com sucesso!");
              return true;
          }
          Serial.println("Aguardando registro na rede...");
          delay(2000);
      }
      return false;
  }

  bool connectMqtt() {
      Serial.println("--- Conectando ao Broker MQTT ---");
      sendCommand("AT+QICSGP=1,1,\"" + String(apn) + "\",\"\",\"\",1", "OK", 10000);
      sendCommand("AT+QIACT=1", "OK", 30000);

      String cmd = "AT+QMTOPEN=0,\"" + String(mqtt_broker) + "\"," + String(mqtt_port);
      String response = sendCommand(cmd, "+QMTOPEN: 0,0", 60000);
      if (response.indexOf("+QMTOPEN: 0,0") == -1) {
          Serial.println("Falha ao abrir conexao TCP com broker.");
          return false;
      }

      cmd = "AT+QMTCONN=0,\"" + String(mqtt_client_id) + "\"";
      response = sendCommand(cmd, "+QMTCONN: 0,0,0", 60000);
      if (response.indexOf("+QMTCONN: 0,0,0") != -1) {
          Serial.println("Conectado ao Broker MQTT com sucesso!");
          isMqttConnected = true;
          return true;
      }
      
      Serial.println("Falha ao conectar ao Broker MQTT.");
      isMqttConnected = false;
      return false;
  }

#endif

void manageMcpSensors() { // Menager dos sensores ligados ao MCP
   uint16_t currentMcp1States = mcp1.readGPIOAB();
  if (currentMcp1States != lastMcp1States) {
    for (int i = 0; i < 16; i++) {
      bool currentPinState = (currentMcp1States >> i) & 1;
      bool lastPinState = (lastMcp1States >> i) & 1;

      if (currentPinState != lastPinState) {
        switch (i) {
          case SENSOR_MAG_1_PIN:
          case SENSOR_MAG_2_PIN:
          case SENSOR_MAG_3_PIN:
            if (currentPinState == HIGH) logEvent("Porta Magnetico", i, "PORTA ABERTA");
            else logEvent("Porta Magnetico", i, "Porta Fechada");
            break;
          case SENSOR_PIR_1_PIN:
            if (currentPinState == HIGH) logEvent("Movimento PIR", i, "MOVIMENTO DETECTADO");
            break;
          case SENSOR_VIB_1_PIN:
          case SENSOR_VIB_2_PIN:
            if (currentPinState == LOW) logEvent("Vibracao", i, "VIBRACAO DETECTADA");
            break;
        }
      }
    }
    lastMcp1States = currentMcp1States;
  }
}


void setup() {
    Serial.begin(115200);
    cameraSerial.begin(115200, SERIAL_8N1, 16, 17);
    Serial.println("\n TCC Yan Kumara - Firmware v4.0 - Com Expansor do Teclado Matricial");
    
    Wire.begin(); 
    if (!mcp1.begin_I2C(MCP1_ADDRESS)) {
      Serial.println("ERRO: Nao foi possivel encontrar o MCP23017 no endereco 0x20. Travando.");
      while (1);
    }
    Serial.println("MCP23017 #1 (0x20) encontrado!");
    
    mcp1.pinMode(SENSOR_PIR_1_PIN, INPUT);
    mcp1.pinMode(SENSOR_VIB_1_PIN, INPUT_PULLUP);
    mcp1.pinMode(SENSOR_VIB_2_PIN, INPUT_PULLUP);
    mcp1.pinMode(SENSOR_MAG_1_PIN, INPUT_PULLUP);
    mcp1.pinMode(SENSOR_MAG_2_PIN, INPUT_PULLUP);
    mcp1.pinMode(SENSOR_MAG_3_PIN, INPUT_PULLUP);
    Serial.println("Pinos dos sensores no MCP configurados.");
    lastMcp1States = mcp1.readGPIOAB();

    ledcSetup(KEYPAD_BUZZER_CHANNEL, NOTE_KEYPRESS, KEYPAD_BUZZER_RESOLUTION);
    ledcAttachPin(KEYPAD_BUZZER_PIN, KEYPAD_BUZZER_CHANNEL);
    ledcWrite(KEYPAD_BUZZER_CHANNEL, 0);

    // Configuração do Sirene do Alarme
    pinMode(SIREN_RELAY_PIN, OUTPUT);
    digitalWrite(SIREN_RELAY_PIN, HIGH); 

    #if SIMULATION_MODE == 1 // diferenciamento dos modos
      Serial.println("MODO DE SIMULACAO DO MODEM ATIVADO");
      isMqttConnected = true;
    #else
      Serial.println("MODO DE MODEM REAL");
      MODEM_SERIAL.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
      Serial.println("Aguardando 10 segundos para o modem iniciar...");
      delay(10000);
      if (connectToNetwork()) { connectMqtt(); }
    #endif
   
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    Serial.println("Calibrando sensores... Aguarde 15 segundos.");
    delay(15000);
    Serial.println("Sistema pronto.");
}

void loop() {

  manageKeypad();
  updateStatusLEDs();
  manageCameraCommunication();
  #if SIMULATION_MODE == 0
    if (millis() - lastMcpCheck > 60000) {
        lastMcpCheck = millis();
        if (!isMqttConnected) {
            Serial.println("Conexao MQTT perdida. Tentando reconectar...");
            connectMqtt();
        }
    }
  #endif

  if (isSystemArmed) {
    manageMcpSensors();
    manageSmokeSensor();

    bool isDoorOpen = (mcp1.digitalRead(SENSOR_MAG_1_PIN) == HIGH ||
                       mcp1.digitalRead(SENSOR_MAG_2_PIN) == HIGH ||
                       mcp1.digitalRead(SENSOR_MAG_3_PIN) == HIGH);
    
    if (isDoorOpen) {// Caso de Alarme

      digitalWrite(SIREN_RELAY_PIN, LOW);
  
      if (!cameraTriggered) {
        cameraTriggered = true;
        triggerCameraAndUpload();
      }
    } else {
      digitalWrite(SIREN_RELAY_PIN, HIGH);
      cameraTriggered = false;
    }
  } else {
    // Garante que a sirene esteja desligada quando o sistema está desarmado
    digitalWrite(SIREN_RELAY_PIN, HIGH);
    cameraTriggered = false;
  }
  
}