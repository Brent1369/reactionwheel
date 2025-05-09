#include "driver/mcpwm.h"

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>



#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <driver/pcnt.h>
#include "freertos/queue.h"


//const char* ssid = "Xperia XA2_e9a9";
//const char* password = "brent123";


const char *ssid = "telenet-A6AD7E7";
const char *password = "vzemhjvX4arp";



AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

LSM9DS1 imu;


//#define PRINT_RAW
static unsigned long lastPrint = 0;  // Keep track of print time

#define DECLINATION -8.58  // Declination (degrees) in Boulder, CO.

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
void handleRoot(AsyncWebServerRequest *request);
void handlePID();
void handleData();


#define PWM_GPIO1 23  // Define the GPIO pin for the PWM signal
#define DIRECTION_PIN1 32

#define PWM_GPIO2 25
#define DIRECTION_PIN2 33

//#define i2cclock 22
//#define i2cdata 21

#define TACHO_A1 18
#define TACHO_B1 19

#define TACHO_A2 17
#define TACHO_B2 16

#define BUTTON 26


void MainLoop(void *pvParameter);
void loop2(void *pvParameter);
void loop3(void *pvParameter);
void loop4(void *pvParameter);
void tachoTask1(void *pvParameter);
void tachoTask2(void *pvParameter);
void SerialTask(void *pvParameter);
void ServerTask(void *pvParameter);
void tachoTask(void *pvParameter);
void driveMotor1(double dutyCycle);
void driveMotor2(double dutyCycle);
void setupPcnt1();
void setupPcnt2();

//#include <QuickPID.h>
//#include <sTune.h>
//#include "PID_AutoTune_v0.h"


float global_pitch =0;
float global_pitch_accel =0;
float global_pitch_gyro = 0;
float global_roll =0;
float global_roll_accel =0;
float global_roll_gyro = 0;

//#include <WebServer.h>
//WebServer server(80);  // HTTP server on port 80

float Kp1 = 80.0;  // Proportional Gain (Increase for faster response)
float Ki1 = 0.0;   // Derivative Gain (Increase to reduce overshoot)
float Kd1 = 10.0;
float motorMultiplier1 = -200;
float motorMultiplier2 = -200;

float Kp2 = 80.0;  // Proportional Gain (Increase for faster response)
float Ki2 = 0.0;   // Derivative Gain (Increase to reduce overshoot)
float Kd2 = 10.0;

double Setpoint1 = 0;
float Output1 = 0;

float Setpoint2 = 0;
float Output2 = 0;

float tachSpeed1 = 0;
float tachSpeed1RA = 0;
float tachSpeed2RA = 0;
float tachSpeed2 = 0;
float maxspeed = 3300;
float pid_output1 = 0;
float pid_output2 =0;

float target_pitch = 3;  //1;
float target_roll = 2;

QueueHandle_t pcntQueue;  // FreeRTOS queue for ISR → Task communication


TaskHandle_t tachoTaskHandle1 = NULL;  // Declare the task handle
TaskHandle_t tachoTaskHandle2 = NULL;  // Declare the task handle



void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("Client %u connected\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("Client %u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      if (len > 0) {
        String message = String((char *)data, len);
        if (message.indexOf('/') != -1) {
          // Format: "motorNum/P/I/D/target/multiplier"
          int motorNum = message.substring(0, message.indexOf('/')).toInt();
          message = message.substring(message.indexOf('/') + 1);

          float p = message.substring(0, message.indexOf('/')).toFloat();
          message = message.substring(message.indexOf('/') + 1);

          float i = message.substring(0, message.indexOf('/')).toFloat();
          message = message.substring(message.indexOf('/') + 1);

          float d = message.substring(0, message.indexOf('/')).toFloat();
          message = message.substring(message.indexOf('/') + 1);

          float target = message.substring(0, message.indexOf('/')).toFloat();
          float multiplier = message.substring(message.indexOf('/') + 1).toFloat();

          if (motorNum == 1) {
            Kp1 = p; Ki1 = i; Kd1 = d;
            target_pitch = target;
            motorMultiplier1 = multiplier;
          } else if (motorNum == 2) {
            Kp2 = p; Ki2 = i; Kd2 = d;
            target_roll = target;
            motorMultiplier2 = multiplier;
          }
        }
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}



  struct GraphConfig {
    const char *name;
    const char *label;
    const char *color;
    volatile float *valuePtr;
    float min;
    float max;
    const char *axis;
  };

  

  // Define all graph variables in ONE PLACE
  const GraphConfig variables[] = {
    { "pitch", "Pitch Angle (°)", "rgb(255, 99, 132)", &global_pitch, -15, 15, "y" },
    { "roll", "Roll Angle (°)", "rgb(255, 159, 64)", &global_roll, -15, 15, "y" },

    { "pitch_accel", "Pitch Angle accel (°)", "rgb(255, 99, 132)", &global_pitch_accel, -15, 15, "y" },
    { "roll_accel", "Roll Angle accel (°)", "rgb(255, 159, 64)", &global_roll_accel, -15, 15, "y" },

    { "pitch_gyro", "Pitch gyro dps", "rgb(255, 99, 132)", &global_pitch_gyro, -15, 15, "y" },
    { "roll_gyro", "Roll gyro dps", "rgb(255, 159, 64)", &global_roll_gyro, -15, 15, "y" },



    { "speed1", "Motor 1 (RPM)", "rgb(54, 162, 235)", &tachSpeed1, -50, 50, "y1" },
    { "speed2", "Motor 2 (RPM)", "rgb(75, 192, 192)", &tachSpeed2, -50, 50, "y1" },
    { "pid1", "PID Output 1", "rgb(153, 102, 255)", &pid_output1, -100, 100, "y2" },
    { "pid2", "PID Output 2", "rgb(201, 203, 207)", &pid_output2, -100, 100, "y2" }
  };


  void sendGraphData() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 50 && ws.count() > 0) {
    String result = "";
    for (size_t i = 0; i < 10; i++) {
      result += String(*variables[i].valuePtr, 2);  // Two decimal places
      if (i < sizeof(variables) / sizeof(variables[0]) - 1) {
        result += ",";
      }
    }
    result += ";";
    ws.textAll(result);
    lastSend = millis();
  }
}


// HTML Page Handler
void handleRoot(AsyncWebServerRequest *request) {

  // Central configuration - the ONLY place to edit variables



  // Generate datasets JavaScript dynamically
  String datasetsJS;
  for (size_t i = 0; i < sizeof(variables) / sizeof(variables[0]); i++) {
    if (i > 0) datasetsJS += ",";
    datasetsJS += "{";
    datasetsJS += "label:'" + String(variables[i].label) + "',";
    datasetsJS += "borderColor:'" + String(variables[i].color) + "',";
    String colorStr = String(variables[i].color);
    colorStr.replace(")", ",0.1)");
    datasetsJS += "backgroundColor:'" + colorStr + "',";
    datasetsJS += "borderWidth:1,pointRadius:0,";
    datasetsJS += "yAxisID:'" + String(variables[i].axis) + "',";
    datasetsJS += "data:[]";
    datasetsJS += "}";
  }

  // Generate axes configuration
  String axesJS = R"=====(
    x: {
      type: 'realtime',
      realtime: {
        duration: 10000,
        refresh: 100,
        delay: 50,
        pause: false,
        ttl: 100000

      }
    },
    y: {
      title: { display: true, text: 'Angle (°)' },
      beginAtZero: false,  // Let the scale adapt to data
        grace: '5%'  // Add 5% padding above/below data
    },
    y1: {
      position: 'right',
      title: { display: true, text: 'Speed (RPM)' },
      grid: { drawOnChartArea: false },
      beginAtZero: false,  // Let the scale adapt to data
      grace: '5%'  // Add 5% padding above/below data
    },
    y2: {
      position: 'right',
      title: { display: true, text: 'PID Output' },
      grid: { drawOnChartArea: false },
      display: false,
      beginAtZero: false,  // Let the scale adapt to data
      grace: '5%'  // Add 5% padding above/below data
    }
  )=====";

  String html = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Reaction Wheel - PID Control</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    .pid-controls { 
      background: #f5f5f5; 
      padding: 15px; 
      border-radius: 5px; 
      margin-bottom: 20px;
      display: flex;
      flex-wrap: wrap;
      gap: 20px;
    }
    .pid-group {
      background: white;
      padding: 10px;
      border-radius: 3px;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
    }
    .pid-group h3 {
      margin-top: 0;
      color: #333;
    }
    label {
      display: inline-block;
      width: 80px;
      font-weight: bold;
    }
    input {
      width: 80px;
      padding: 5px;
      margin-bottom: 8px;
      border: 1px solid #ddd;
      border-radius: 3px;
    }
    button {
      padding: 5px 10px;
      background: #4CAF50;
      color: white;
      border: none;
      border-radius: 3px;
      cursor: pointer;
    }
    button:hover { background: #45a049; }
    button:disabled { 
      background: #cccccc; 
      cursor: not-allowed; 
    }
    #chartControls {
      margin: 10px 0;
    }
    #chartContainer { 
      width: 100%; 
      height: 70vh; 
      margin-bottom: 20px;
      position: relative;
    }
    .zoom-instructions {
      position: absolute;
      top: 10px;
      right: 10px;
      background: rgba(255,255,255,0.8);
      padding: 5px 10px;
      border-radius: 3px;
      font-size: 12px;
    }
    #status {
      font-size: 14px; 
      color: #666;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <h1>Reaction Wheel PID Control</h1>
  
  <div class="pid-controls">
    <div class="pid-group">
      <h3>Motor 1 (Pitch)</h3>
      <div>
        <label for="p1">P:</label>
        <input type="number" id="p1" value=")=====";
      html += String(Kp1, 1);
    html += R"=====(" step="0.1">
        </div>
        <div>
          <label for="i1">I:</label>
          <input type="number" id="i1" value=")=====";
    html += String(Ki1, 2);
    html += R"=====(" step="0.01">
        </div>
        <div>
          <label for="d1">D:</label>
          <input type="number" id="d1" value=")=====";
    html += String(Kd1, 1);
    html += R"=====(" step="0.1">
        </div>
        <div>
          <label for="multiplier1">Multiplier:</label>
          <input type="number" id="multiplier1" value=")=====";
    html += String(motorMultiplier1, 1);
    html += R"=====(" step="0.1" min="0.1" max="5.0">
        </div>
        <div>
          <label for="target1">Target:</label>
          <input type="number" id="target1" value=")=====";
    html += String(target_pitch, 1);
    html += R"=====(" step="0.1">
        </div>
        <button onclick="updatePID(1)">Update</button>
      </div>
      
      <div class="pid-group">
        <h3>Motor 2 (Roll)</h3>
        <div>
          <label for="p2">P:</label>
          <input type="number" id="p2" value=")=====";
    html += String(Kp2, 1);
    html += R"=====(" step="0.1">
        </div>
        <div>
          <label for="i2">I:</label>
          <input type="number" id="i2" value=")=====";
    html += String(Ki2, 2);
    html += R"=====(" step="0.01">
        </div>
        <div>
          <label for="d2">D:</label>
          <input type="number" id="d2" value=")=====";
    html += String(Kd2, 1);
    html += R"=====(" step="0.1">
        </div>
        <div>
          <label for="multiplier2">Multiplier:</label>
          <input type="number" id="multiplier2" value=")=====";
    html += String(motorMultiplier2, 1);
    html += R"=====(" step="0.1" min="0.1" max="5.0">
        </div>
        <div>
          <label for="target2">Target:</label>
          <input type="number" id="target2" value=")=====";
    html += String(target_roll, 1);
    html += R"=====(" step="0.1">
        </div>
        <button onclick="updatePID(2)">Update</button>
      </div>
    </div>


  <div id="chartControls">
    <button id="pauseBtn">Pause</button>
    <button id="resumeBtn" disabled>Resume Live</button>
    <button id="resetZoomBtn">Reset View</button>
  </div>

  <div id="chartContainer">
    <div class="zoom-instructions">
      <b>Zoom/Pan Controls:</b><br>
      • Drag to pan<br>
      • Scroll to zoom<br>
      • Shift+Drag to zoom area<br>
      • Double-click to reset
    </div>
    <canvas id="chart"></canvas>
  </div>
  
  <div id="status">Live streaming - Interact with chart</div>

  <script src="https://cdn.jsdelivr.net/npm/hammerjs@2.0.8/hammer.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/luxon@3.4.3/build/global/luxon.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-luxon@1.2.0/dist/chartjs-adapter-luxon.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@1.2.1/dist/chartjs-plugin-zoom.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-streaming@2.0.0/dist/chartjs-plugin-streaming.min.js"></script>

  <script>
    // Central configuration for all graph variables
    const graphConfig = {
      datasets: [)=====" + datasetsJS + R"=====(],
      axes: {)=====" + axesJS + R"=====(}
    };

    let ws;
    let chart;
    let isPaused = false;
    
    function connectWebSocket() {
      ws = new WebSocket('ws://' + window.location.hostname + '/ws');
      
      ws.onopen = function() {
        console.log("WebSocket connected");
        updateStatus("Live streaming - Interact with chart");
      };
      
      ws.onclose = function() {
        console.log("WebSocket disconnected");
        updateStatus("Disconnected - Attempting to reconnect...");
        setTimeout(connectWebSocket, 2000);
      };
      
      ws.onerror = function(error) {
        console.error("WebSocket error:", error);
        updateStatus("Connection error");
      };
      
      ws.onmessage = function(event) {
        const now = Date.now();
        const points = event.data.split(';').filter(point => point.trim() !== '');
        
        points.forEach(entry => {
          const values = entry.split(',').map(Number);
          if (values.length === graphConfig.datasets.length) {
            values.forEach((val, i) => {
              chart.data.datasets[i].data.push({ x: now, y: val });
            });
          }
        });
        
        if (!isPaused && !chart.zoom._isPanning && !chart.zoom._isZooming) {
          chart.update('quiet');
        }
      };
    }
    
    function updatePID(motorNum) {
      const prefix = motorNum;
      const p = document.getElementById('p' + prefix).value;
      const i = document.getElementById('i' + prefix).value;
      const d = document.getElementById('d' + prefix).value;
      const target = document.getElementById('target' + prefix).value;
      const multiplier = document.getElementById('multiplier' + prefix).value;

      const command = `${motorNum}/${p}/${i}/${d}/${target}/${multiplier}`;
      ws.send(command);
      console.log("Sent PID update:", command);
      updateStatus(`Updated Motor ${motorNum} PID: P=${p} I=${i} D=${d} Target=${target}`);
    }
    
    function updateStatus(message) {
      document.getElementById('status').textContent = message;
    }
    
    function setupControls() {
      document.getElementById('pauseBtn').addEventListener('click', function() {
        isPaused = true;
        chart.options.scales.x.realtime.pause = true;
        this.disabled = true;
        document.getElementById('resumeBtn').disabled = false;
        updateStatus("Paused - Interact with chart");
      });
      
      document.getElementById('resumeBtn').addEventListener('click', function() {
        isPaused = false;
        chart.options.scales.x.realtime.pause = false;
        this.disabled = true;
        document.getElementById('pauseBtn').disabled = false;
        chart.update();
        updateStatus("Live streaming - Interact with chart");
      });
      
      document.getElementById('resetZoomBtn').addEventListener('click', function() {
        if (chart.resetZoom) {
          chart.resetZoom();
        }
        if (!isPaused) {
          chart.options.scales.x.realtime.pause = false;
          chart.update();
          updateStatus("Live streaming - Interact with chart");
        }
      });
    }
    
    function initChart() {
      // Initialize Hammer.js for touch gestures
      new Hammer(document.getElementById('chart'));
      
      const ctx = document.getElementById('chart').getContext('2d');
      chart = new Chart(ctx, {
        type: 'line',
        data: {
          datasets: graphConfig.datasets
    
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          interaction: {
            intersect: false,
            mode: 'nearest',
            axis: 'xy'
          },
          scales: {
            x: {
              type: 'realtime',
              realtime: {
                duration: 10000,
                refresh: 100,
                delay: 100,
                pause: false,
                ttl: 20000,


                onRefresh: chart => {
                  if (!isPaused && !chart.zoom._isPanning && !chart.zoom._isZooming) {
                    chart.options.scales.x.realtime.pause = false;
                  }
                }
                
              }
              
            },
            ...graphConfig.axes
          },
          plugins: {
            zoom: {
              pan: {
                enabled: true,
                mode: 'x',
                threshold: 10,
                modifierKey: null,
                onPanStart: () => {
                  chart.options.scales.x.realtime.pause = true;
                  updateStatus("Panning - Drag to navigate");
                },
                onPanComplete: () => {
                  if (!isPaused) {
                    chart.options.scales.x.realtime.pause = false;
                    updateStatus("Live streaming");
                  }
                }
              },
              zoom: {
                wheel: {
                  enabled: true,
                  speed: 0.1
                },
                drag: {
                  enabled: true,
                  modifierKey: 'shift',
                  backgroundColor: 'rgba(225,225,225,0.3)',
                  borderColor: 'rgba(225,225,225)',
                  borderWidth: 1,
                  threshold: 0
                },
                mode: 'x',
                onZoomStart: () => {
                  chart.options.scales.x.realtime.pause = true;
                  updateStatus("Zooming - Select area with Shift+Drag");
                },
                onZoomComplete: () => {
                  if (!isPaused) {
                    updateStatus("Paused - Click 'Resume' to return to live view");
                  }
                }
              }
            }
          }
        }
      });
    }
    
    window.addEventListener('load', function() {
      initChart();
      setupControls();
      connectWebSocket();
      
      // Debugging
      console.log('Chart.js version:', Chart.version);
      console.log('Zoom plugin available:', Chart.Zoom !== undefined);
    });
  </script>
</body>
</html>
)=====";
  request->send(200, "text/html", html);
}

/*
// Handle PID value submission
void handlePID() {
    if (server.hasArg("P") && server.hasArg("I") && server.hasArg("D")) {
        Kp1 = server.arg("P").toFloat();
        Ki1 = server.arg("I").toFloat();
        Kd1 = server.arg("D").toFloat();
        
        Serial.printf("Updated PID values: P = %.2f, I = %.2f, D = %.2f\n", Kp1, Ki1, Kd1);
    }
    
    // Redirect back to the root page so values are updated in the form
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

*/


void setup() {







  // Configure MCPWM parameters for the first PWM output (PWM_GPIO1)
  mcpwm_config_t pwm_config1;
  pwm_config1.frequency = 20000;                // 20 kHz frequency
  pwm_config1.cmpr_a = 100.0;                    // Duty cycle for MCPWM0A (50%)
  pwm_config1.cmpr_b = 0.0;                     // Not useds
  pwm_config1.counter_mode = MCPWM_UP_COUNTER;  // Count-up mode
  pwm_config1.duty_mode = MCPWM_DUTY_MODE_0;    // Active high duty cycle

  // Apply configuration to MCPWM Timer 0
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config1);

  // Configure MCPWM parameters for the second PWM output (PWM_GPIO2)
  mcpwm_config_t pwm_config2;
  pwm_config2.frequency = 20000;                // 20 kHz frequency
  pwm_config2.cmpr_a = 100.0;                    // Duty cycle for MCPWM1A (25%)
  pwm_config2.cmpr_b = 0.0;                     // Not used
  pwm_config2.counter_mode = MCPWM_UP_COUNTER;  // Count-up mode
  pwm_config2.duty_mode = MCPWM_DUTY_MODE_0;    // Active high duty cycle

  // Apply configuration to MCPWM Timer 1
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config2);


  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_GPIO1);  // GPIO23 for PWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_GPIO2);  // GPIO22 for PWM1A



  //Serial.begin(115200);
  Serial.begin(500000);
  //delay(1000);

  Wire.begin();
  pinMode(DIRECTION_PIN1, OUTPUT);
  pinMode(DIRECTION_PIN2, OUTPUT);
  Serial.println("test");

  pinMode(BUTTON, INPUT_PULLDOWN);
  /*
    tuner.Configure(
        360.0f,                    // Input span (degrees)
        OUTPUT_MAX - OUTPUT_MIN,   // Output span (200 for -100 to 100)
        0.0f,                      // Start from 0 output
        OUTPUT_STEP,               // 30% step size (optimal for wheels)
        TEST_TIME_SEC,             // 5 sec test duration
        SETTLE_TIME_SEC,           // 2 sec settling time
        SAMPLES                    // 300 samples
    );
    
    // 5. Safety and Mode Setup
    tuner.SetEmergencyStop(10000);    // Disable safety limits (or set a safe value)
    myPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX); // Set bounds first!
    myPID.SetMode(myPID.Control::manual); // Must be manual during tuning

*/


  imu.settings.gyro.enabled = true;
  imu.settings.gyro.scale = 2000;       // Max sensitivity for fast motion
  imu.settings.gyro.sampleRate = 6;     // 952 Hz (fastest)
  imu.settings.gyro.bandwidth = 3;      // Highest bandwidth
  imu.settings.gyro.HPFEnable = false;  // Raw data, no high-pass filtering


  imu.settings.accel.enabled = true;
  imu.settings.accel.scale = 16;            // Highest motion range
  imu.settings.accel.sampleRate = 6;        // 952 Hz (fastest)
  imu.settings.accel.bandwidth = 0;         // Max bandwidth (408 Hz)
  imu.settings.accel.highResEnable = true;  // More precision



  if (imu.begin() == false)  // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will work for an out of the box LSM9DS1 Breakout, but may need to be modified if the board jumpers are.");
    //while (1);
  }

//delay(10000);
  // Initialize MCPWM unit
  // Initialize GPIOs for PWM outputs
  





  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }


  // OTA Setup
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();


  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());






  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve HTML
  server.on("/", HTTP_GET, handleRoot);

  // Start server
  server.begin();

  // Disable WiFi sleep for maximum performance
  WiFi.setSleep(false);


  //vTaskDelay(20000);


  pcntQueue = xQueueCreate(10, sizeof(int));  // Queue for PCNT data
  if (pcntQueue == NULL) {
    Serial.println("Error creating queue!");
    return;
  }

  setupPcnt1();
  setupPcnt2();

  xTaskCreatePinnedToCore(MainLoop, "MainLoop", 2048 * 2, NULL, 5, NULL, CONFIG_ARDUINO_RUNNING_CORE);

  //xTaskCreatePinnedToCore(&loop2, "loop2", 2048 * 2, NULL, 5, NULL, 0);

  xTaskCreatePinnedToCore(SerialTask, "loop3", 2048 * 2, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(ServerTask, "loop4", 2048 * 2, NULL, 4, NULL, 0);


  //xTaskCreatePinnedToCore(&tachoTask, "tachoTask", 2048 * 2, NULL, 4, NULL, 0);


  //xTaskCreatePinnedToCore(tachoTask1, "TachoTask1", 2048, NULL, 5, NULL, 0);
  //xTaskCreatePinnedToCore(tachoTask2, "TachoTask2", 2048, NULL, 5, NULL, 0);

  xTaskCreatePinnedToCore(tachoTask1, "TachoTask3", 2048, NULL, 5, &tachoTaskHandle1, 0);
  xTaskCreatePinnedToCore(tachoTask2, "TachoTask3", 2048, NULL, 5, &tachoTaskHandle2, 0);
}


int currentFrequency = 0;  // Stores the last set frequency



//float target_pitch = 0;  // Desired pitch angle (e.g., level position)
float previous_error1 = 0;
float previous_error2 = 0;




int AccelerationPrevioustime = 0;
int AccelerationCurrentTime = 0;


float previousDutyCycle1 = 0;
float previousDutyCycle2 = 0;

//float motor_speed_pwmX =0;



/*
void sendWebSocketData() {
  static uint32_t lastSend = 0;
  if(micros() - lastSend >= 3000) {  // 3ms interval
    char data[32];
    snprintf(data, sizeof(data), "%.2f,%.1f", global_pitch, tachSpeed1);
    webSocket.broadcastTXT(data);
    lastSend = micros();
  }
}*/

int setMotorAcceleration1(int PID_OUT) {

  int newDutyCycle = 0;

  PID_OUT = constrain(PID_OUT, -100, 100);

  //previousDutyCycle1 = previousDutyCycle1 * 0.9 + PID_OUT ;
  previousDutyCycle1 = PID_OUT;

  //previousDutyCycle =  constrain(PID_OUT + 0.0137 * motor_speed_pwmX, -100, 100);
  //motor_speed_pwmX += previousDutyCycle;

  driveMotor1(previousDutyCycle1);
  return previousDutyCycle1;
}

int setMotorAcceleration2(int PID_OUT) {

  int newDutyCycle = 0;

  PID_OUT = constrain(PID_OUT, -100, 100);

  //previousDutyCycle2 = previousDutyCycle2 * 0.9 + PID_OUT;
  previousDutyCycle2 =   PID_OUT;

  //previousDutyCycle =  constrain(PID_OUT + 0.0137 * motor_speed_pwmX, -100, 100);
  //motor_speed_pwmX += previousDutyCycle;

  driveMotor2(previousDutyCycle2);
  return previousDutyCycle2;
}


// Drive motor with direction control
void driveMotor1(double dutyCycle) {
  //Serial.printf("DRIVE COMMAND: %.2f\n", dutyCycle);  // Add this line

  dutyCycle = constrain(dutyCycle, -100, 100);  // Clamp to range

  // Set direction
  if (dutyCycle >= 0) {
    digitalWrite(DIRECTION_PIN1, LOW);  // Forward
  } else {
    digitalWrite(DIRECTION_PIN1, HIGH);  // Reverse
  }

  if(digitalRead(BUTTON) == 0){ //switch = 0 = turn motor off
    dutyCycle = 0;
  }

  dutyCycle = 100 - abs(int(dutyCycle));
  // Convert to absolute PWM value (0-255)
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyCycle);
}

// Drive motor with direction control
void driveMotor2(double dutyCycle) {
  //Serial.printf("DRIVE COMMAND: %.2f\n", dutyCycle);  // Add this line

  dutyCycle = constrain(dutyCycle, -100, 100);  // Clamp to range

  // Set direction
  if (dutyCycle >= 0) {
    digitalWrite(DIRECTION_PIN2, LOW);  // Forward
  } else {
    digitalWrite(DIRECTION_PIN2, HIGH);  // Reverse
  }

  if(digitalRead(BUTTON) == 0){ //switch = 0 = turn motor off
    dutyCycle = 0;
  }

  dutyCycle = 100 - abs(int(dutyCycle));
  // Convert to absolute PWM value (0-255)
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyCycle);
}

float weighted_speed(float speed) {
  float multiplier = 1;
  if (speed <= 300 || speed >= -300) {
    return speed * multiplier;  // Apply 10x multiplier for values in range [-300, 300]
  } else if (speed > 300) {
    return (300 * multiplier) + (speed - 300);   // Apply 10x up to 300, then add extra normally
  } else {                                       // speed < -300
    return (-300 * multiplier) + (speed + 300);  // Apply 10x up to -300, then add remaining normally
  }
}


static unsigned long last_adjust_time = 0;
static float equilibrium_buffer = 0;
static float search_adjustment = 0;  // Adjustment for active search

// Low-pass filter for smoothing the global_pitch (and tachSpeed1 if needed)
float filtered_pitch = 0;  // Filtered global pitch

float angular_acceleration_wheel = 0;
float angular_acceleration_wheel_filtered = 0;
float previous_angular_velocity_wheel = 0;




#define SAMPLE_SIZE 100  // Number of samples for averaging

float relativeSamplesPositive[SAMPLE_SIZE] = { 0 };
float relativeSamplesNegative[SAMPLE_SIZE] = { 0 };
int sampleIndexPositive = 0;
int sampleIndexNegative = 0;
float relative_avg_positive = 0;
float relative_avg_negative = 0;


void updateRelativeAverages(float relative, float global_pitch) {
  if (global_pitch > 0.5) {
    // Store new value and update moving average for positive angles
    relativeSamplesPositive[sampleIndexPositive] = relative;
    sampleIndexPositive = (sampleIndexPositive + 1) % SAMPLE_SIZE;

    float sum = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      sum += relativeSamplesPositive[i];
    }
    relative_avg_positive = sum / SAMPLE_SIZE;
  } else if (global_pitch < -0.5) {
    // Store new value and update moving average for negative angles
    relativeSamplesNegative[sampleIndexNegative] = relative;
    sampleIndexNegative = (sampleIndexNegative + 1) % SAMPLE_SIZE;

    float sum = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      sum += relativeSamplesNegative[i];
    }
    relative_avg_negative = sum / SAMPLE_SIZE;
  }
}




void sendData() {
  String json = String(global_pitch, 2) + "," + String(tachSpeed1, 1);
  ws.textAll(json);
}

#define BATCH_SIZE 20  // 100ms / 5ms = 20 samples per batch

String dataBuffer = "";



void MainLoop(void *pvParameter) {



  float angleincrement = 0.01;
  float incrementmax = 1;



  float integral_error1 = 0;      // NEW: Integral error accumulator
  float integral_error2 = 0;      // NEW: Integral error accumulator
  const float max_integral = 50;  // NEW: Anti-windup limit
  unsigned long last_time = 0;

  unsigned long last_time_batch = 0;



  float equilibrium_pitch = 0;

  float motorspeedtotaltest = 0;



  float prevderivative1 = 0;
  float changederivative = 0;


  while (1) {


    vTaskDelay(5);




    while (1) {
      if (imu.gyroAvailable() && imu.accelAvailable()) {
        imu.readGyro();
        imu.readAccel();
        printAttitude(imu.ax, imu.ay, imu.az, -imu.gx, -imu.gy, imu.gz);
        break;
      }
      //vTaskDelay(1);
    }

    // Time difference (dt)
    unsigned long current_time = micros();
    //float current_time = micros();
    float dt = (current_time - last_time) / 1000000.0;
    //if (dt < 0.001) dt = 0.001;
    last_time = current_time;

    // Flip target_pitch logic (unchanged)
    //if (abs(global_pitch - target_pitch) < 2) {
    //  target_pitch = (target_pitch > 0) ? -target_pitch : target_pitch;
    //}
    if (global_pitch < target_pitch) {
      //target_pitch += 0.1 * dt;
    } else {
      // target_pitch -= 0.1 * dt;
    }
    //target_pitch -= 0.00000005 * (tachSpeed1/60) / dt * 2 * 3.14; //convert to rad/s

    // Check if the system is stable (ignore if moving too fast)

    //target_pitch += 0.00005 * (tachSpeed1 / 60.0) * 2 * 3.14; // Convert RPM to rad/s







    //target_pitch = constrain(target_pitch, -15, 15);


    float error1 = target_pitch - global_pitch;

    float error2 = target_roll - global_roll;

    /*if(global_pitch > 0){
      target_pitch -= angleincrement;
    }else if(global_pitch < -0){
      target_pitch += angleincrement;
    }
    if(target_pitch > incrementmax) target_pitch = incrementmax;
    if(target_pitch < (-incrementmax)) target_pitch = -incrementmax;*/

    //target_pitch = - tachSpeed1/maxspeed  * 2;
    //target_roll = - tachSpeed2/maxspeed  * 2;




    integral_error1 += error1 * dt;                                             // NEW: Accumulate integral
    integral_error1 = constrain(integral_error1, -max_integral, max_integral);  // Anti-windup

    float derivative1 = (error1 - previous_error1) / dt;
    previous_error1 = error1;

    changederivative = (derivative1 - prevderivative1) / dt;
    prevderivative1 = derivative1;

    float derivative2 = (error2 - previous_error2) / dt;
    previous_error2 = error2;




    float currenttach = tachSpeed1;
    // NEW: PID output instead of PD
    pid_output1 = Kp1 * error1 + Ki1 * integral_error1 + Kd1 * derivative1  +  motorMultiplier1 * (tachSpeed1RA / 50.0);
    pid_output1 = constrain(pid_output1, -100, 100);

    //motorspeedtotaltest +=pid_output1;

    pid_output1 = setMotorAcceleration1(pid_output1);
    //pid_output1 = constrain(pid_output1, -100, 100);


    pid_output2 = Kp2 * error2 + Ki2 * integral_error2 + Kd2 * derivative2 +  motorMultiplier2  * (tachSpeed2RA / 50.0);
    pid_output2 = constrain(pid_output2, -100, 100);
    pid_output2 = setMotorAcceleration2(pid_output2);
    //pid_output2 = constrain(pid_output2, -100, 100);


    float alpha = 0.05;


    filtered_pitch = alpha * global_pitch + (1 - alpha) * filtered_pitch;


    // Calculate the angular acceleration of the reaction wheel
    angular_acceleration_wheel = (currenttach - previous_angular_velocity_wheel) / dt;
    previous_angular_velocity_wheel = currenttach;

    angular_acceleration_wheel_filtered = angular_acceleration_wheel_filtered * (1 - alpha) + angular_acceleration_wheel * alpha;

    float relative = 0;

    if (derivative1 != 0) {
      relative = angular_acceleration_wheel_filtered / changederivative;
    }

    if (abs(relative) > 0.5) {
      updateRelativeAverages(relative, global_pitch);
    }

    // Estimate the imbalance based on pitch angle
    float imbalance_threshold = 1.0;  // Threshold for determining imbalance (in radians)
    float correction_factor = 0.1;    // Correction factor to adjust target pitch

    // If the pitch exceeds the threshold in either direction, adjust the target pitch
    if (abs(filtered_pitch) > imbalance_threshold) {
      // Apply a correction in the opposite direction to balance the system
      //target_pitch = -correction_factor * filtered_pitch;
    } else {
      // Otherwise, maintain the current equilibrium
      //target_pitch = 0.0;  // Or you can use a more sophisticated equilibrium detection method
    }



    // Direction and duty cycle (unchanged logic, but use pid_output)
    //int dutyCycle1 = 100 - abs(int(pid_output1));
    //dutyCycle1 = constrain(dutyCycle1, 0, 100);


    sendGraphData();

    /*
    dataBuffer += String(global_pitch, 2) + "," + String(tachSpeed1, 2) + ";";

     if (millis() - last_time_batch >= 100) {
      if (ws.count() > 0) {
        ws.textAll(dataBuffer);  // Send to all clients
      }
      dataBuffer = "";              // Clear buffer
      last_time_batch = millis();      // Reset timer
      // Serial.print(" send data ");
    }*/


    // Debugging prints updated for PID

    // Debugging prints updated for PID


    Serial.print(" | motor1: ");
    Serial.print(currenttach >= 0 ? "+" : "");
    Serial.print(currenttach, 2);
    Serial.print(" | motor1RA: ");
    Serial.print(tachSpeed1RA >= 0 ? "+" : "");
    Serial.print(tachSpeed1RA, 2);



    Serial.print(" | TimeLoop: ");
    Serial.print(dt >= 0 ? "+" : "");
    Serial.print(dt * 1000000, 2);

    Serial.print(" pitch: ");
    Serial.print(global_pitch >= 0 ? "+" : "");
    Serial.print(global_pitch, 2);
    Serial.print(" pitchF: ");
    Serial.print(filtered_pitch >= 0 ? "+" : "");
    Serial.print(filtered_pitch, 2);
    Serial.print(" | Target: ");
    Serial.print(target_pitch >= 0 ? "+" : "");
    Serial.print(target_pitch, 2);

    Serial.print(" | PID Out1: ");
    Serial.print(pid_output1 >= 0 ? "+" : "");
    Serial.print(pid_output1, 2);
    Serial.println();


    /*

    Serial.println();
    Serial.print(" | relative_avg_positive: ");
    Serial.print(relative_avg_positive  >= 0 ? "+" : ""); Serial.print(relative_avg_positive , 2);
    Serial.print(" | relative_avg_negative : ");
    Serial.print(relative_avg_negative  >= 0 ? "+" : ""); Serial.print(relative_avg_negative , 2);
    Serial.print(" | changederivative : ");
    Serial.print(changederivative  >= 0 ? "+" : ""); Serial.print(changederivative , 2);
    


    Serial.print(Kp1 >= 0 ? "+" : ""); Serial.print(Kp1, 2);
    Serial.print(" | I1: ");
    Serial.print(Ki1 >= 0 ? "+" : ""); Serial.print(Ki1, 2);
    Serial.print(" | D1: ");
    Serial.print(Kd1 >= 0 ? "+" : ""); Serial.print(Kd1, 2);


    Serial.print(" accel wheel: ");
    Serial.print(angular_acceleration_wheel >= 0 ? "+" : ""); Serial.print(angular_acceleration_wheel, 2);
    Serial.print(" accel filter: ");
    Serial.print(angular_acceleration_wheel_filtered >= 0 ? "+" : ""); Serial.print(angular_acceleration_wheel_filtered, 2);

    Serial.print(" speed stick: ");
    Serial.print(derivative1 >= 0 ? "+" : ""); Serial.print(derivative1, 2);

    Serial.print(" relative: ");
    Serial.print(relative >= 0 ? "+" : ""); Serial.print(relative, 2);
    

    
*/
    /*
    Serial.print(" roll: ");
    Serial.print(global_roll >= 0 ? "+" : ""); Serial.print(global_roll, 2);
    Serial.print(" | Target: ");
    Serial.print(target_roll >= 0 ? "+" : ""); Serial.print(target_roll, 2);
*/


    /*

    Serial.print(" | PID Out2: ");
    Serial.print(pid_output2 >= 0 ? "+" : ""); Serial.print(pid_output2, 2);
    //Serial.print(" | Duty: ");
    //Serial.print(dutyCycle >= 0 ? "+" : ""); Serial.print(dutyCycle, DEC);
    Serial.print( " | P1: ");
    Serial.print(Kp1 >= 0 ? "+" : ""); Serial.print(Kp1, 2);
    Serial.print(" | I1: ");
    Serial.print(Ki1 >= 0 ? "+" : ""); Serial.print(Ki1, 2);
    Serial.print(" | D1: ");
    Serial.print(Kd1 >= 0 ? "+" : ""); Serial.print(Kd1, 2);

    Serial.print(" | P2: ");
    Serial.print(Kp2 >= 0 ? "+" : ""); Serial.print(Kp2, 2);
    Serial.print(" | Integral error: ");
    Serial.print(integral_error1 >= 0 ? "+" : ""); Serial.print(integral_error1, 2);
    Serial.print(" | D2: ");
    Serial.print(Kd2 >= 0 ? "+" : ""); Serial.print(Kd2, 2);

    Serial.print(" | motor2: ");
    Serial.print(tachSpeed2 >= 0 ? "+" : ""); Serial.print(tachSpeed2,2);

    */








    /*
    Serial.print(" | P_E: ");
    Serial.print(error1 >= 0 ? "+" : ""); Serial.print(error1, 2);
    Serial.print(" | I_E: ");
    Serial.print(integral_error1 >= 0 ? "+" : ""); Serial.print(integral_error1, 2);
    Serial.print(" | D_E: ");
    Serial.print(derivative1 >= 0 ? "+" : ""); Serial.print(derivative1, 2);
    
    Serial.print(" | motor: ");
    Serial.print(tachSpeed1 >= 0 ? "+" : ""); Serial.println(tachSpeed1, DEC);*/
  }
}

#define PRINT_SPEED 20  // 250 ms between prints

// Kalman Filter Variables
float Q_angle = 0.001;  // Covariance of gyroscope noise
float Q_gyro = 0.003;   // Reduce bias estimation speed
float R_angle = 0.5;    // Trust accelerometer less when moving

float bias_pitch = 0, bias_roll = 0;
float angle_pitch = 0, angle_roll = 0;
float P_pitch[2][2] = { { 0, 0 }, { 0, 0 } };
float P_roll[2][2] = { { 0, 0 }, { 0, 0 } };
float dt = 0.01;  // Filter sampling time

void Kalman_Filter(float &angle, float &bias, float P[2][2], float angle_m, float gyro_m, bool ignore_accel) {
  // Predict step
  angle += (gyro_m - bias) * dt;

  // Error covariance update
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;

  if (!ignore_accel) {  // Ignore accelerometer correction when moving fast
    float angle_err = angle_m - angle;
    float S = P[0][0] + R_angle;
    float K_0 = P[0][0] / S;
    float K_1 = P[1][0] / S;

    angle += K_0 * angle_err;
    bias += K_1 * angle_err;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K_0 * P00_temp;
    P[0][1] -= K_0 * P01_temp;
    P[1][0] -= K_1 * P00_temp;
    P[1][1] -= K_1 * P01_temp;
  }
}



#define ALPHA 0.999  // Complementary filter weight (adjust if needed)

unsigned long lastUpdate = 0;  // Store the last update time in microseconds
float angle_pitch2 = 0;
float angle_roll2 = 0;
float angle_pitch_gyro = 77;  // Angle based purely on gyroscope
float angle_roll_gyro = 0;    // Angle based purely on gyroscope

void printAttitude(float ax, float ay, float az, float gx, float gy, float gz) {
  // Calculate dt (time difference between updates) using micros for better accuracy
  unsigned long now = micros();                // Current time in microseconds
  float dt2 = (now - lastUpdate) / 1000000.0;  // Convert microseconds to seconds


  float sensitivityFactor = 2000.0 / 32768.0;  // This is the scaling factor

 
  gy = imu.calcGyro(imu.gy) + 0.5;  //+ 0.5;  // pitch
  gx = imu.calcGyro(imu.gx) -2.5;//- 2.5;  //roll


global_pitch_gyro = gy;
global_roll_gyro = gx;
  // Update last update time
  lastUpdate = now; gx = imu.calcGyro(imu.gx) -1.5;//- 2.5;  //roll

  // Compute roll and pitch from accelerometer
  float accel_roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

global_pitch_accel = accel_pitch;
global_roll_accel = accel_roll;

  Kalman_Filter(angle_pitch, bias_pitch, P_pitch, accel_pitch, gy, false);
  Kalman_Filter(angle_roll, bias_roll, P_roll, accel_roll, gx, false);

  // Complementary filter
  angle_pitch2 = ALPHA * (angle_pitch2 + gy * dt2) + (1.0 - ALPHA) * accel_pitch;
  angle_roll2 = ALPHA * (angle_roll2 + gx * dt2) + (1.0 - ALPHA) * accel_roll;


  // Gyroscope-only angle (integrated over time)
  angle_pitch_gyro += gy * dt2;  // Integrating gyroscope data
  angle_roll_gyro += gx * dt2;   // Integrating gyroscope data

  // Global angles
  global_pitch = angle_pitch2 - 3 + 1.5 + 2 - 1;  //-0.8 + 1.5 -1.8;// angle_pitch - 0.8
  global_roll = angle_roll2;

  // Send values to Serial Plotter
  //Serial.print(accel_pitch); // Raw accelerometer pitch
  // Serial.print("\t");
  // Serial.print(angle_pitch2); // Filtered pitch (Complementary filter)
  //Serial.print("\t");
  // Serial.print(angle_pitch_gyro); // Gyroscope-only pitch (pure gyro)
  // Serial.print("\t");
  //Serial.println(angle_pitch); // Gyroscope-only pitch (pure gyro)
  // Serial.print("\t");

  // Serial.print(100); // Gyroscope-only pitch (pure gyro)
  //Serial.print("\t");
  // Serial.println(gx); // Gyroscope pitch rate*/
}







#define ENCODER_PPR 200  // Pulses Per Revolution (adjust for your encoder)






// Setup function for the first tacho (PCNT_UNIT_0)
/*
void setupPcnt1() {
    pcnt_config_t pcntConfig = {
        .pulse_gpio_num = TACHO_A1,
        .ctrl_gpio_num = TACHO_B1,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcntConfig);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}*/



volatile uint32_t lastMicros = 0;


void IRAM_ATTR pcntISR1(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /*
 // Get the event status
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_0, &status);

    // Check if the high limit event triggered
    if (status & PCNT_EVT_H_LIM) {
        // Handle the high limit event (e.g., reset counter or other logic)
        Serial.println("High limit event triggered");
    }

    // Check if the low limit event triggered
    if (status & PCNT_EVT_L_LIM) {
        // Handle the low limit event (e.g., reset counter or other logic)
        Serial.println("Low limit event triggered");
    }*/
  vTaskNotifyGiveFromISR(tachoTaskHandle1, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR pcntISR2(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /*
 // Get the event status
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_0, &status);

    // Check if the high limit event triggered
    if (status & PCNT_EVT_H_LIM) {
        // Handle the high limit event (e.g., reset counter or other logic)
        Serial.println("High limit event triggered");
    }

    // Check if the low limit event triggered
    if (status & PCNT_EVT_L_LIM) {
        // Handle the low limit event (e.g., reset counter or other logic)
        Serial.println("Low limit event triggered");
    }*/
  vTaskNotifyGiveFromISR(tachoTaskHandle2, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



#define AVG_WINDOW_SIZE 10  // Number of values to average



void tachoTask1(void *pvParameter) {
  int16_t lastCount = 0;
  uint32_t lastMicros = micros();
  int16_t count;

  float speedHistory[AVG_WINDOW_SIZE] = { 0 };  // Stores the last N speed values
  int historyIndex = 0;                         // Index to store the current speed in the array
  float runningAverage = 0.0f;                  // Running average of speed

  while (1) {
    int direction = 0;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for ISR notification

    uint32_t currentMicros = micros();
    pcnt_get_counter_value(PCNT_UNIT_0, &count);  // Read PCNT count
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_0, &status);

    // Check if the high limit event triggered
    if (status & PCNT_EVT_H_LIM) {
      // Handle the high limit event (e.g., reset counter or other logic)
      //Serial.println("High limit event triggered");
      direction = 1;
    }

    // Check if the low limit event triggered
    if (status & PCNT_EVT_L_LIM) {
      // Handle the low limit event (e.g., reset counter or other logic)
      //Serial.println("Low limit event triggered");
      direction = -1;
    }
    //pcnt_counter_clear(PCNT_UNIT_0);            // Reset counter after reading

    int countold = count;
    count = direction * 10;

    float deltaTime = (currentMicros - lastMicros) / 1e6f;  // Convert to seconds
    int16_t deltaCount = count - lastCount;
    float speed = ((float)deltaCount / ENCODER_PPR) / deltaTime;  // RPs
                                                                  /*
            Serial.print("Count: ");
            Serial.print(count);
            Serial.print("countold: ");
            Serial.print(countold);
            Serial.print(", Speed (RPM): ");
            Serial.println(speed);
            Serial.print(", deltaTime: ");
            Serial.println(deltaTime * 1000000);

*/
    lastCount = 0;
    lastMicros = currentMicros;

    tachSpeed1 = speed;


    speedHistory[historyIndex] = speed;                   // Store the current speed
    historyIndex = (historyIndex + 1) % AVG_WINDOW_SIZE;  // Circular buffer

    // Calculate the running average
    float sum = 0.0f;
    for (int i = 0; i < AVG_WINDOW_SIZE; i++) {
      sum += speedHistory[i];
    }
    runningAverage = sum / AVG_WINDOW_SIZE;  // Calculate the average speed
    tachSpeed1RA = runningAverage;
  }
}






void tachoTask2(void *pvParameter) {
  int16_t lastCount = 0;
  uint32_t lastMicros = micros();
  int16_t count;

  float speedHistory[AVG_WINDOW_SIZE] = { 0 };  // Stores the last N speed values
  int historyIndex = 0;                         // Index to store the current speed in the array
  float runningAverage = 0.0f;                  // Running average of speed

  while (1) {
    int direction = 0;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for ISR notification

    uint32_t currentMicros = micros();
    pcnt_get_counter_value(PCNT_UNIT_1, &count);  // Read PCNT count
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_1, &status);

    // Check if the high limit event triggered
    if (status & PCNT_EVT_H_LIM) {
      // Handle the high limit event (e.g., reset counter or other logic)
      //Serial.println("High limit event triggered");
      direction = 1;
    }

    // Check if the low limit event triggered
    if (status & PCNT_EVT_L_LIM) {
      // Handle the low limit event (e.g., reset counter or other logic)
      //Serial.println("Low limit event triggered");
      direction = -1;
    }
    //pcnt_counter_clear(PCNT_UNIT_0);            // Reset counter after reading

    int countold = count;
    count = direction * 10;

    float deltaTime = (currentMicros - lastMicros) / 1e6f;  // Convert to seconds
    int16_t deltaCount = count - lastCount;
    float speed = ((float)deltaCount / ENCODER_PPR) / deltaTime;  // RPs
                                                                  /*
            Serial.print("Count: ");
            Serial.print(count);
            Serial.print("countold: ");
            Serial.print(countold);
            Serial.print(", Speed (RPM): ");
            Serial.println(speed);
            Serial.print(", deltaTime: ");
            Serial.println(deltaTime * 1000000);

*/
    lastCount = 0;
    lastMicros = currentMicros;

    tachSpeed2 = speed;


    speedHistory[historyIndex] = speed;                   // Store the current speed
    historyIndex = (historyIndex + 1) % AVG_WINDOW_SIZE;  // Circular buffer

    // Calculate the running average
    float sum = 0.0f;
    for (int i = 0; i < AVG_WINDOW_SIZE; i++) {
      sum += speedHistory[i];
    }
    runningAverage = sum / AVG_WINDOW_SIZE;  // Calculate the average speed
    tachSpeed2RA = runningAverage;
  }
}






void setupPcnt1() {
  pcnt_config_t pcntConfig = {
    .pulse_gpio_num = TACHO_A1,  // Encoder signal
    .ctrl_gpio_num = TACHO_B1,   // Direction control
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DEC,
    .counter_h_lim = 10,   // Set threshold for interrupt
    .counter_l_lim = -10,  // No need for a negative limit
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0
  };

  pcnt_unit_config(&pcntConfig);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);

  // Enable interrupt on high limit
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcntISR1, NULL);

  pcnt_counter_resume(PCNT_UNIT_0);  // put before maybe
}

void setupPcnt2() {
  pcnt_config_t pcntConfig = {
    .pulse_gpio_num = TACHO_A2,  // Encoder signal
    .ctrl_gpio_num = TACHO_B2,   // Direction control
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DEC,
    .counter_h_lim = 10,   // Set threshold for interrupt
    .counter_l_lim = -10,  // No need for a negative limit
    .unit = PCNT_UNIT_1,
    .channel = PCNT_CHANNEL_0
  };

  pcnt_unit_config(&pcntConfig);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);

  // Enable interrupt on high limit
  pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

  //pcnt_isr_service_install(1);
  pcnt_isr_handler_add(PCNT_UNIT_1, pcntISR2, NULL);

  pcnt_counter_resume(PCNT_UNIT_1);  // put before maybe
}






void ServerTask(void *pvParameter) {
  while (1) {

    ArduinoOTA.handle();
    //server.handleClient();
    vTaskDelay(100);
  }
}


void resetTuning() {
  //tuningComplete = false;

  /*tuner.Configure(
        360.0f,                    // Input span (degrees)
        OUTPUT_MAX - OUTPUT_MIN,   // Output span (200 for -100 to 100)
        0.0f,                      // Start from 0 output
        OUTPUT_STEP,               // 30% step size (optimal for wheels)
        TEST_TIME_SEC,             // 5 sec test duration
        SETTLE_TIME_SEC,           // 2 sec settling time
        SAMPLES                    // 300 samples
    );
    
    // 5. Safety and Mode Setup
    tuner.SetEmergencyStop(10000);    // Disable safety limits (or set a safe value)
    myPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX); // Set bounds first!
    myPID.SetMode(myPID.Control::manual); // Must be manual during tuning
  Serial.println("Tuning system fully reset");*/
}

// Add this global variable (with your other declarations)


void SerialTask(void *pvParameter) {
  while (1) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      input.toLowerCase();  // Make command case-insensitive

      // Handle "tune" command
      if (input == "t") {
        //startZNTuning();
        //startAutoTune();
        //resetTuning(); // Full reset instead of just flag change
        vTaskDelay(100);
        continue;
      }

      // Find slashes to parse input
      int firstSlash = input.indexOf('/');
      int secondSlash = input.indexOf('/', firstSlash + 1);
      int thirdSlash = input.indexOf('/', secondSlash + 1);

      if (firstSlash != -1 && secondSlash != -1 && thirdSlash != -1) {
        String pidSelect = input.substring(0, firstSlash);
        String P_str = input.substring(firstSlash + 1, secondSlash);
        String I_str = input.substring(secondSlash + 1, thirdSlash);
        String D_str = input.substring(thirdSlash + 1);

        float P = P_str.toFloat();
        float I = I_str.toFloat();
        float D = D_str.toFloat();

        if (pidSelect == "1") {
          Kp1 = P;
          Ki1 = I;
          Kd1 = D;
          Serial.printf("Updated PID 1: P=%.2f I=%.2f D=%.2f\n", Kp1, Ki1, Kd1);
        } else if (pidSelect == "2") {
          Kp2 = P;
          Ki2 = I;
          Kd2 = D;
          Serial.printf("Updated PID 2: P=%.2f I=%.2f D=%.2f\n", Kp2, Ki2, Kd2);
        } else {
          Serial.println("Invalid PID selection. Use '1/P/I/D' or '2/P/I/D'.");
        }
      } else {
        Serial.println("Invalid format. Use '1/P/I/D' or '2/P/I/D'.");
      }
    }
    vTaskDelay(100);
  }
}








void loop() {
  vTaskDelay(1000);
}



/*

int countTest = 0;
int lastCountTest = 0;

// Task to handle the first tacho sensor (PCNT_UNIT_0)
void tachoTask1(void *pvParameter) {
    setupPcnt1();
    int16_t lastCount = 0;
    uint32_t lastMicros = micros();

    while (1) {
        int16_t count;
        pcnt_get_counter_value(PCNT_UNIT_0, &count);
        uint32_t currentMicros = micros();

        if (count != lastCount) {
            float deltaTime = (currentMicros - lastMicros) / 1e6f;
            int16_t deltaCount = count - lastCount;
            float speed = deltaCount / deltaTime;
            tachSpeed1 = (speed / ENCODER_PPR) * 60.0f;

            // Print values using Serial

            
            Serial.print("Count: ");
            Serial.print(count);
            Serial.print(", LastCount: ");
            Serial.print(lastCount);
            Serial.print(", deltaCount: ");
            Serial.print(deltaCount);
            Serial.print(", DeltaTime: ");
            Serial.print(deltaTime * 1000000); // Print with 6 decimal places
            Serial.print(" s, Speed: ");
            Serial.println(tachSpeed1);


            lastCount = count;
            lastMicros = currentMicros;
            
        }

       vTaskDelay(10);
    }
}


// Setup function for the second tacho (PCNT_UNIT_1)
void setupPcnt2() {
    pcnt_config_t pcntConfig = {
        .pulse_gpio_num = TACHO_A2,
        .ctrl_gpio_num = TACHO_B2,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_1,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcntConfig);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}


// Task to handle the second tacho sensor (PCNT_UNIT_1)
void tachoTask2(void *pvParameter) {
    //setupPcnt1();
    setupPcnt2();
    int16_t lastCount = 0;
    uint32_t lastMicros = micros();

    while (1) {
        int16_t count;
        pcnt_get_counter_value(PCNT_UNIT_1, &count);
        uint32_t currentMicros = micros();

        if (count != lastCount) {
            float deltaTime = (currentMicros - lastMicros) / 1e6f;
            int16_t deltaCount = count - lastCount;
            float speed = deltaCount / deltaTime;
            tachSpeed2 = (speed / ENCODER_PPR) * 60.0f;

            lastCount = count;
            lastMicros = currentMicros;
        }

        vTaskDelay(100);
    }
}


*/


/*

void startAutoTune() {
    tuning = true;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetControlType(1); // PID
    aTuneStartTime = millis();
    lastDebugTime = millis();
    maxPitchDuringTuning = global_pitch;
    minPitchDuringTuning = global_pitch;
    oscillationCount = 0;
    lastPitch = global_pitch;
    increasing = false;
    
    Serial.println("\n\n=== Starting Auto-Tune ===");
    Serial.println("Parameters:");
    Serial.printf("- Step Size: %.1f%%\n", aTuneStep);
    Serial.printf("- Noise Band: %.1f deg\n", aTuneNoise);
    Serial.printf("- Timeout: 30 seconds\n");
    Serial.println("Monitoring system response...");
    Serial.println("Time(s)\tOutput\tPitch\tState");
}


// Optional Monitoring
void logData() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Setpoint:"); Serial.print(Setpoint);
    Serial.print(" Input:"); Serial.print(global_pitch);
    Serial.print(" Output:"); Serial.println(Output);
    lastPrint = millis();
  }
}

bool tuningComplete = true;



void printTuningDebug() {
    unsigned long currentTime = millis();
    if (currentTime - lastDebugTime >= debugInterval) {
        lastDebugTime = currentTime;
        
        // Track oscillations
        if ((global_pitch > lastPitch && !increasing) || 
            (global_pitch < lastPitch && increasing)) {
            oscillationCount++;
            increasing = !increasing;
        }
        lastPitch = global_pitch;
        
        // Track min/max
        if (global_pitch > maxPitchDuringTuning) maxPitchDuringTuning = global_pitch;
        if (global_pitch < minPitchDuringTuning) minPitchDuringTuning = global_pitch;
        
        // Print status
        Serial.printf("%.1f\t%.1f\t%.2f\t", 
                     (currentTime - aTuneStartTime)/1000.0,
                     Output, 
                     global_pitch);
        Serial.println();
        // Print tuning state

    }
}


bool znTuning = false;
float znKu = 0;
float znPu = 0;
unsigned long znLastSwitchTime = 0;
float znLastPeak = 0;
int znState = 0; // 0=disabled, 1=increasing Kp, 2=measuring oscillations

void startZNTuning() {
  znTuning = true;
  znState = 1;
  Kp1 = 50.0;  // Start with small Kp
  Ki1 = 0;
  Kd1 = 0;
  znLastPeak = 0;
  Serial.println("\nStarting Z-N Tuning");
  Serial.println("Phase 1: Finding critical gain (Ku)");
}

void handleZNTuning() {
  if (!znTuning) return;

  float currentPitch = global_pitch;
  
  // Phase 1: Find Ku (critical gain)
  if (znState == 1) {
    // Gradually increase Kp until oscillations sustain
    Kp1 += 0.5;
    Serial.printf("Testing Kp=%.1f, Pitch=%.2f\n", Kp1, currentPitch);
    
    // Check for sustained oscillations
    if (abs(currentPitch) > 20.0) {  // Large enough oscillations
      znKu = Kp1;
      znState = 2;
      znLastSwitchTime = millis();
      znLastPeak = currentPitch;
      Serial.println("\nFound critical gain Ku=" + String(znKu));
      Serial.println("Phase 2: Measuring oscillation period (Pu)");
    }
  }
  // Phase 2: Measure Pu (oscillation period)
  else if (znState == 2) {
    // Detect zero crossings
    if ((znLastPeak > 0 && currentPitch < 0) || 
        (znLastPeak < 0 && currentPitch > 0)) {
      unsigned long now = millis();
      znPu = (now - znLastSwitchTime) / 1000.0; // Convert to seconds
      znLastSwitchTime = now;
      znLastPeak = currentPitch;
      Serial.printf("Oscillation detected! Period Pu=%.2fs\n", znPu);
      
      // Complete after 3-4 oscillations
      static int oscillationCount = 0;
      if (++oscillationCount >= 3) {
        completeZNTuning();
      }
    }
  }
}

void completeZNTuning() {
  znTuning = false;
  
  // Ziegler-Nichols PID rules
  Kp1 = 0.6 * znKu;
  Ki1 = 2 * Kp1 / znPu;
  Kd1 = Kp1 * znPu / 8;
  
  Serial.println("\nZiegler-Nichols Tuning Complete!");
  Serial.printf("Ku=%.2f, Pu=%.2fs\n", znKu, znPu);
  Serial.printf("Calculated PID:\nKp=%.2f\nKi=%.2f\nKd=%.2f\n", Kp1, Ki1, Kd1);
}
*/
/*
void loop2(void *pvParameter) {
  unsigned long lastTime = micros();
  while (1) {
    vTaskDelay(1000);
    unsigned long now = micros();
    unsigned long loopTime = now - lastTime;
    lastTime = now;

    //Serial.print("Loop time (us): ");
    //Serial.println(loopTime);  // Prints execution time in microseconds

    if ( imu.gyroAvailable() && imu.accelAvailable() ) {
      imu.readGyro();
      imu.readAccel();
      printAttitude(imu.ax, imu.ay, imu.az, -imu.gx, -imu.gy, imu.gz);
    }
    //if ( imu.accelAvailable() ) {
   //   imu.readAccel();
    //}

    if ((lastPrint + PRINT_SPEED) < millis() || 1==1) {
      //printAttitude(imu.ax, imu.ay, imu.az, -imu.gx, -imu.gy, imu.gz);
      lastPrint = millis();
    }
  }
}*/


// Initialize sTune
//sTune tuner(&global_pitch, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);

// Initialize QuickPID
//QuickPID myPID(&global_pitch, &Output, &Setpoint);


/*
const float OUTPUT_MIN = -100.0f;
const float OUTPUT_MAX = 100.0f;
const float OUTPUT_STEP = 60.0f;  // 30% of range (optimal for reaction wheels)
const uint32_t SETTLE_TIME_SEC = 1;
const uint32_t TEST_TIME_SEC = 2;
const uint16_t SAMPLES = 300;


// PID AutoTune variables
//PID_ATune aTune(&global_pitch, &Output); // Input = angle, Output = motor command

bool tuning = false;
float aTuneStep = 50.0; // Bang-bang amplitude (start with 50% of max output)
float aTuneNoise = 1.0; // How much noise is acceptable in the input (degrees)
unsigned long aTuneStartTime = 0;


unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 500; // ms between debug prints
float maxPitchDuringTuning = 0;
float minPitchDuringTuning = 0;
unsigned int oscillationCount = 0;
float lastPitch = 0;
bool increasing = false;
*/
