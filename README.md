1)esp32

#include <Wire.h>
#include <WiFi.h>

const char* ssid = "abinesh";      // Update with your WiFi SSID
const char* password = "12345678";  // Update with your WiFi Password

WiFiServer tcpServer(1234);         // TCP server on port 1234
#define ARDUINO_ADDRESS 8          // I2C address of the Arduino Uno

void setup() {
  Serial.begin(115200);
  Wire.begin();                    // Initialize I2C as master
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  
  tcpServer.begin();
}

void sendCommandToArduino(String cmd) {
  Wire.beginTransmission(ARDUINO_ADDRESS);
  // Send the command as a series of bytes:
  Wire.write((uint8_t*)cmd.c_str(), cmd.length());
  Wire.endTransmission();
}

void loop() {
  WiFiClient client = tcpServer.available();
  if (client && client.connected()) {
    String command = client.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      Serial.println("Received: " + command);
      sendCommandToArduino(command);
    }
  }
}



2)
Uno
#include <Wire.h>

#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6

#define TRIG1 5
#define ECHO1 4
#define TRIG2 3
#define ECHO2 2

#define IR_LEFT A0
#define IR_RIGHT A1

String currentCommand = "STOP";  // 🔄 Keeps track of the last command

void setup() {
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveCommand);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  Serial.println("System Initialized.");
}

void loop() {
  // Obstacle Detection
  int distance1 = getDistance(TRIG1, ECHO1);
  int distance2 = getDistance(TRIG2, ECHO2);

  if (distance1 < 100 || distance2 < 100) {
    stopMotors();
    Serial.println("🚨 Obstacle Detected! Stopping...");
    delay(300);
    return;
  }

  // Execute last received command
  if (currentCommand == "FORWARD") {
    moveForward();
    Serial.println("Moving Forward...");
  } else if (currentCommand == "LEFT") {
    turnLeft();
    Serial.println("Turning Left...");
  } else if (currentCommand == "RIGHT") {
    turnRight();
    Serial.println("Turning Right...");
  } else if (currentCommand == "STOP") {
    stopMotors();
    Serial.println("Stopped.");
  }

  delay(300); // Delay for loop cycle
}

void receiveCommand(int numBytes) {
  String command = "";
  while (Wire.available()) {
    char c = Wire.read();
    command += c;
  }
  command.trim();
  Serial.println("I2C Command Received: " + command);

  currentCommand = command;  // 🔁 Update the stored command
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}




python code for autonomous 

import cv2
import socket
import time
from ultralytics import YOLO

ESP_IP = "192.168.230.102"  # Replace with your ESP32's IP address
ESP_PORT = 1234

# Load YOLOv8 model (ensure "yolov8n.pt" is available)
model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

prev_command = None
CONFIDENCE_THRESHOLD = 0.5


def send_command(command):
    global prev_command
    if command != prev_command:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ESP_IP, ESP_PORT))
            sock.send((command + "\n").encode())
            sock.close()
            print(f"Sent command: {command}")
            prev_command = command
        except Exception as e:
            print(f"Connection Error: {e}")
            time.sleep(2)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    obstacle_left = obstacle_right = obstacle_center = False
    detected_objects = False

    for result in results:
        for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
            x_min, y_min, x_max, y_max = map(int, box[:4])
            confidence = float(conf)
            label = model.names[int(cls)]

            if confidence < CONFIDENCE_THRESHOLD:
                continue

            detected_objects = True
            center_x = (x_min + x_max) // 2

            # Decide direction based on object position:
            if center_x < frame.shape[1] / 3:
                obstacle_left = True
            elif center_x > (2 * frame.shape[1]) / 3:
                obstacle_right = True
            else:
                obstacle_center = True

            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Define command based on detected obstacles:
    if obstacle_center:
        command = "STOP"
    elif obstacle_left:
        command = "RIGHT"
    elif obstacle_right:
        command = "LEFT"
    else:
        command = "FORWARD"

    if not detected_objects:
        print("⚠️ No objects detected!")

    send_command(command)

    cv2.putText(frame, f"Direction: {command}", (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()




python code for manual

from flask import Flask, render_template, request, redirect, Response
import socket
import cv2

app = Flask(__name__)

ESP32_IP = "192.168.230.102"
ESP32_PORT = 1234

cap = cv2.VideoCapture(0)  # Open webcam

def send_command(command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((ESP32_IP, ESP32_PORT))
            sock.sendall((command + "\n").encode())
            print(f"Sent: {command}")
    except Exception as e:
        print(f"Failed to send command: {e}")

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        command = request.form['command']
        send_command(command)
        return redirect('/')
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)




    html code for autonomous

<!DOCTYPE html>
<html>
<head>
    <title>Car Control Panel</title>
    <style>
        body { text-align: center; font-family: Arial; background-color: #f4f4f4; }
        h1 { margin-top: 20px; }

        .video-container {
            margin: 20px auto;
            width: 640px;
            height: 480px;
            border: 3px solid #333;
        }

        .btn-group {
            margin-top: 20px;
        }

        button {
            width: 120px;
            height: 50px;
            margin: 10px;
            font-size: 16px;
            font-weight: bold;
            background-color: #007BFF;
            color: white;
            border: none;
            border-radius: 8px;
            cursor: pointer;
        }

        button:hover {
            background-color: #0056b3;
        }
    </style>
</head>
<body>
    <h1>🕹️BUMBLEEBEE Manual Car Control Panel</h1>

    <div class="video-container">
        <img src="{{ url_for('video_feed') }}" width="640" height="480">
    </div>

    <form method="POST">
        <div class="btn-group">
            <button name="command" value="FORWARD">FORWARD</button><br>
            <button name="command" value="LEFT">LEFT</button>
            <button name="command" value="STOP">STOP</button>
            <button name="command" value="RIGHT">RIGHT</button><br>
            <button name="command" value="BACKWARD">BACKWARD</button>
        </div>
    </form>
</body>
</html>
