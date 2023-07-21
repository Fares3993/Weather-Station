#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ESP32_MailClient.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

#define DHTPIN 33     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT11     // DHT 11

DHT dht(DHTPIN, DHTTYPE);




// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "TP-LINK_5956";
const char* password = "32714227";

// To send Emails using Gmail on port 465 (SSL)
#define emailSenderAccount    "iot74031@gmail.com"
#define emailSenderPassword   "fwukizhenuvcihnj"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "[ALERT] ESP32 Weather"

// Default Recipient Email Address
String inputMessage = "iot74031@gmail.com";
String enableEmailChecked = "checked";
String inputMessage2 = "true";
// Default Threshold Temperature Value
String inputMessage3 = "50.0";
String lastTemperature;
// Default Threshold Humidity Value
String inputMessage4 = "10.0";
String lastHumidity;

// HTML web page to handle 3 input fields (email_input, enable_email_input, threshold_input)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Email Notification with Weather</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h2>ESP32 Temperature</h2>
  <h3>%TEMPERATURE% &deg;C</h3>
   <h2>ESP32 Humidity</h2>
  <h3>%HUMIDITY% &percnt;</h3>
  <h2>ESP Email Notification</h2>
  <form action="/get">
    Email Address <input type="email" name="email_input" value="%EMAIL_INPUT%" required><br>
    Enable Email Notification <input type="checkbox" name="enable_email_input" value="true" %ENABLE_EMAIL%><br>
    Temperature Threshold <input type="number" step="0.1" name="threshold_input" value="%THRESHOLD%" required><br>
    Humidity Threshold <input type="number" step="0.1" name="threshold_input1" value="%THRESHOLD1%" required><br>

    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

String readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.print(t);
    Serial.println("*C");

    return String(t);
  }
}
String readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.print(h);
    Serial.println("%");

    return String(h);
  }
}
// Replaces placeholder with DS18B20 values
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return lastTemperature;
  }
  else if(var == "HUMIDITY"){
    return lastHumidity;
  }
  else if(var == "EMAIL_INPUT"){
    return inputMessage;
  }
  else if(var == "ENABLE_EMAIL"){
    return enableEmailChecked;
  }
  else if(var == "THRESHOLD"){
    return inputMessage3;
  }
  else if(var == "THRESHOLD1"){
    return inputMessage4;
  }
  return String();
}
// Flag variable to keep track if email notification was sent or not
bool emailSent = false;
// bool emailSent = false;

const char* PARAM_INPUT_1 = "email_input";
const char* PARAM_INPUT_2 = "enable_email_input";
const char* PARAM_INPUT_3 = "threshold_input";
const char* PARAM_INPUT_4 = "threshold_input1";
// Interval between sensor readings. Learn more about timers: https://RandomNerdTutorials.com/esp32-pir-motion-sensor-interrupts-timers/
unsigned long previousMillis = 0;     
const long interval = 5000;    

// The Email Sending data object contains config and data to send
SMTPData smtpData;

void check_thresh(float temp, float hum ,float temp_thresh ,float hum_thresh)
{
  if(temp >= temp_thresh && inputMessage2 == "true" && !emailSent)
  {
    if(hum >= hum_thresh)
    {
      String emailMessage = String("Temperature and humidity are above threshold.\nCurrent temperature: ") + 
                          String(temp) + String("C, \n Current humidity: ")+String(hum) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }
    else
    {
      String emailMessage = String("Temperature is above threshold. Current temperature: ") + 
                          String(temp) + String("C, \n humidity is below threshold. Current humidity: ")+String(hum) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }    
  }
    // Check if temperature is below threshold and if it needs to send the Email alert
  else if(temp < temp_thresh && inputMessage2 == "true" && !emailSent){
    if(hum >= hum_thresh)
    {
      String emailMessage = String("Temperature is below threshold. Current temperature: ") + 
                          String(temp) + String("C, \n Humidity is above threshold.Current humidity: ")+String(hum) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }
    else
    {
      String emailMessage = String("Temperature and humidity are below threshold. Current temperature: ") + 
                          String(temp) + String("C, \n  Current humidity: ")+String(hum) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }
  }
  
}




void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
   if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
	while (1) {}
  }
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
  // Start the DHT11 sensor
  dht.begin();

  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Receive an HTTP GET request at <ESP_IP>/get?email_input=<inputMessage>&enable_email_input=<inputMessage2>&threshold_input=<inputMessage3>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // GET email_input value on <ESP_IP>/get?email_input=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      // GET enable_email_input value on <ESP_IP>/get?enable_email_input=<inputMessage2>
      if (request->hasParam(PARAM_INPUT_2)) {
        inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
        enableEmailChecked = "checked";
      }
      else {
        inputMessage2 = "false";
        enableEmailChecked = "";
      }
      // GET threshold_input value on <ESP_IP>/get?threshold_input=<inputMessage3>
      if (request->hasParam(PARAM_INPUT_3)) {
        inputMessage3 = request->getParam(PARAM_INPUT_3)->value();
      }
      if (request->hasParam(PARAM_INPUT_4)) {
        inputMessage4 = request->getParam(PARAM_INPUT_4)->value();
      }
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    Serial.println(inputMessage2);
    Serial.println(inputMessage3);
    Serial.println(inputMessage4);
    request->send(200, "text/html", "HTTP GET request sent to your ESP.<br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;   
    lastTemperature = readDHTTemperature();
    lastHumidity = readDHTHumidity();
    float temperature = lastTemperature.toFloat();  
    float humidity = lastHumidity.toFloat();   
  if(temperature >= inputMessage3.toFloat() && inputMessage2 == "true" && !emailSent)
  {
    if(humidity >= inputMessage4.toFloat())
    {
      String emailMessage = String("Temperature and humidity are above threshold.\nCurrent temperature: ") + 
                          String(temperature) + String("C, \n Current humidity: ")+String(humidity) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }
    else
    {
      String emailMessage = String("Temperature is above threshold. Current temperature: ") + 
                          String(temperature) + String("C, \n humidity is below threshold. Current humidity: ")+String(humidity) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }    
  }
    // Check if temperature is below threshold and if it needs to send the Email alert
  else if(temperature < inputMessage3.toFloat() && inputMessage2 == "true" && !emailSent){
    if(humidity >= inputMessage4.toFloat())
    {
      String emailMessage = String("Temperature is below threshold. Current temperature: ") + 
                          String(temperature) + String("C, \n Humidity is above threshold.Current humidity: ")+String(humidity) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }
    else
    {
      String emailMessage = String("Temperature and humidity are below threshold. Current temperature: ") + 
                          String(temperature) + String("C, \n  Current humidity: ")+String(humidity) + String("%");
      if(sendEmailNotification(emailMessage)) 
      {
      Serial.println(emailMessage);
      emailSent = true;
      }
      else {
        Serial.println("Email failed to send");
      } 
    }
  }
    // check_thresh(temperature,humidity,inputMessage3.toFloat() ,inputMessage4.toFloat() ) ;
    // Check if temperature is above threshold and if it needs to send the Email alert
    // if(temperature > inputMessage3.toFloat() && inputMessage2 == "true" && !emailSent){
      
    //   String emailMessage = String("Temperature above threshold. Current temperature: ") + 
    //                         String(temperature) + String("C");
    //   if(sendEmailNotification(emailMessage)) {
    //     Serial.println(emailMessage);
    //     emailSent = true;
    //   }
    //   else {
    //     Serial.println("Email failed to send");
    //   }    
    // }
    // // Check if temperature is below threshold and if it needs to send the Email alert
    // else if((temperature < inputMessage3.toFloat()) && inputMessage2 == "true" && emailSent) {
    //   String emailMessage = String("Temperature below threshold. Current temperature: ") + 
    //                         String(temperature) + String(" C");
    //   if(sendEmailNotification(emailMessage)) {
    //     Serial.println(emailMessage);
    //     emailSent = false;
    //   }
    //   else {
    //     Serial.println("Email failed to send");
    //   }
    // }
    // //************************************************
    // if(humidity > inputMessage4.toFloat() && inputMessage2 == "true" && emailSent){
    //   String emailMessage = String("Humidity above threshold. Current humidity: ") + 
    //                         String(humidity) + String("%");
    //   if(sendEmailNotification(emailMessage)) {
    //     Serial.println(emailMessage);
    //     emailSent = true;
    //   }
    //   else {
    //     Serial.println("Email failed to send");
    //   }    
    // }
    // // Check if Humidity is below threshold and if it needs to send the Email alert
    // else if(humidity < inputMessage4.toFloat() && inputMessage2 == "true" && emailSent){
    //   String emailMessage = String("Humidity below threshold. Current humidity: ") + 
    //                         String(humidity) + String("%");
    //   if(sendEmailNotification(emailMessage)) {
    //     Serial.println(emailMessage);
    //     emailSent = true;
    //   }
    //   else {
    //     Serial.println("Email failed to send");
    //   }    
    // }
  }
  //////////////////////////////////////////////////////
  Serial.print("DHT Temp = ");
  readDHTTemperature();
  //Serial.println(" *C");

  Serial.print("DHT Humidity = ");
  readDHTHumidity();
  //Serial.println(" %");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
    
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(102000));
  Serial.println(" meters");
    
  Serial.println();
  ///////////////////////////////////////////////////////
  delay(5000);

}

bool sendEmailNotification(String emailMessage){
  // Set the SMTP Server Email host, port, account and password
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);

  // For library version 1.2.0 and later which STARTTLS protocol was supported,the STARTTLS will be 
  // enabled automatically when port 587 was used, or enable it manually using setSTARTTLS function.
  //smtpData.setSTARTTLS(true);

  // Set the sender name and Email
  smtpData.setSender("ESP32", emailSenderAccount);

  // Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
  smtpData.setPriority("High");

  // Set the subject
  smtpData.setSubject(emailSubject);

  // Set the message with HTML format
  smtpData.setMessage(emailMessage, true);

  // Add recipients
  smtpData.addRecipient(inputMessage);

  smtpData.setSendCallback(sendCallback);

  // Start sending Email, can be set callback function to track the status
  if (!MailClient.sendMail(smtpData)) {
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    return false;
  }
  // Clear all data from Email object to free memory
  smtpData.empty();
  return true;
}

// Callback function to get the Email sending status
void sendCallback(SendStatus msg) {
  // Print the current status
  Serial.println(msg.info());

  // Do something when complete
  if (msg.success()) {
    Serial.println("----------------");
  }
}