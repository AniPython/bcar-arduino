/*
   依赖库:
    MPU6050_tockn
    ArduinoJson
*/

#include <MPU6050_tockn.h>
#include <Wire.h>
#include <WiFi.h> 
#include <WebServer.h>
#include <ArduinoJson.h>

const int led = 8;

// wifi 名称和路由
const char *ssid = "小亦站";
const char *password = "88889999";

// 设置静态 IP 配置
IPAddress local_ip(192, 168, 2, 180);  // 设置固定 IP 地址
IPAddress gateway(192, 168, 2, 1);     // 路由器网关地址
IPAddress subnet(255, 255, 255, 0);    // 子网掩码

WebServer server(80);

#define IN1 1
#define IN2 2
#define IN3 4
#define IN4 3

int sdaPin = 6, sclPin = 5; //自定义eps32的I2C引脚

float kp = 12, ki = 0.2, kd = 0.5;  // pid 参数预设
float angleOffset = 0;  // 角度补偿, 平衡站立时 mpu6050 实测角度, 参数值: -3 ~ 3
float targetAngle = 0;  // 目标角度, 直立状态下为 0, 更改可以前进或后退, 参数值: -3 ~ 3
int leftMotorPwmOffset = 32, rightMotorPwmOffset = 32;  // 马达pwm补偿, 马达达到一定电压才开始转动, 参数值: 0 ~ 80
float turnKp = 0;  // 转向 pid 参数中的 turnKp , 简化省略了 turnKi 和 turnKd 参数, turnKp 参数值: 0.2 ~ 1
float turnSpeed = 0;  // 转向角速度, 参数值: 100 ~ 600

// 全局变量, 全局创建一次, 在函数体内进行更改值, 避免在函数体内频繁下创建与销毁
float bias, integrate;  // 角度偏差，偏差积分
float angleY, gyroY, gyroZ;  // 陀螺仪 mpu6050 的输出值, angleY: Y轴角度; gyroY: Y轴角速度; gyroZ: Z轴角速度
int verticalPwm, turnPwm, L_Pwm, R_Pwm;  //各种PWM计算值

String indexHtml = "<!DOCTYPE html><html lang=\"en\"><head>    <meta charset=\"UTF-8\">    <title>PID Parameter Settings</title>    <style>        body {            font-family: \"Courier New\", Courier, monospace;        }        .comments-text {            color: darkgreen;            font-size: 12px;        }    </style></head><body><h1>PID 参数设置</h1><div>    <label for=\"kp\">Kp:</label>    <input type=\"text\" id=\"kp\" value=\"0\" readonly>    <button onclick=\"resetValue('kp', 'kp')\">Set 0</button>    <button onclick=\"changeValue('kp', 1.0, 'kp')\">+</button>    <button onclick=\"changeValue('kp', -1.0, 'kp')\">-</button>    <span class=\"comments-text\">[15]</span></div><br><div>    <label for=\"ki\">Ki:</label>    <input type=\"text\" id=\"ki\" value=\"0\" readonly>    <button onclick=\"resetValue('ki', 'ki')\">Set 0</button>    <button onclick=\"changeValue('ki', 0.1, 'ki')\">+</button>    <button onclick=\"changeValue('ki', -0.1, 'ki')\">-</button>    <span class=\"comments-text\">[0.5]</span></div><br><div>    <label for=\"kd\">Kd:</label>    <input type=\"text\" id=\"kd\" value=\"0\" readonly>    <button onclick=\"resetValue('kd', 'kd')\">Set 0</button>    <button onclick=\"changeValue('kd', 0.1, 'kd')\">+</button>    <button onclick=\"changeValue('kd', -0.1, 'kd')\">-</button>    <span class=\"comments-text\">[1.0]</span></div><br><h1>角度设置</h1><div>    <label for=\"angleOffset\">角度补偿(angleOffset):</label>    <input type=\"text\" id=\"angleOffset\" value=\"0\" readonly>    <button onclick=\"resetValue('angleOffset', 'angleOffset')\">Set 0</button>    <button onclick=\"changeValue('angleOffset', 0.1, 'angleOffset')\">+</button>    <button onclick=\"changeValue('angleOffset', -0.1, 'angleOffset')\">-</button>    <span class=\"comments-text\">[-3 ~ 3]</span></div><br><div>    <label for=\"targetAngle\">目标角度(targetAngle):</label>    <input type=\"text\" id=\"targetAngle\" value=\"0\" readonly>    <button onclick=\"resetValue('targetAngle', 'targetAngle')\">Set 0</button>    <button onclick=\"changeValue('targetAngle', 0.2, 'targetAngle')\">+</button>    <button onclick=\"changeValue('targetAngle', -0.2, 'targetAngle')\">-</button>    <span class=\"comments-text\">[-3 ~ 3] 控制前进后退</span></div><br><h1>转向设置</h1><div>    <label for=\"turnKp\">转速比例(turnKp):</label>    <input type=\"text\" id=\"turnKp\" value=\"0\" readonly>    <button onclick=\"resetValue('turnKp', 'turnKp')\">Set 0</button>    <button onclick=\"changeValue('turnKp', 0.1, 'turnKp')\">+</button>    <button onclick=\"changeValue('turnKp', -0.1,'turnKp')\">-</button>    <span class=\"comments-text\">[0.2 ~ 1] 控制转向灵敏度</span></div><br><div>    <label for=\"turnSpeed\">转向转速(turnSpeed):</label>    <input type=\"text\" id=\"turnSpeed\" value=\"0\" readonly>    <button onclick=\"resetValue('turnSpeed', 'turnSpeed')\">Set 0</button>    <button onclick=\"changeValue('turnSpeed', 100, 'turnSpeed')\">+</button>    <button onclick=\"changeValue('turnSpeed', -100,'turnSpeed')\">-</button>    <span class=\"comments-text\">[100 ~ 600] 控制转向灵敏度</span></div><br><h1>马达 PWM 补偿设置</h1><div>    <label for=\"leftMotorPwmOffset\">左马达 PWM 补偿:</label>    <input type=\"text\" id=\"leftMotorPwmOffset\" value=\"0\" readonly>    <button onclick=\"resetValue('leftMotorPwmOffset', 'leftMotorPwmOffset')\">Set 0</button>    <button onclick=\"changeValue('leftMotorPwmOffset', 1, 'leftMotorPwmOffset')\">+</button>    <button onclick=\"changeValue('leftMotorPwmOffset', -1,'leftMotorPwmOffset')\">-</button>    <span class=\"comments-text\">[30 ~ 60]</span></div><br><div>    <label for=\"rightMotorPwmOffset\">右马达 PWM 补偿:</label>    <input type=\"text\" id=\"rightMotorPwmOffset\" value=\"0\" readonly>    <button onclick=\"resetValue('rightMotorPwmOffset', 'rightMotorPwmOffset')\">Set 0</button>    <button onclick=\"changeValue('rightMotorPwmOffset', 1, 'rightMotorPwmOffset')\">+</button>    <button onclick=\"changeValue('rightMotorPwmOffset', -1,'rightMotorPwmOffset')\">-</button>    <span class=\"comments-text\">[30 ~ 60]</span></div></body><script>    const esp32Address = \"http://192.168.2.180\";    function updateParam(param, value) {        fetch(`${esp32Address}/set_params?param=${param}&value=${value}`, {method: \"POST\"})            .then(response => response.text())            .then(data => {                console.log(data);                data = JSON.parse(data);                updateDataToPage(data.data);            })            .catch(error => console.error(\"Error:\", error));    }    function changeValue(param, step, inputId) {        const inputElement = document.getElementById(inputId);        let currentValue = parseFloat(inputElement.value);        let newValue = currentValue + step;        if ([\"kp\", \"ki\", \"kd\", \"turnKp\", \"leftMotorPwmOffset\", \"rightMotorPwmOffset\"].includes(param)) {            if (newValue < 0) newValue = 0;        }        updateParam(param, newValue);    }    function resetValue(param, inputId) {        document.getElementById(inputId).value = \"0.0\";        updateParam(param, 0.0);    }    function updateDataToPage(data) {        for (const [key, value] of Object.entries(data)) {            const inputElement = document.getElementById(key);            if (inputElement) {                inputElement.value = value;            }        }    }    function getCurrentParams() {        fetch(`${esp32Address}/get_params`, {method: \"GET\"})            .then(response => response.text())            .then(data => {                data = JSON.parse(data);                console.log(data);                updateDataToPage(data.data);            })            .catch(error => console.error(\"Error:\", error));    }    window.onload = () => {        getCurrentParams();    }</script></html>";

MPU6050 mpu6050(Wire);    //实例化mpu6050对象

// StaticJsonDocument<360> okRespond;
// StaticJsonDocument<300> currentParams; 
JsonDocument okRespond;
JsonDocument currentParams; 
String okRespondJsonStr;

String getCurrentParams (String msg) {

  currentParams["kp"] = kp;
  currentParams["ki"] = ki;
  currentParams["kd"] = kd;
  currentParams["targetAngle"] = targetAngle;
  currentParams["angleOffset"] = angleOffset;
  currentParams["turnKp"] = turnKp;
  currentParams["turnSpeed"] = turnSpeed;
  currentParams["leftMotorPwmOffset"] = leftMotorPwmOffset;
  currentParams["rightMotorPwmOffset"] = rightMotorPwmOffset;

  okRespond["msg"] = msg;
  okRespond["status"] = "ok";
  okRespond["data"] = currentParams;

  serializeJson(okRespond, okRespondJsonStr);
  return okRespondJsonStr;
}

void motor(int left_EN, int right_EN)   //马达输出函数
{
  left_EN = constrain(left_EN, -255, 255);
  right_EN = constrain(right_EN, -255, 255);  //限定PWM区间在-255~255
  if (left_EN >= 0)
  {
    analogWrite(IN1, left_EN);
    analogWrite(IN2, 0);
  }
  if (left_EN < 0)
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0 - left_EN);
  }
  if (right_EN >= 0)
  {
    analogWrite(IN3, right_EN);
    analogWrite(IN4, 0);
  }
  if (right_EN < 0)
  {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0 - right_EN);
  }
}


void vertical_pwm_calculation() //直立PMW计算
{
  angleY = mpu6050.getAngleY();
  gyroY = mpu6050.getGyroY();
  bias = angleY - angleOffset - targetAngle; // 计算角度偏差。bias为小车角度是静态平衡角度的差值。
  integrate += bias; //偏差的积分，integrate为全局变量，一直积累。
  integrate = constrain(integrate, -1000, 1000); //限定误差积分的最大和最小值
  /*==直立PID计算PWM==通过陀螺仪返回数据计算，前倾陀螺仪Y轴为正，后仰陀螺仪Y轴为负。
    前倾车前进，后仰车后退，保持直立。但可能为了直立，车会随时移动。*/
  verticalPwm = kp * bias + ki * integrate + kd * gyroY;
}

void turn_pwm_calculation()  //转向PMW计算
{
  gyroZ = mpu6050.getGyroZ(); //获取陀螺仪Z轴角速度
  turnPwm = turnKp * (turnSpeed - gyroZ);
  turnPwm = constrain(turnPwm, -180, 180);
}

void combine_pwm() //马达PWM控制函数
{

  // 马达补偿
  if (verticalPwm >= 0)
  {
    L_Pwm = verticalPwm + leftMotorPwmOffset;
    R_Pwm = verticalPwm + rightMotorPwmOffset;
  }
  if (verticalPwm < 0)
  {
    L_Pwm = verticalPwm - leftMotorPwmOffset;
    R_Pwm = verticalPwm - rightMotorPwmOffset;
  }

  // 转向控制
  L_Pwm -= turnPwm;
  R_Pwm += turnPwm;

  // 限制输出
  L_Pwm = constrain(L_Pwm, -255, 255);
  R_Pwm = constrain(R_Pwm, -255, 255);

}

// 处理跨域请求
void handleCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  server.send(204);
}

void handleRoot() {
  digitalWrite(led, 1);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/html", indexHtml);
  digitalWrite(led, 0);
}

void handleSetParams() {

  if (server.hasArg("param") && server.hasArg("value")) {
      String param = server.arg("param");
      float value = server.arg("value").toFloat();

      // Update the corresponding PID parameter
      if (param == "kp") {
        kp = value;
      } else if (param == "ki") {
        ki = value;
      } else if (param == "kd") {
        kd = value;
      } else if (param == "angleOffset") {
        angleOffset = value;
      } else if (param == "targetAngle") {
        targetAngle = value;
      } else if (param == "turnKp") {
        turnKp = value;
      } else if (param == "turnSpeed") {
        turnSpeed = value;
      } else if (param == "leftMotorPwmOffset") {
        leftMotorPwmOffset = value;
      } else if (param == "rightMotorPwmOffset") {
        rightMotorPwmOffset = value;
      } else {
        String jsonResponse = "{\"status\":\"error\",\"msg\":\"Invalid parameter name\"}";
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(400, "application/json", jsonResponse);
        return;
      }

      String jsonResponse = getCurrentParams("set param success");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "application/json", jsonResponse);

      // Debugging
      Serial.println(jsonResponse);
    } else {
      // Missing parameters
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(400, "application/json", "{\"status\":\"error\",\"msg\":\"handleSetParams() Params error\"}");
    }

}

void handleGetParams() {

  String jsonResponse = getCurrentParams("get param success");

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", jsonResponse);

  // Debugging
  Serial.println(jsonResponse);
}

void setup() {  //初始化

  Serial.begin(9600);

  /*
    ********************************************
    服务器设置
  */
  // 设置静态 IP 地址
  if (!WiFi.config(local_ip, gateway, subnet)) {
    Serial.println("配置静态 IP 失败!");
  }

  // 连接到 Wi-Fi 网络
  Serial.print("连接到 WiFi网络: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // 等待连接
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // 输出连接的 IP 地址
  Serial.println();
  Serial.print("已连接到 WiFi, IP 地址：");
  Serial.println(WiFi.localIP());

  // 允许跨域请求
  server.on("/cors", HTTP_OPTIONS, handleCors);

  // 配置 Web 服务器的处理器
  server.on("/", handleRoot);
  server.on("/get_params", handleGetParams);
  server.on("/set_params", HTTP_POST, handleSetParams);

  server.begin();
  /*
    ********************************************
    服务器设置结束
  */

  Wire.begin(sdaPin, sclPin);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
  motor(0, 0);
  delay(10); //循环前延时，确保各种初始和准备完成
}

void loop() {
  // WIFI调试+控制
  server.handleClient();

  // 陀螺仪刷新
  mpu6050.update();

  // 直立环 PWM 计算, 更新全局变量: verticalPwm
  vertical_pwm_calculation(); 

  // 转向环 PWM 计算, 更新全局变量: turnPwm
  turn_pwm_calculation();

  // 结合 verticalPwm, turnPwm, leftMotorPwmOffset, rightMotorPwmOffset 更新全局变量: L_Pwm, R_Pwm
  combine_pwm();

  // 马达输出
  if (angleY > 45 || angleY < -45) // 倾角过大（车倒下时），停止马达输出
  {
    motor(0, 0);
  } else {
    motor(L_Pwm, R_Pwm); // 正常运转
  }
  
}
