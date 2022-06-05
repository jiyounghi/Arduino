#include <MyMusic.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo KFServo;
Servo CommonServo;
SoftwareSerial mySerial(5, 6);

#include "LiquidCrystal_I2C.h" //LCD
#include "DHT.h" //온습도
#include "emotion.h" //이모티콘
#include "PMsensor.h" //미세먼지 측정기

#define sensitivity  0.1  //먼지 센서의 민감도 수치
//민감도의 숫자가 클 경우 : 센서 값의 변화가 민감함
//민감도의 숫자가 작을 경우 : 센서 값의 변화가 둔함

#define DHTPIN 2  // 온습도센서 핀 번호
#define DHTTYPE DHT11
#define SLAVE_ADDR 8

const int sensorPin = A0;
const int sensorLED = 3;
float data = 0;
int rd; // Variable for received data

int redPin = 9;
int greenPin = 10;
int bluePin = 11;


byte b = 0;
// notes in the melody:
int melody[] = {NOTE_FS6 , NOTE_E6 , NOTE_D6 , NOTE_CS6 , NOTE_B5 , NOTE_A5 , NOTE_B5 , NOTE_CS6 , NOTE_FS6 , NOTE_E6 , NOTE_D6 , NOTE_CS6 , NOTE_G5 , NOTE_FS5 , NOTE_G5 , NOTE_CS6 , NOTE_D6 , NOTE_CS6 , NOTE_D6 , NOTE_D5 , NOTE_CS5 , NOTE_A5 , NOTE_E5 , NOTE_FS5 , NOTE_D5 , NOTE_D6 , NOTE_CS6 , NOTE_B5 , NOTE_CS6 , NOTE_FS6 , NOTE_A6 , NOTE_B6 , NOTE_B5 , NOTE_FS6 , NOTE_E6 , NOTE_G6 , NOTE_FS6 , NOTE_E6 , NOTE_D6 , NOTE_CS6 , NOTE_B5 , NOTE_A5 , NOTE_G5 , NOTE_B5 , NOTE_A5 , NOTE_G6 , NOTE_FS6 , NOTE_G6 , NOTE_A6 , NOTE_FS6 , NOTE_G6 , NOTE_A6 , NOTE_FS6 , NOTE_G6 , NOTE_A6 , NOTE_A5 , NOTE_B5 , NOTE_CS6 , NOTE_D6 , NOTE_E6 , NOTE_FS6 , NOTE_G6 , NOTE_FS6 , NOTE_D6 , NOTE_E6 , NOTE_FS6 , NOTE_FS5 , NOTE_G5 , NOTE_A5 , NOTE_B5 , NOTE_A5 , NOTE_G5 , NOTE_A5 , NOTE_FS5 , NOTE_G5 , NOTE_A5 , NOTE_G5 , NOTE_B5 , NOTE_A5 , NOTE_G5 , NOTE_FS5 , NOTE_E5 , NOTE_FS5 , NOTE_G5 , NOTE_E5 , NOTE_D5 , NOTE_E5 , NOTE_FS5 , NOTE_G5 , NOTE_A5 , NOTE_B5 , NOTE_G5 , NOTE_B5 , NOTE_A5 , NOTE_B5 , NOTE_CS6 , NOTE_D6 , NOTE_A5 , NOTE_B5 , NOTE_CS6 , NOTE_D6 , NOTE_E6 , NOTE_FS6 , NOTE_G6 , NOTE_A6 , NOTE_A6 , NOTE_FS6 , NOTE_G6 , NOTE_A6 , NOTE_FS6 , NOTE_E6 , NOTE_FS6 , NOTE_A5 , NOTE_B5 , NOTE_CS6 , NOTE_D6 , NOTE_E6 , NOTE_FS6 , NOTE_G6 , NOTE_FS6 , NOTE_D6 , NOTE_CS6 , NOTE_FS6 , NOTE_D5 , NOTE_E5 , NOTE_FS5 , NOTE_G5 , NOTE_FS5 , NOTE_D6 , NOTE_CS6 , NOTE_D6 , NOTE_B5 , NOTE_D6 , NOTE_CS6 , NOTE_B5 , NOTE_A5 , NOTE_G5 , NOTE_A5 , NOTE_G5 , NOTE_F5 , NOTE_G5 , NOTE_A5 , NOTE_B5 , NOTE_CS6 , NOTE_D6 , NOTE_B5 , NOTE_D6 , NOTE_CS6 , NOTE_D6 , NOTE_CS6 , NOTE_D6 , NOTE_E6 , NOTE_D6 , NOTE_CS6 , NOTE_D6 , NOTE_B5 , NOTE_CS6 , NOTE_D6};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8 , 16 , 16 , 8 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 16 , 8};

int PM_status = 0;
PMsensor PM;
LiquidCrystal_I2C lcd(0x27, 16, 2);
//LCD 이름 : 0x27 또는 0x3F 입력

DHT dht(DHTPIN, DHTTYPE);

//data는 미세먼지
////////////////////////측정값에 따른 이모티콘, LED 세팅///////////////////////////
void setEmoticon(float data) {
  if (data > 100) {                                 //Worst. 80 < data, Red LED
    lcd.createChar(4, topAngry1);
    lcd.createChar(5, topAngry2);
    lcd.createChar(6, bottomAngry1);
    lcd.createChar(7, bottomAngry2);
    //digitalWrite(9, HIGH);
    //digitalWrite(10, LOW);
    //digitalWrite(11, LOW);
    setColor(255, 0, 0); // red
    PM_status = 2;
  }
  else if (data > 50) {                           //Normal. 30 < data < 80, Yellow LED
    lcd.createChar(4, topSoSo1);
    lcd.createChar(5, topSoSo2);
    lcd.createChar(6, bottomSoSo1);
    lcd.createChar(7, bottomSoSo2);
    //digitalWrite(9, HIGH);
    //digitalWrite(10, HIGH);
    //digitalWrite(11, LOW);
    setColor(255, 255, 0);      // yellow
    PM_status = 1;
  }
  else {                                                     //Good. data < 30, Green LED
    lcd.createChar(4, topSmile1);
    lcd.createChar(5, topSmile2);
    lcd.createChar(6, bottomSmile1);
    lcd.createChar(7, bottomSmile2);
    //digitalWrite(9, LOW);
    //digitalWrite(10, HIGH);
    //digitalWrite(11, LOW);
    setColor(0, 255, 0);        //green
    PM_status = 0;
  }
}
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}
void Moter_KF94(int angle) {
  for (angle; angle <=  180; angle += 1) { // 0도에서 180도로 1도씩 이동
    KFServo.write(angle); // pos가 가진 각도의 위치로 이동
    delay(1); // 서보가 해당 위치에 도달할 때까지 30ms 대기
  }
}

void Moter_Common(int angle) {
  for (angle; angle <=  180; angle += 1) { // 0도에서 180도로 1도씩 이동
    CommonServo.write(angle); // pos가 가진 각도의 위치로 이동
    delay(1); // 서보가 해당 위치에 도달할 때까지 30ms 대기
  }
}
void setup_Moter(void) {
  KFServo.write(180);
  CommonServo.write(180);
}

void cannon() {
  //for (int thisNote = 64; thisNote < 158; thisNote++) {
  for (int thisNote = 48; thisNote < 80; thisNote++) {

    int noteDuration = 2000 / noteDurations[thisNote];
    tone(3, melody[thisNote], noteDuration);

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(3);
  }
}
void print_lcd(float data, byte temp, byte humi) {
  //----------------------------------------------------LCD 출력----------------------------------------------------//
  lcd.setCursor(0, 0);
  lcd.write(2);
  lcd.print(" ");
  lcd.print(data);

  if (data > 100) {
    lcd.print("ug");
  }
  else if (data > 10) {
    lcd.print(" ug");
  }
  else {
    lcd.print(" ug  ");
  }

  lcd.setCursor(0, 1);
  lcd.write((byte)0);
  lcd.print(" ");
  lcd.print((int)temp);
  lcd.write(3);

  lcd.print(" ");
  lcd.write(1);
  lcd.print(" ");
  lcd.print((int)humi);
  lcd.print("%");

  lcd.setCursor(13, 0);
  lcd.write(4);
  lcd.write(5);

  lcd.setCursor(13, 1);
  lcd.write(6);
  lcd.write(7);
}
void setup() {
  KFServo.attach(12); // 핀 2번에 연결된 서보를 서보 객체에
  CommonServo.attach(13); // 핀 2번에 연결된 서보를 서보 객체에

  lcd.init(); //LCD 초기화
  lcd.backlight(); // LCD의 백라이트 켜기

  lcd.createChar((byte)0, temp); //사용자 정의 문자 생성
  lcd.createChar(1, humi);
  lcd.createChar(2, dust);
  lcd.createChar(3, C);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  //pinMode(9, OUTPUT);
  //pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);

  /////(infrared LED pin, sensor pin)  /////
  PM.init(sensorLED, sensorPin);

  Serial.begin(9600);
  mySerial.begin(2400);
}
void receiveEvent() {
  Serial.print("rd = ");  Serial.println(rd); // Print value of incoming data

  if (rd) {
    setColor(255, 0, 0); // red
    Serial.print("Result");  Serial.println(rd); // Print value of incoming data
  }
}
void loop() {
  int recv_data = 0;
  byte temp = dht.readTemperature();
  byte humi = dht.readHumidity();

  recv_data = mySerial.read();
  if (isnan(humi) || isnan(temp) ) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("dht_temp  = "); Serial.print(temp);  Serial.println();
  Serial.print("dht_humi = "); Serial.print(humi); Serial.println();
  //Serial.println();
  int err = PMsensorErrSuccess;
  if ((err = PM.read(&data, true, sensitivity)) != PMsensorErrSuccess) {
    Serial.print("data Error = ");
    Serial.println(err);
    delay(1500);
    return;
  }

  Serial.println(data);
  setEmoticon(data);
  print_lcd(data, temp, humi);

  if (recv_data == '1') {
    setColor(0, 0, 255); // blue
    if (PM_status == 2) {     // RED
      Moter_KF94(0);
    }
    if (PM_status == 1) {     // Yellow
      Moter_KF94(0);
    }
    if (PM_status == 0) {     // Green
      Moter_Common(0);
    }
    Serial.println("TEST");
    //cannon();
    //Serial.println(recv_data);
  } else {
    setup_Moter();
  }

  delay(1000);
}




///미세먼지 측정값에 따라 LCD에 출력되는 이모티콘과 RGB가 다르다.
//80 이상시 화난 표정, 빨간 LED
//50 이상시 80 미만시 그저그런 표정, 노란 LED
//50 미만시 웃는 표정, 초록 LED

//LCD 뿐만 아니라 시리얼 모니터에서도 온도, 습도, 미세먼지 측정 볼 수 있음있음
