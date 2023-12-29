const int motorPins[] = {26, 25, 33}; // Пины для подключения к каналам PWM
const int numMotors = 3; // Количество моторов
int motorSpeeds[numMotors] = {210, 178, 170};
int durations[numMotors] = {0, 0, 0};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < numMotors; i++) {
    pinMode(motorPins[i], OUTPUT);
    ledcSetup(i, 1000, 8); // Настройка канала PWM
    ledcAttachPin(motorPins[i], i); // Привязка канала PWM к пину
  }
}

void setMotorSpeed(int motor, int speed) {
  motorSpeeds[motor] = speed;
}

void applyMotorSpeeds() {
  for (int i = 0; i < numMotors; i++) {
    ledcWrite(i, motorSpeeds[i]);
    Serial.println (motorSpeeds[i]); // Установка скорости мотора
  }
}

void stopMotors() {
  for (int i = 0; i < numMotors; i++) {
    motorSpeeds[i] = 0;
  }
  applyMotorSpeeds(); // Остановка моторов
}

void loop() {
  // Ожидание команд из консоли
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("SPEED")) {
      command.remove(0, 6); // Удалить "SPEED " из команды
      int motor = 0;
      while (command.length() > 0) {
        int commaPos = command.indexOf(",");
        if (commaPos != -1) {
          // Если найдена запятая, получите значение до запятой
          int speed = command.substring(0, commaPos).toInt();
          setMotorSpeed(motor, speed);
          command.remove(0, commaPos + 1); // Удалите обработанный фрагмент
          motor++;
        } else {
          // Если больше нет запятых, то это последнее значение
          int speed = command.toInt();
          setMotorSpeed(motor, speed);
          break;
        }
      }
      Serial.println("Motor speeds set.");
    } else if (command.startsWith("DURATION")) {
      int duration = command.substring(9).toInt();
      Serial.println("Received Duration: " + String(duration) + " seconds");
      applyMotorSpeeds(); // Установка скоростей моторов
      delay(duration * 1); // Подождите указанное количество секунд
      stopMotors();
    }
  }
delay(500);
}























// const int motorPin1 = 26; // Пин для подключения Mosfet модуля к первому мотору
// const int motorPin2 = 25; // Пин для подключения Mosfet модуля ко второму мотору
// const int motorPin3 = 33; // Пин для подключения Mosfet модуля к третьему мотору
// const int motorRunTime = 600; // Время работы мотора в миллисекундах

// void runMotor(int motorPin, int motorTime) {
//   analogWrite(motorPin, 255); // Включаем мотор на максимальной скорости (255 для Arduino)
//   delay(motorTime); // Ждем указанное время
//   analogWrite(motorPin, 0); // Выключаем мотор
// }

// //первые корешки
// void dosingFirstRoots(int liter) {
//   int motor1Time = liter * 0.5 * motorRunTime;
//   int motor2Time = liter * 0.5 * motorRunTime;
//   int motor3Time = liter * 0.5 * motorRunTime;

//   runMotor(motorPin1, motor1Time);
//   runMotor(motorPin2, motor2Time);
//   runMotor(motorPin3, motor3Time);
// }

// //вегетация
// void dosingVegit(int liter) {
//   int motor1Time = liter * 1.8 * motorRunTime;
//   int motor2Time = liter * 1.2 * motorRunTime;
//   int motor3Time = liter * 0.6 * motorRunTime;

//   runMotor(motorPin1, motor1Time);
//   runMotor(motorPin2, motor2Time);
//   runMotor(motorPin3, motor3Time);
// }

// void setup() {
//   pinMode(motorPin1, OUTPUT);
//   pinMode(motorPin2, OUTPUT);
//   pinMode(motorPin3, OUTPUT);
// }

// void loop() {
//   // Вызываем функцию dosingFirstRoots и передаем значение INT (на сколько литров замешать раствор, например 2)
//   dosingFirstRoots(5);
//   //dosingVegit (2);
//   delay(1000); // Ждем 1 секунду перед следующим запуском
// }

