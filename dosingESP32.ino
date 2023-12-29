const int motorPins[3] = {26, 25, 33}; // Пины PWM, подключенные к моторам на ESP32
const int motorSpeeds[3] = {180, 174, 169}; // Скорость для каждого мотора
const int motorCycleTime = 600; // Время цикла работы мотора в миллисекундах (600мс)

void setup() {
  Serial.begin(9600);
  // Инициализация пинов мотора как выходов
  for (int i = 0; i < 3; i++) {
    pinMode(motorPins[i], OUTPUT);
    ledcSetup(i, 1000, 8); // Настройка канала PWM, частота 1000 Гц, разрешение 8 бит
    ledcAttachPin(motorPins[i], i); // Привязка пина мотора к каналу PWM
  }
}

void loop() {
  // Ожидание названия функции, которую пользователь введет в Serial порт
    if (Serial.available() > 0) {
      Serial.println(" ");
    String command = Serial.readStringUntil('\n'); // Чтение команды из последовательного монитора
    command.toLowerCase(); //перевод в нижний регистр

    //запуск сценария добавления удобрений в соответствии с введенной командой 
    //Сценарий запускается с расчетом на 1 литр раствора
    //После перемешивания и анализа eC раствора, можно запустить функцию еще раз
    if (command == "firstroots") {
      firstRoots();
    } else if (command == "firsttrueleaves") {
      firstTrueLeaves();
    } else if (command == "growing") {
      growing();
    } else if (command == "preflowering") {
      preFlowering();
    } else if (command == "flowering") {
      flowering();
    }
  } 
  delay(500);
}

void runMotor(int motorIndex, float cycles) {
  int duration = motorCycleTime * cycles; // Расчет продолжительности работы мотора
  ledcWrite(motorIndex, motorSpeeds[motorIndex]); // Установка скорости мотора с использованием PWM
  Serial.print("Мотор № ");Serial.print(motorIndex);Serial.print(". Объем (мл): ");Serial.println(cycles);
  delay(duration); // Поддержание работы мотора в течение рассчитанной продолжительности
  ledcWrite(motorIndex, 0); // Остановка мотора
}

//первые корешки
void firstRoots() {
  Serial.println("Первые корешки");
  runMotor(0, 0.5);
  runMotor(1, 0.5); 
  runMotor(2, 0.5); 
}

//первые настоящие листья
void firstTrueLeaves() {
  Serial.println("Первые настоящие листья");
  runMotor(0, 1);
  runMotor(1, 1); 
  runMotor(2, 1); 
}

//рост
void growing() {
  Serial.println("Вегетация");
  runMotor(0, 1.8);
  runMotor(1, 1.2); 
  runMotor(2, 0.6); 
}

//предцвет
void preFlowering() {
  Serial.println("Предцвет");
  runMotor(0, 2);
  runMotor(1, 2); 
  runMotor(2, 1.5); 
}

//цветение
void flowering() {
  Serial.println("Цветение");
  runMotor(0, 0.8);
  runMotor(1, 1.6); 
  runMotor(2, 2.4); 
}
