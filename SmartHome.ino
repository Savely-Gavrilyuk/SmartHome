/*
Система "SmartHome" оснащена ЖК-дисплеем на который выводится текущая  информация с датчика темпрературы в градусах целься и влажности в процентах. 
Также выводится информация с датчика концентрации газов в миллионных долях отношения одного газа к другому.
Система сигнализирует, если концентрация вредных примесей в воздухе преодолеет пороговое значение.
Во второй строке на дисплее выводится ярокость матрицы в процентном отношении, а также состояние двери. Якрость матрицы изменяется патенциометром
Система оснащена 2умя светодиодами и динамиком для индикации событий.
*/

#define REMOTEXY_MODE__HARDSERIAL //Использование аппаратного UART
#include <RemoteXY.h>             //Библиотека для приложения RemoteXY
#include <LiquidCrystal_I2C.h>    //Библиотека для ЖК-диспелея по I2C
#include <Wire.h>                 //Билиотека для I2C
#include <PCF8574.h>              //Библиотека для расширителя пинов PCF8574
#include <DHT.h>                  //Библиотека датчика DHT11 
#include <LedMatrix.h>            //Библиотека для матрицы MAX7219
#include <MFRC522.h>              //Библиотека для мети RFID-RC522
#include <SPI.h>                  //Билиотека для SPI
#include <Servo.h>                //Библиотека для сервопривода 

#define REMOTEXY_SERIAL Serial           //UART для Bluetooth модуля
#define REMOTEXY_SERIAL_SPEED 9600       //Скорость передачи данных
#define REMOTEXY_ACCESS_PASSWORD "smart" //Пароль для входа в приложение

PCF8574 expander;   //Создаем бъект класса для расширителя пинов
#define LedRed 0    //Пин светодиода
#define LedGreen 1  //Пин светодиода
#define ZummerPin 2 //Пин зуммера
#define FlamePin 3  //Пин датчика пламени

#define SoundPin 2                //Пин датчика звука на плате
#define LigthPin 3                //Пин светодиода (свет)
volatile bool svet=0;             //Переменная для хранения состояния светодиода
volatile unsigned long debounce;  //Переиенная для хранения времени предыдущего прерывания

LiquidCrystal_I2C lcd(0x27, 16, 2);                                          //Указываем адрес и размер дисплея
byte charDisp[]={B11100,B10100,B11100,B00000,B00000,B00000,B00000,B00000};   //Массив для рисования знака градуса
byte charDisp1[]={B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000};  //Массив зануления ячейки матрицы

#define SS_PIN 10              //Пин шины данных RFID модуля по SPI
#define RST_PIN 9              //Пин сброса
MFRC522 rfid(SS_PIN, RST_PIN); //Создаем объект класса для RFID
#define brelok 0xDAC49280      //ID брелка
unsigned long ID=0;            //Переменная для хранения ID метки

LedMatrix mtrx(8);                                                                                                //Создаем объект класса для матрицы
const byte krest[] PROGMEM={B11000011,B11100111,B01111110,B00111100,B00111100,B01111110,B11100111,B11000011};     //Массив для рисования креста
const byte strelka[] PROGMEM={B00011000,B00110000,B01100000,B11111111,B11111111,B01100000,B00110000,B00011000};   //Массив для рисования стрелки

Servo dverka; //Создаем объект класса для серводвигателя

#define EchoPin 5                                                                                               //Пин приема сигнала УЗ-датчика
#define TrigPin 4                                                                                               //Пин отправки сигнала УЗ-датчика
unsigned long duration;                                                                                         //Переменная для хранения времени
const byte strela[] PROGMEM={B00011000,B00011000,B00011000,B10011001,B11011011,B01111110,B00111100,B00011000};  //Массив для рисования стрелки

#define DHTPIN 7       //Пин датчика влажности и температуры
DHT dht(DHTPIN,DHT11); //Создаем объект класса для датчика влажности и температуры

#define MQPin A0    //Пин датчика газа
#define PotenPin A1 //Пин потенциометра
#define IKPin A2    //Пин датчика движения
#define PhotoPin A3 //Пин фоторезистора

// === Код программы для смартфона ===
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] =   // 133 байта
  { 255,1,0,2,0,126,0,16,31,1,70,20,9,31,9,9,4,37,0,70,
  20,45,31,9,9,4,37,0,10,48,24,59,15,15,4,1,31,79,78,0,
  31,79,70,70,0,129,0,1,25,61,4,24,208,146,208,190,208,183,208,179,
  208,190,209,128,208,176,208,189,208,184,208,181,33,32,32,32,32,32,32,32,
  32,32,32,32,32,208,163,209,130,208,181,209,135,208,186,208,176,32,208,179,
  208,176,208,183,208,176,33,0,129,0,22,53,21,4,24,208,148,208,178,208,
  184,208,182,208,181,208,189,208,184,208,181,33,0 };                              
struct 
 {
   uint8_t pushSwitch_1;  //Переменная для хранения состояния кнопки вкл/выкл датчика движения
   uint8_t led_1;         //Переменная для хранения состояния индикации вкл/выкл датчика 
   uint8_t led_2;         //Переменная для хранения состояния индикации вкл/выкл датчика 
   uint8_t connect_flag;
 }
 RemoteXY;
#pragma pack(pop)

void setup() 
{
  RemoteXY_Init();                         //Инициализируем RemoteXY
  rfid.PCD_Init();                         //Инициализируем RFID
  lcd.init();                              //Инициализируем дисплей
  SPI.begin();                             //Запускаем SPI
  dht.begin();                             //Запускаем DHT11
  expander.begin(0x20);                    //Запускаем расширитель портов
  Serial.begin(9600);                      //Запускаем последовательный порт
  lcd.backlight();                         //Включаем подсветку дисплея
  lcd.clear();                             //Очищаем дисплей
  lcd.setCursor(4,0);                      //Устанавливаем курсор 
  lcd.print("Welcome!");                   //Выводим текст на экран
  lcd.setCursor(3,1);                      //Устанавливаем курсор 
  lcd.print("SmartHome");                  //Выводим текст на экран
  lcd.createChar(0,charDisp);              //Создаем символ градуса
  lcd.createChar(1,charDisp1);             //Создаем символ зануления ячейки дисплея
  mtrx.setIntensity(1);                    //Устанавливаем начальную яркость матрицы
  dverka.attach(6);                        //Пин серводвигателя 
  dverka.write(117);                       //Устанавливаем начальное положение в градусах
  //Конфигурируем пины
  pinMode(TrigPin,OUTPUT);
  pinMode(EchoPin,INPUT);
  pinMode(IKPin,INPUT);
  pinMode(PhotoPin,INPUT);
  pinMode(MQPin,INPUT);
  pinMode(SoundPin,INPUT_PULLUP); 
  pinMode(LigthPin,OUTPUT);
  digitalWrite(LigthPin,LOW);
  expander.pinMode(LedRed,OUTPUT);
  expander.pinMode(LedGreen,OUTPUT);
  expander.pinMode(ZummerPin,OUTPUT);
  expander.pinMode(FlamePin,INPUT_PULLUP);
  expander.digitalWrite(LedRed,LOW);
  expander.digitalWrite(LedGreen,LOW);
  expander.digitalWrite(ZummerPin,HIGH);
  attachInterrupt(0, sound, CHANGE);        //Обработчик прерывания для датчика звука
  delay(3000);
  lcd.clear();                              //Очищаем матрицу
}

void loop() 
{
  RemoteXY_Handler(); //Вызов обработчика для приложения
  
  lcd.setCursor(4,1);        //Устанавливаем курсор
  lcd.print("Door Close ");  //Выводим текст на экран
  lcd.write(1);              //Зануляем ячейку

  tempVlagDet();  //Считываем и выводим показания датчика температуры и влажности на экран
  matrReg();      //Считываем и выводим значения яркости матрицы на экран
  gasDet();       //Считываем и выводим показания датчика газа на экран и дисплей смартфона (при утечке)
  flameDet();     //Детектируем значения с датчика и выводим на экран дисплей смартфона в случае срабатывания
  if(RemoteXY.pushSwitch_1) movDec();  //Если на смартфоне нажата кнопка включения датчика движения, включаем его
  autoLight();    //Считываем значения с фоторезистора и при низком уровне освещенности включаем диод
  closely();      //Считываем значения с уз-датчика и выводим стрелку при соответствующих значениях
  readMark();     //Считываем RFID-метку

  if (ID == brelok)                             //Если ID совподает со ID брелка
  {                                                                 
    expander.digitalWrite(LedGreen,HIGH);       //Зажигаем светодиод
    lcd.setCursor(4,1);                         //Устанавливаем курсор
    lcd.print("Door Open! ");                   //Выводим на дисплей
    for(int i=0;i<2;i++)                        //Звуковая индикация                        
    {
      expander.digitalWrite(ZummerPin,LOW);     //Включаем зуммер
      delay(100);
      expander.digitalWrite(ZummerPin,HIGH);    //Выключаем зуммер
      delay(100);
    }
    for(int i=0;i<8;i++)                        //Выводим стрелу на матрицу
    {
      mtrx.setRow(i,pgm_read_byte(&strelka[i]));
      delay(50);
    }
    for(int i=117;i>=57;i--)                    //Открываем дверь
    {
      dverka.write(i);
      delay(15);
    }
    delay(3000);
    for(int i=57;i<=117;i++)                    //Закрываем дверь
    {
      dverka.write(i);
      delay(15);
    }
    mtrx.clear();                               //Очищаем матрицу
    expander.digitalWrite(LedGreen,LOW);        //Тушим светодиод
  }
  else if (ID != brelok && ID != 0)             //Если ID не совподает со ID брелка
  {
    expander.digitalWrite(LedRed,HIGH);         //Зажигаем светодиод 
    lcd.setCursor(4,1);                         //Устанавливаем курсор
    lcd.print("Invalid key!");                  //Выводим на дисплей
    for(int i=0;i<8;i++)                        //Выводим крест на матрицу
    {
      mtrx.setRow(i,pgm_read_byte(&krest[i]));
      delay(50);
    }
    expander.digitalWrite(ZummerPin,LOW);       //Включаем зуммер
    delay(500);
    expander.digitalWrite(ZummerPin,HIGH);      //Выключаем зуммер
    delay(500);
    mtrx.clear();                               //Очищаем матрицу
    expander.digitalWrite(LedRed,LOW);          //Тушим светодиод  
  }
  ID=0;
}

  void tempVlagDet()               
  {
    int h=dht.readHumidity();    //Считываем значение влажности
    int t=dht.readTemperature(); //Считываем значение температуры
    lcd.home();                  //Устанавливаем курсор в начальное положение
    lcd.print(t);                //Выводим зачение температуры на дисплей
    lcd.write(0);                
    lcd.print("C");             
    lcd.print("/");             
    lcd.print(h);                //Выводим зачение влажности на дисплей
    lcd.print("%");
  }

  void matrReg()                              
  {
    int brightness=analogRead(PotenPin);     //Считываем значение с потенциометра
    brightness=constrain(brightness,0,1000); //Ограничиваем значения
    brightness=map(brightness,0,950,0,10);   //Маштабируем значения 
    mtrx.setIntensity(brightness);           //Устанавливаем яркость матрицы
    brightness=map(brightness,0,10,0,100);   //Маштабируем значения в проценты
    lcd.setCursor(0,1);                      //Устанавливаем курсор
    lcd.print(brightness);                   //Выводим значение на дисплей
    if(brightness==0) mtrx.shutdown();       //Если значение равно нулю, то выключаем матрицу
    else  mtrx.wakeup();                     //Если нет, то включаем
    if(brightness<10)                        //Если значение яркости меньше 10%
    {
      lcd.setCursor(2,1);                    //Устанавливаем курсор
      lcd.write(1);                          //Зануляем ячейку
      lcd.setCursor(1,1);                    //Устанавливаем курсор
      lcd.print("%");                        //Выводим на дисплей
    }
    else if(brightness<100)                  //Если значение яркости меньше 100%
    { 
      lcd.setCursor(3,1);                    //Устанавливаем курсор
      lcd.write(1);                          //Зануляем ячейку
      lcd.setCursor(2,1);                    //Устанавливаем курсор
      lcd.print("%");                        //Выводим на дисплей
    }
    else                                     
    {
      lcd.setCursor(3,1);                    //Устанавливаем курсор
      lcd.print("%");                        //Выводим на дисплей
    }    
  }

  void gasDet()    
  {
    int MQ=analogRead(MQPin);                //Считываем значения с датчика газа
    lcd.setCursor(9,0);                      //Устанавливаем курсор
    lcd.print(MQ);                           //Выводим значение на дисплей
    lcd.print("ppm");                        //Выводим на дисплей
    if(MQ>600)                               //Если больше порога
    {
      RemoteXY.led_2=1;                      //Включаем индикатор на смартфоне
      lcd.setCursor(4,1);                    //Устанавливаем курсор
      lcd.print("Gas Leak!!!");              //Выводим на дисплей
      indication(50);                        //Свето-звуковая индикация
    }
    else RemoteXY.led_2=0;                   //Выключаем индикатор на смартфоне             
  }

  void flameDet()
  {
    bool Flame=expander.digitalRead(FlamePin); //Считываем значение с датчика пламени
    if(!Flame)                                 //Если датчик зафиксировал пламя
    {
      RemoteXY.led_1=1;                        //Включаем индикатор на смартфоне
      lcd.setCursor(4,1);                      //Устанавливаем курсор
      lcd.print("Fire!!!");                    //Выводим на дисплей
      lcd.write(1);                            //Зануляем ячейку
      lcd.write(1);                            //Зануляем ячейку
      lcd.write(1);                            //Зануляем ячейку
      indication(300);                         //Свето-звуковая индикация
    }
    else RemoteXY.led_1=0;                     //Выключаем индикатор на смартфоне 
  }

  void movDec()
  {
    if(digitalRead(IKPin)==HIGH)             //Если движение замечено
    {  
      lcd.setCursor(4,1);                    //Устанавливаем курсор
      lcd.print("Movement!!!");              //Выводим на дисплей
      indication(100);                       //Свето-звуковая индикация
    }
  }

  void autoLight()
  {
    int Photo=analogRead(PhotoPin);                       //Считываем значения с фоторезистора 
    if(Photo>550) digitalWrite(LigthPin,HIGH);            //Если больше порога, зажигаем светодиод
    if(Photo<550 && svet==0) digitalWrite(LigthPin,LOW);  //Если меньше порога и состояние светодиода на датчике звука - выключен, тушим светодиод
  }

  void closely()
  {
    if(distance()<10)                                       //Если расстояние до УЗ-датчика менее 10см             
    {
      while(!rfid.PICC_IsNewCardPresent() && distance()<10) //Пока карта не приложена и расстояние менее 10см
      {
        for(int i=0;i<8;i++)                                //Выводим стрелку на матрицу
        {
          mtrx.setRow(i,pgm_read_byte(&strela[i]));         //Считываем байт массива из программной памяти и выводим 
          delay(50);
        }
        delay(1000);
        mtrx.clear();                                       //Очищаем матрицу
      }
    }    
  }
 
  int distance()                    //Функция измерения расстояния УЗ-датчика (HC-SR04)
  {
    digitalWrite(TrigPin,LOW);      //Генерируем короткий импульс длительностью 5 микросекунд.
    delayMicroseconds(5);           
    digitalWrite(TrigPin,HIGH);     //Выставив высокий уровень сигнала, ждем 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
    delayMicroseconds(10);          
    digitalWrite(TrigPin,LOW);      
    duration=pulseIn(EchoPin,HIGH); //Время задержки акустического сигнала на эхолокаторе
    return (duration/58.2);         //преобразуем время в см и возваращаем значение
  }

  void readMark()
  {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) //Если карта приложена и считана
    {
      for(byte i=0;i<4;i++)                                         //Извлекаем ID метки (4 байта)
      {
        ID<<=8;                                                     //Сдвигаем в переменной один байт влево            
        ID|=rfid.uid.uidByte[i];                                    //Записываем туда байт из метки
      }
    }
  }
  
  void indication(int time)
  {
    expander.digitalWrite(ZummerPin,LOW);  //Включаем зуммер
    expander.digitalWrite(LedRed,HIGH);    //Зажигаем светодиод
    delay(time);
    expander.digitalWrite(ZummerPin,HIGH); //Выключаем зуммер
    expander.digitalWrite(LedRed,LOW);     //Тушим светодиод
    delay(time);
  }  
  
  void sound()                                                                //Прерывание, если датчик звука уловил сигнал
  { 
    if (millis() - debounce >= 500)                                           //Задержка для стабилизации сигнала
    {
      debounce = millis();                                                    //Запоминаем время
      svet=!svet;                                                             //Инвертируем переменную состояния светодиода
      digitalWrite(LigthPin,svet);                                            //Меняем состояние светодиода
    }
  }
