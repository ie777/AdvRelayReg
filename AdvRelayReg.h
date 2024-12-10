/********************************************************************************************
 * 
 * Работа с релейным регулятором. 
 * Включает улучшеный алгоритм регулирования для инерционных систем (подсмотрено у GyverRelay)
 *
 *******************************************************************************************/
#pragma once
#include <Arduino.h>

//Типы регистров портов для разных платформ
#if defined(__AVR__)
  #define _GPIO_REG_TYPE   uint8_t
#elif defined(ARDUINO_ARCH_ESP32)
  #define _GPIO_REG_TYPE   uint8_t
#elif defined(ARDUINO_ARCH_ESP8266)
  #define _GPIO_REG_TYPE   uint8_t
#elif defined(ARDUINO_ARCH_STM32)
  #define _GPIO_REG_TYPE   uint32_t
#endif

enum regDirection {
  REG_DIR_NORMAL = 0,  //Включаем нагрузку при переходе через значение снизу (пример: охлаждение)
  REG_DIR_REVERS       //Включаем нагрузку при переходе через значение сверху (пример: нагрев)
};

enum regMode {
  REG_MODE_OFF = 0,  
  REG_MODE_ONCE,
  REG_MODE_CONTINIOUS
};

enum regStatus {
  REG_STATUS_TRANSIENT = 0,   //Процесс идет к целевой точке (напр., нагрев)
  REG_STATUS_STABLE           //Процесс достиг целевой точки (напр., остывание) 
};

enum regErrorStatus {
  REG_ERROR_OK,
  REG_ERROR_OVERTIME
};

class AdvRelayReg 
{
public:
/**
 * Конструктор. Принимает пин реле, 
 * указатель на переменную в которой хранится текущее значение сигнала (периодически обновляется с помощью внешней функциии),
 * гистерезис,
 * режим - нормальный (например, нагрев) либо инверсный (охлаждение)
 * коэффициент и время выборки для улучшеного регулирования инерционных систем (подбирается индивидуально)
 * Таймаут, в течение которого процесс должен достичь целевой точки, для констатации ошибки в случае недостижения ( 0 - не проверять ошибку)
*/
  AdvRelayReg (_GPIO_REG_TYPE relayPin, float *actualVal, float hysteresis, bool dir = REG_DIR_NORMAL, float k = 0, float dtSec = 0, uint16_t timeoutSec = 0)
  {
    _relayPin = relayPin;
    _actualVal = actualVal;
    _dir = dir;
    _hysteresis = hysteresis;
    _k = k;
    _dtSec = dtSec;
    _timeout = timeoutSec;
    pinMode(_relayPin, OUTPUT);
    digitalWrite(_relayPin, LOW);
  }

  /**
   * Работа регулятора, неблокирующая функция для вызова в цикле.
  */
  int tick () {
    //Если заданы коэф. и интервал, то ускорить срабатывание реле путем добавки к сигналу значения, пропорционального скорости изменения сигнала
    //Иначе регулировать без учета инерции  
    float signal = *_actualVal + calcAddition(); 

    errorCheck();

    switch (mode) {
      //Режим выключено
      case REG_MODE_OFF: 
        return 1;  
      
      //Режим единоразовой работы 
      case REG_MODE_ONCE:   
        //Если показания не достигли целевой точки
        if ((_dir == REG_DIR_NORMAL  &&  signal < _setpoint)  ||  (_dir == REG_DIR_REVERS  &&  signal > _setpoint))  {  
          relOn();        //Включать реле
          status = REG_STATUS_TRANSIENT;
          return 0;       //Переходный процесс
        }
        relOff(); 
        mode = REG_MODE_OFF;          //Завершить регулирование 
        status = REG_STATUS_STABLE;   //Статус стабильно
        return 1;                     //Реле отключено, процесс окончен
      
      //Режим постоянной работы   
      case REG_MODE_CONTINIOUS:       
        if (signal < _setpoint - _hysteresis / 2) {         //Показания понизились
          if (_dir == REG_DIR_NORMAL)   relOn();            //Отработать реле в зависимости от режима
          else                          relOff();   
          status = REG_STATUS_TRANSIENT;
          return 0;                                         //Реле сработано, идет процесс регулирования
        }
        else if (signal > _setpoint + _hysteresis / 2) {    //Показания повысились
          if (_dir == REG_DIR_NORMAL)   relOff();            //Отработать реле в зависимости от режима
          else                          relOn();            
          status = REG_STATUS_TRANSIENT; 
          return 0;                                         //Реле сработано, идет процесс регулирования
        }
        status = REG_STATUS_STABLE; 
        return 1;                                           //Реле отключено, цикл регулирования завершен
    }
    //dataToPort(); Вывод данных в порт для графика в Arduino IDE 
    return 1;
  }

  /**
   * Вывод данных в порт для графика в Arduino IDE 
   */
  void dataToPort (void) {
    Serial.print(*_actualVal);
    Serial.print(' ');
    Serial.print(_setpoint - _hysteresis / 2);
    Serial.print(' ');
    Serial.print(_setpoint + _hysteresis / 2);
    Serial.print(' ');
    Serial.print(_setpoint);
    Serial.print(' ');
    Serial.println(!digitalRead(_relayPin) * _hysteresis + _setpoint - _hysteresis / 2);
    delay(10);
  }
 
  /**
   * Вычисление добавки к сигналу для регулирования инерционных сисием
   * Если заданы коэф. и интервал, то ускорить срабатывание реле путем добавки к сигналу значения, пропорционального скорости изменения сигнала
  */
  float calcAddition () {
    if (_dtSec  &&  _k) {       
      if (millis() - tmr * 1000 > _dtSec) {           //Если интервал истек, делаем измерение
        prevVal = *_actualVal;                        //Сохранить предидущее значение
        tmr = millis();        
        regAdd = _k * (*_actualVal - prevVal) / _dtSec;  //Обновить добавку к сигналу, пропорциональную производной сигнала
      }
      return regAdd;
    }
    else return 0;
  }

  /**
   * Проверка на ошибку (регулирование не идет)
  */
  int errorCheck () {
    static int st = 0;
    static uint32_t tmr;

    if (_timeout == 0)  return 0; //Не проверять ошибку

    switch (st) {
      case 0:   //состояние покоя
        if ( getStatus() == REG_STATUS_STABLE )
          return 0;
        else if ( getStatus() == REG_STATUS_TRANSIENT )  { //Если включилорь реле, то меняем состояние   
          tmr = millis(); //Фиксируем всремя старта
          st = 1;
          return 0;
        }

      case 1: //Состояние регулирования
        if (getStatus() == REG_STATUS_STABLE) {  //Если завершился цикл
          st = 0;
          error = REG_ERROR_OK;
          return 0;
        }
        //Если не было завершения за отведенный таймаут
        if (millis() - tmr > _timeout * 1000) {  
          error = REG_ERROR_OVERTIME;
          return 1;
        }
    }
    return 0;
  }

  void setPoint (float setpoint) {
    _setpoint = setpoint;
  }  

  void setKoeff (float k) {
    _k = k;
  }

  void setInterval(float dtSec) {
    _dtSec = dtSec;
  }  

  void relOn() {
    digitalWrite(_relayPin, HIGH);
  }

  void relOff() {
    digitalWrite(_relayPin, LOW);
  }

  /**
   * Запуск работы с указанием целевого значения, при этом функция tick должна постоянно вызываться в цикле
  */
  void start(float setpoint) {
    _setpoint = setpoint;
    mode = REG_MODE_CONTINIOUS;
  }

  /**
   * Запуск без параметров, должны быть заданы ранее
  */  
  void start() {
    start(_setpoint);
  }

  /**
   * Запуск работы с указанием целевого значения, далее необходимо вызывать функцию tick в цикле
  */
  void startOnce(float setpoint) {
    _setpoint = setpoint;
    mode = REG_MODE_ONCE;
  }

  void startOnce() {
    startOnce(_setpoint);
  }

  /**
   * Остановка, отключает реле
  */
  void stop() {
    mode = REG_MODE_OFF;
    relOff();
  }

  /**
   * Работа реле - блокирующая функция, вечный цикл, для использования в многозадачности
  */
  void run (float setpoint) {
    start(setpoint);
    while (mode != REG_MODE_OFF)
      tick();
  } 

  void runOnce (float setpoint) {
    startOnce(setpoint);
    while (mode != REG_MODE_OFF)
      tick();
  } 

  int getStatus () {
    return status;
  }

  int getMode () {
    return mode;
  }

  int isRelayOn () {
    return digitalRead(_relayPin);
  }

  int getError () {
    return error;
  }

  void errorClear () {
    error = 0;
  }

  bool _dir;
  float _setpoint;
  float _hysteresis;
  float _k;
  float _dtSec;
  float *_actualVal;
  uint16_t _timeout;
  int error;

private:
  _GPIO_REG_TYPE _relayPin;
  float  prevVal;
  float regAdd;
  uint32_t tmr;
  int mode = 0;
  int status = 0;
};
