#include "Adafruit_MCP4725.h"
#include "EEPROM.h"
#include "LiquidCrystal_I2C.h"
#include "OneButton.h"
#include "OneWire.h" //DS18B20
#include "Wire.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

OneButton button1(8, true); // увеличиваем напряжение, ток
OneButton button2(6, true); // уменьшаем напряжение, ток
OneButton button3(7, true); // выбор значения, меню
OneButton button4(9, true); // отключаем выход БП

Adafruit_MCP4725 Udac;
Adafruit_MCP4725 Idac;

OneWire ds(2); // Объект OneWire

const char NAME[] = "LBP";
const char DEVICE[] = "by IVAN-KLUCH";
const char VERSION[] = "1.1 DAC";

enum {
    RelayOut = 4, // реле подаем напряжение на выход
    Relay1 = 10, // переключение обмоток 1-е реле
    Relay2 = 11, // переключение обмоток 2-е реле
    Relay3 = 12, // переключение обмоток 3-е реле
    fan = 5, // управление вентилятором
    buzzer = 3, // сигнал спикера
    RelayOff = A3, // сигнал автоотключения
};

struct fix_t {
    float U = 0.0; // переменная для цифр фиксированого напряжения 1-й блок
    uint16_t dacU = 0; // переменая значения ЦАП для напряжения 1-й блок
    float I = 0.0; // переменная для фиксированого тока 1-й блок
    uint16_t dacI = 0; // переменая значения ЦАП для тока 1-й блок
} fix[5]; // 5 блоков (определение струкруры fix_t и объявление переменной fix)

struct addr_t {
    addr_t()
        : addr {
            sizeof(fix_t) * 0, // Начальный адресс ячейки памяти для 1-го блока
            sizeof(fix_t) * 1, // Начальный адресс ячейки памяти для 2-го блока
            sizeof(fix_t) * 2, // Начальный адресс ячейки памяти для 3-го блока
            sizeof(fix_t) * 3, // Начальный адресс ячейки памяти для 4-го блока
            sizeof(fix_t) * 4, // Начальный адресс ячейки памяти для 5-го блока};
            sizeof(fix_t) * 5 + sizeof(int) * 0, // Начальный адресс ячейки памяти для напряжения
            sizeof(fix_t) * 5 + sizeof(int) * 1, // Начальный адресс ячейки памяти для тока
        }
    {
    }

    enum : int {
        A, // Начальный адресс ячейки памяти для 1-го блока
        B, // Начальный адресс ячейки памяти для 2-го блока
        C, // Начальный адресс ячейки памяти для 3-го блока
        D, // Начальный адресс ячейки памяти для 4-го блока
        E, // Начальный адресс ячейки памяти для 5-го блока

        U, // Начальный адресс ячейки памяти для напряжения
        I, // Начальный адресс ячейки памяти для тока
    };

    const uint16_t addr[I + 1];

    inline int operator[](uint8_t a) { return addr[a]; }
} Address;

struct {
    float U = 0; // переменная для вывода на дисплей напряжения (при выборе фиксированных значений)
    float I = 0; // переменная для вывода на дисплей тока (при выборе фиксированных значений)
} Preset;

struct dac_t {
    enum : int {
        MaxU = 4052, // максимальное значение
        MinU = 10, // минимальное значение
        MaxI = 4070, // максимальное значение (ток)
        MinI = 10, // минимальное значение  (ток)
    };
    uint16_t voltage = 0; // установка напряжения, отсчет в диапазоне 0-4095
    uint16_t current = 0; //установка тока, отсчет в диапазоне 0-4095
} Dac;

struct Freq {
    enum : int {
        sw = 2000, // часота тона при нажатии кнопок
        alarm = 1500, // частота тона, сигнал тревоги  sw = 20, // длительность тона при нажатии кнопок
    };
};

struct Time {
    enum : int {
        sw = 20, // длительность тона при нажатии кнопок
        alarm = 500, // длительность тона, сигнал тревоги
    };
};

struct timer_t {
    enum : int { off_set = 30 };
    uint8_t off;
} Timer;

const uint8_t gradus[8] {
    // кодируем символ градуса
    B01110,
    B01010,
    B01110,
    B00000,
    B00000,
    B00000,
    B00000,
};

int temperature = 0; // переменная для хранения значение температуры с датчика DS18B20
long lastUpdateTime = 0; // Переменная для хранения времени последнего считывания с датчика

const int TEMP_UPDATE_TIME = 1000; // Определяем периодичность проверок
const int VOLTAGE = A0;
const int CURRENT = A6;

const float CoefI = 0.838; // коэффициент делителя тока CoefI=Iout/5v опорное
const float CoefU = 7.921; // коэффициент делителя напряжения CoefU=Uout(max)/5v.опорное
const float Ref_vol = 5.05; // опорное напряжение
const float U_relay_off = 19.5; // напряжение отключения реле (переключение обмоток)
const float U_relay_on = 20.0; // напряжение включения реле (переключение обмоток)

float Iout = 0;
float Ucalc = 0;
float Ucorr = 0;
float Uout = 0;

//float set_Iout = 0;

float CurrentSet = 0; // переменная установленного тока

int set = 0; // пункты меню
int menu = 0; // меню
uint8_t preset_select = addr_t::A; // блоки фикированных напряжений
int set_fix_val = 0; // настройка фиксированных значений
int output_control = 0; // управление выходом БП
unsigned long store_exit_time; // храним врема выхода из меню
const int exit_time = 4000; // устанавливаем время выхода из меню

//byte var = 0;

unsigned long store_countdown_timer; // храним время автоотключения
const int countdown_timer = 10; // устанавливаем время автоотключения в минутах
unsigned long store_shutdown_time; // храним время обратного отсчета
const int shutdown_time = 1000; // время обратного отсчета 1 сек.

int detectTemperature();
void avto_off();
void cancel_auto_off();
void click1();
void click2();
void click3();
void click4();
void doubleclick3();
void functions_for_fixed_values();
void longPress1();
void longPress2();
void longPress3();
void longPress4();
void record_fixed_values();
void select_fixed_value();
void signal_attention();

//-------------------------------------------------------------------
void setup()
{

    lcd.init();
    lcd.backlight(); // Включаем подсветку дисплея
    lcd.createChar(1, const_cast<uint8_t*>(gradus)); // Создаем символ под номером 1
    lcd.setCursor(6, 0);
    lcd.print(NAME);
    lcd.setCursor(1, 1);
    lcd.print(DEVICE);
    delay(1500);
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Version ");
    lcd.setCursor(4, 1);
    lcd.print(VERSION);
    delay(1500);
    lcd.clear();
    Serial.begin(115200);
    analogReference(EXTERNAL); // опорное напряжение
    // Чтение данных из EEPROM
    EEPROM.get(Address[addr_t::U], Dac.voltage);
    EEPROM.get(Address[addr_t::I], Dac.current);

    pinMode(RelayOut, OUTPUT);
    pinMode(Relay1, OUTPUT);
    pinMode(Relay2, OUTPUT);
    pinMode(Relay3, OUTPUT);
    pinMode(fan, OUTPUT);
    pinMode(RelayOff, OUTPUT);
    Udac.begin(0x60);
    Idac.begin(0x61);
    Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    Idac.setVoltage(Dac.current, false); // даем команду цап, устанавливаем ток

    button1.attachClick(click1);
    button1.attachDuringLongPress(longPress1);

    button2.attachClick(click2);
    button2.attachDuringLongPress(longPress2);

    button3.attachClick(click3);
    button3.attachDoubleClick(doubleclick3);
    button3.attachDuringLongPress(longPress3);

    button4.attachClick(click4);
    button4.attachDuringLongPress(longPress4);

    Timer.off = timer_t::off_set; // обратный отсчет устанавливаем в начало
    digitalWrite(RelayOff, 1);
} //конец setup

void loop()
{
    Serial.print("HalaBuda\n");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(1000); // wait for a second
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    delay(1000); // wait for a second

    if (millis() - store_countdown_timer > countdown_timer * 60000 && Iout == 0) { // условия автоотключения
        avto_off();
    }
    if (Iout > 0) { // если ток более нуля сбрасываем таймер автооткл.
        cancel_auto_off();
    }

    // функция опроса кнопок
    button1.tick();
    button2.tick();
    button3.tick();
    button4.tick();

    detectTemperature(); // Определяем температуру от датчика DS18b20

    //------------------- запись в память напряжения ------------------------
    if (millis() - store_exit_time > exit_time && set == 1 && set != 4) {
        set = 0;
        lcd.clear();
        EEPROM.put(Address[addr_t::U], Dac.voltage); // Запись напряжения в память
    }
    //------------------- запись в помять напряжения и тока -----------------
    if (millis() - store_exit_time > exit_time && set == 2 && set != 4) {
        set = 0;
        lcd.clear();
        EEPROM.put(Address[addr_t::U], Dac.voltage); // Запись напряжения в память
        EEPROM.put(Address[addr_t::I], Dac.current); // Запись тока в память
    }
    //--------------------считаем напряжение и ток--------------------------
    Ucalc = analogRead(VOLTAGE) * (Ref_vol / 1023.0) * CoefU; //узнаем напряжение на выходе
    Ucorr = (40.0 - Ucalc) * 0.0026; //коррекция напряжения, при желании можно подстроить
    Uout = Ucalc + Ucorr;
    Iout = analogRead(CURRENT) * (Ref_vol / 1023.0) * CoefI; // узнаем ток в нагрузке

    CurrentSet = Ref_vol / dac_t::MaxI * CoefI * Dac.current; // в этой строке считаем какой ток установлен на выходе

    //--------------действия при высокой температуре радиатора---------
    if (temperature >= 35)
        digitalWrite(fan, 1);
    if (temperature <= 31)
        digitalWrite(fan, 0);
    if (temperature >= 38) {
        digitalWrite(RelayOut, 1);
        output_control = 1;
    }

    //---------------переключение обмотк трансфотматора----------------
    if (Uout >= U_relay_on)
        digitalWrite(Relay1, 1);
    if (Uout <= U_relay_off)
        digitalWrite(Relay1, 0);

    if (Dac.voltage >= dac_t::MaxU)
        Dac.voltage = dac_t::MaxU; //не выходим за приделы максимума
    if (Dac.voltage <= dac_t::MinU)
        Dac.voltage = dac_t::MinU; //не выходим за приделы минимума

    if (Dac.current >= dac_t::MaxI)
        Dac.current = dac_t::MaxI; //не выходим за приделы максимума
    if (Dac.current <= dac_t::MinI)
        Dac.current = dac_t::MinI; //не выходим за приделы минимума
    //----------------выводим информацию на дисплей--------------------
    if (menu == 0) {
        lcd.setCursor(0, 0);
        if (Uout < 10)
            lcd.print(" ");
        lcd.print(Uout, 2);
        lcd.print(" V");

        lcd.setCursor(9, 0);
        lcd.print(Iout, 2);
        lcd.print(" A");
    }
    if (menu == 1 && set_fix_val == 1 || set_fix_val == 2) {
        lcd.setCursor(0, 0);
        if (Uout < 10)
            lcd.print(" ");
        lcd.print(Uout, 2);
        lcd.print(" V");
        lcd.setCursor(9, 0);
        lcd.print(CurrentSet, 2);
        lcd.print(" A ");
    }
    if (menu == 1 && set_fix_val == 1) {
        lcd.setCursor(0, 1);
        lcd.print("voltage record");
    }
    if (menu == 1 && set_fix_val == 2) {
        lcd.setCursor(0, 1);
        lcd.print("current record");
    }
    if (menu == 1) {
        lcd.setCursor(15, 1);
        lcd.print(preset_select + 1);
    }
    if (set == 0 && Timer.off == timer_t::off_set && menu == 0) {
        lcd.setCursor(9, 1);
        lcd.print("t ");
        lcd.print(temperature);
        lcd.print("\1C ");
    }
    if (set == 0 && output_control == 0 && menu == 0) {
        lcd.setCursor(0, 1);
        if (Uout * Iout < 10)
            lcd.print(" ");
        lcd.print(Uout * Iout, 2);
        lcd.print(" W ");
    }
    if (menu == 0 && set == 0 && output_control == 1) {
        lcd.setCursor(0, 1);
        lcd.print("Out OFF");
    }
    if (set == 1 && menu == 0) {
        lcd.setCursor(0, 1);
        lcd.print("VOLTAGE");
    }
    if (set == 2 && menu == 0) {
        lcd.setCursor(0, 1);
        lcd.print("CURRENT");
        lcd.setCursor(9, 1);
        lcd.print(CurrentSet, 2);
        lcd.print(" A ");
    }
    if (menu == 1 && set_fix_val == 0) {
        functions_for_fixed_values();
    }
    if (menu == 1 && set_fix_val == 3) {
        for (int i = 0; i <= 9; i++) {
            Serial.println(i);
            tone(buzzer, Freq::alarm, Time::alarm);
            record_fixed_values();
        }
        set_fix_val = 0;
    }
    if (menu == 1 && set_fix_val == 4) {
        select_fixed_value();
        set_fix_val = 0;
        menu = 0;
        lcd.clear();
    }
    if (menu == 1) {
        digitalWrite(RelayOut, 1);
        output_control = 1;
    }
} //конец loop
//-------------функции для кнопки плюс (один клик)--------------------------
void click1()
{
    if (menu == 0 && set == 0 || set == 1) {
        set = 1;
        if (Dac.voltage < dac_t::MaxU)
            Dac.voltage = Dac.voltage + 2; //добавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (set == 2 && menu == 0) {
        if (Dac.current < dac_t::MaxU)
            Dac.current = Dac.current + 10;
        Idac.setVoltage(Dac.current, false);
    }
    if (menu == 1 && set_fix_val == 0) { // переключаем блоки фиксированных значений +
        ++preset_select;
        if (preset_select == addr_t::E)
            preset_select = 0;
    }
    if (menu == 1 && set_fix_val == 1) {
        if (Dac.voltage < dac_t::MaxU)
            Dac.voltage = Dac.voltage + 2; //добавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 1 && set_fix_val == 2) {
        if (Dac.current < dac_t::MaxU)
            Dac.current = Dac.current + 10;
        Idac.setVoltage(Dac.current, false);
    }
    tone(buzzer, Freq::sw, Time::sw); // сигнал спикера
    store_exit_time = millis();
    cancel_auto_off();
} //конец click1()
//---------------функции для кнопки плюс (непрерывно нажата)----------------
void longPress1()
{
    if (menu == 0 && set == 0 || set == 1) {
        set = 1;
        if (Dac.voltage > 200) {
            Dac.voltage = Dac.voltage + 10; //убавляем
        }
        if (Dac.voltage <= 200) {
            Dac.voltage = Dac.voltage + 4; //убавляем
        }
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 0 && set == 2) {
        if (Dac.current < dac_t::MaxU)
            Dac.current = Dac.current + 30;
        Idac.setVoltage(Dac.current, false);
    }
    if (menu == 1 && set_fix_val == 1 && Dac.voltage > 200) {
        Dac.voltage = Dac.voltage + 10; //добавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 1 && set_fix_val == 1 && Dac.voltage <= 200) {
        Dac.voltage = Dac.voltage + 4; //добавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 1 && set_fix_val == 2) {
        if (Dac.current < dac_t::MaxU)
            Dac.current = Dac.current + 30;
        Idac.setVoltage(Dac.current, false);
    }
    store_exit_time = millis();
    cancel_auto_off();
} //конец longPress1
//---------------функции для кнопки минус (один клик)--------------------
void click2()
{
    tone(buzzer, Freq::sw, Time::sw);
    if (menu == 0 && set == 0 || set == 1) {
        set = 1;
        if (Dac.voltage > dac_t::MinU)
            Dac.voltage = Dac.voltage - 2; //убавляем
        store_exit_time = millis();
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (set == 2 && menu == 0) {
        if (Dac.current > dac_t::MinU)
            Dac.current = Dac.current - 10; //убавляем
        Idac.setVoltage(Dac.current, false);
    }
    if (menu == 1 && set_fix_val == 0) { // переключаем блоки фиксированных значений -
        --preset_select;
        if (preset_select > addr_t::E)
            preset_select = addr_t::E;
    }

    if (menu == 1 && set_fix_val == 1) {
        if (Dac.voltage > dac_t::MinU)
            Dac.voltage = Dac.voltage - 2; //убавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
        //fix[Fix::A].dacU = Dac.VOLTAGE;
    }
    if (menu == 1 && set_fix_val == 2) {
        if (Dac.current > dac_t::MinU)
            Dac.current = Dac.current - 10; //убавляем
        Idac.setVoltage(Dac.current, false);
    }
    store_exit_time = millis();
    cancel_auto_off();
} //конец click2
//----------------функции для кнопки минус (непрерывно нажата)-------------
void longPress2()
{
    if (menu == 0 && set == 0 || set == 1) {
        set = 1;
        if (Dac.voltage > 200) {
            Dac.voltage = Dac.voltage - 10; //убавляем
        }
        if (Dac.voltage <= 200) {
            Dac.voltage = Dac.voltage - 4; //убавляем
        }
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 0 && set == 2) {
        if (Dac.current > dac_t::MinU)
            Dac.current = Dac.current - 30; //убавляем
        Idac.setVoltage(Dac.current, false);
    }
    if (menu == 1 && set_fix_val == 1 && Dac.voltage > 200) {
        Dac.voltage = Dac.voltage - 10; //убавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 1 && set_fix_val == 1 && Dac.voltage <= 200) {
        Dac.voltage = Dac.voltage - 4; //убавляем
        Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    }
    if (menu == 1 && set_fix_val == 2) {
        if (Dac.current > dac_t::MinU)
            Dac.current = Dac.current - 30; //убавляем
        Idac.setVoltage(Dac.current, false);
    }
    store_exit_time = millis();
    cancel_auto_off();
} // конец longPress2
//-------------------(действия для кнопки меню (один клик)---------------
void click3()
{
    tone(buzzer, Freq::sw, Time::sw); // сигнал спикера
    if (menu == 0) {
        set = set + 1; //поочередно переключаем режим отображения информации
        if (set == 3)
            set = 0; //дошли до конца, начинаем снова
    }
    if (menu == 1 && set_fix_val == 0) {
        set_fix_val = 4;
    }
    store_exit_time = millis();
    cancel_auto_off();
    if (set_fix_val == 1 || set_fix_val == 2) {
        set_fix_val++;
    }
} //конец click3
//-------------------(действия для кнопки меню (двойной клик)-------------
void doubleclick3()
{
    menu = menu + 1;
    if (menu == 2)
        menu = 0; //дошли до конца, начинаем снова
    store_exit_time = millis();
    cancel_auto_off();
    tone(buzzer, Freq::sw, Time::sw); // сигнал спикера
    lcd.clear();
} // конец doubleclick3
//------------------(действия для кнопки меню (длительно нажата)-----------
void longPress3()
{
    if (menu == 1) {
        set_fix_val = 1;
    }
} // конец longPress3
//------------------(действия для кнопки отключение выхода (один клик)-------
void click4()
{
    tone(buzzer, Freq::sw, Time::sw); // сигнал спикера
    output_control = output_control + 1;
    if (output_control == 2)
        output_control = 0;

    if (output_control == 0) {
        digitalWrite(RelayOut, 0);
    }

    if (output_control == 1) {
        digitalWrite(RelayOut, 1);
    }
    cancel_auto_off();
} // конец click4()
//----------------(действия для кнопки отключение выхода (длительно нажата)------
void longPress4()
{
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Power OFF");
    delay(500);
    signal_attention();
    delay(1000);
    digitalWrite(RelayOff, 0);
} // конец longPress4
//---------------сигнал спикера-------------
void signal_attention()
{
    tone(buzzer, Freq::alarm, Time::alarm);
}
//------------------функции с фиксированными значениями --------------------------
void functions_for_fixed_values()
{

    lcd.setCursor(0, 1);
    lcd.print("Preset select");
    lcd.setCursor(0, 0);
    if (Preset.U < 10)
        lcd.print(" ");
    lcd.print(Preset.U);
    lcd.print(" V ");
    lcd.setCursor(9, 0);
    lcd.print(Preset.I);
    lcd.print(" A");

    EEPROM.get(Address[preset_select], fix[preset_select]);
    Preset.U = fix[preset_select].U;
    Preset.I = fix[preset_select].I;

    //    if (preset_select == 1) {
    //        EEPROM.get(Address::A_Uout, fix[Fix::A].U);
    //        EEPROM.get(Address::A_Udac, fix[Fix::A].dacU);
    //        EEPROM.get(Address::A_Iout, fix[Fix::A].I);
    //        EEPROM.get(Address::A_Idac, fix[Fix::A].dacI);
    //        Preset.U = fix[Fix::A].U;
    //        Preset.I = fix[Fix::A].I;
    //    }
} // конец functions_for_fixed_values
//------------------запись в память фиксированных значений-------------------------
void record_fixed_values()
{
    EEPROM.put(Address[preset_select], fix_t { Uout, Dac.voltage, CurrentSet, Dac.current });

    //    if (preset_select == 1) {
    //        EEPROM.put(Address::A_Uout, Uout); // Запись цифр напряжения в память 1-й блок
    //        EEPROM.put(Address::A_Udac, Dac.VOLTAGE); // Запись значения ЦАП напряжения в память 1-й блок
    //        EEPROM.put(Address::A_Iout, CURRENT_SET); // Запись цифр тока в память 1-й блок
    //        EEPROM.put(Address::A_Idac, Dac.CURRENT); // Запись значения ЦАП тока в память 1-й блок
    //    }
} // конец record_fixed_values()
//---------------установка одного из фиксированных значений в активное------------
void select_fixed_value()
{
    //    if (preset_select == 1) {
    //        Dac.VOLTAGE = fix[Fix::A].dacU;
    //        Dac.CURRENT = fix[Fix::A].dacI;
    //    }
    Dac.voltage = fix[preset_select].dacU;
    Dac.current = fix[preset_select].dacI;

    Udac.setVoltage(Dac.voltage, false); // даем команду цап, устанавливаем напряжение
    Idac.setVoltage(Dac.current, false); // даем команду цап, устанавливаем ток
    EEPROM.put(Address[addr_t::U], Dac.voltage); // Запись напряжения в память
    EEPROM.put(Address[addr_t::I], Dac.current); // Запись тока в память
} // конец select_fixed_value

//-------------------отмена автоотключения---------------------------
void cancel_auto_off()
{
    store_countdown_timer = millis(); // сброс таймера автоотключения
    Timer.off = timer_t::off_set; // обратный отсчет устанавливаем в начало
}
//-------------------автоотключение, обратный отсчет-------------------
void avto_off()
{
    if (millis() - store_shutdown_time > shutdown_time) {
        Timer.off--;
        store_shutdown_time = millis();
        lcd.clear();
    }

    lcd.setCursor(0, 1);
    lcd.print("Auto OFF");
    lcd.setCursor(9, 1);
    lcd.print(Timer.off);
    lcd.setCursor(12, 1);
    lcd.print("sec");

    if (Timer.off == 1) {
        tone(buzzer, Freq::alarm, Time::alarm); // сигнал спикера
        delay(1000);
        digitalWrite(RelayOff, 0);
    }
} //конец avto_off
//--------------------мерим температуру-------------------------------
int detectTemperature()
{

    byte data[2];
    ds.reset();
    ds.write(0xCC);
    ds.write(0x44);

    if (millis() - lastUpdateTime > TEMP_UPDATE_TIME) {
        lastUpdateTime = millis();
        ds.reset();
        ds.write(0xCC);
        ds.write(0xBE);
        data[0] = ds.read();
        data[1] = ds.read();

        temperature = (data[1] << 8) + data[0];
        temperature = temperature >> 4;
    }
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////
