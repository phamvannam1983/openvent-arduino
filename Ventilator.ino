/*
 * Connect
 *      Arduino       I2C_LCD
 *        5V            VCC
 *        GND           GND
 *        A4            SDA
 *        A5            SCL
 */
#include <avr/interrupt.h>
#include <Wire.h> 
#include "LiquidCrystal_I2C.h"
#include "Adafruit_BMP085.h"
#include <math.h> 
LiquidCrystal_I2C lcd(0x27,16,2);

// Button Pin
#define   TRUE          1
#define   FAIL          0
#define   ON            LOW  
#define   OF            HIGH
#define   L_ON          LOW 
#define   L_OF          HIGH 
#define   EN_ON         HIGH  
#define   EN_OF         LOW
#define   DIR_ON        LOW  
#define   DIR_OF        HIGH
#define   B_ON          HIGH  
#define   B_OF          LOW

#define   B_RED     8
#define   L_RED     9
#define   B_GREEN   10
#define   L_GREEN   11
#define   IN        12
#define   BUZ       13
#define   PWM       2
#define   DIR       3
#define   EN        4

unsigned char   g_Led = OF, g_time_led = 0;
unsigned char   g_start = OF, g_run_motor = OF;
unsigned long   g_Vt,    g_Ti, g_ti,   g_Te, g_te,   g_F;
unsigned long   g_Vt_p;
#define   MODE      4
#define   CMV       0
#define   CPAP      1
#define   VAC       2
#define   TEST      3
unsigned char   mode = 0;
unsigned long   g_active_VAC = 0, g_active_VAC_bk;

unsigned char g_but_green_1 = OF, g_but_green_2 = OF;
unsigned char g_but_red_1 = OF,   g_but_red_2 = OF;


#define NUM   5
long g_arr[NUM], g_threshold_P_H2O;
Adafruit_BMP085 bmp;
/////////////////////////////////////////////////////////////////////////////////////////
// Function /////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void delay_u_s(unsigned long us)
{
    us = (us*4)/5;
    while(us--) {
      delayMicroseconds(2);
    }
}
////////////////////////////////////////////////////
void go_home(void)
{
  unsigned char sum_sensor, i_h = 7;

    while(i_h--){
        if(digitalRead(IN) == ON) {sum_sensor++;}
        else                      {sum_sensor = 0;}
        delay(1);
    }    
    if(sum_sensor >= 5){
        digitalWrite(EN, EN_OF);
        return;
    }
    digitalWrite(DIR, DIR_OF); 
    digitalWrite(EN, EN_ON);
    while(1){
        if(digitalRead(IN) == ON) {sum_sensor++;}
        else                      {sum_sensor = 0;}
        if(sum_sensor >= 5){
            //digitalWrite(EN, EN_OF);
            break;
        }
        delay_u_s(400);
        digitalWrite(PWM, ON);
        delay_u_s(400);
        digitalWrite(PWM, OF);
    }    
}
////////////////////////////////////////////////////
long read_update_P_sensor(unsigned char j)
{
  long P, P_H2O;
    while(j--){
        P = 0;
        for(int i=0; i<(NUM-1); i++){
            g_arr[i] = g_arr[i+1];
            P = P + g_arr[i];
        }
        g_arr[NUM-1] = bmp.readPressure();
        P = P + g_arr[NUM-1];
        P = P/5;
        P_H2O = P/98;       //98 H2O
        Serial.print("Pressure = ");
        Serial.print(P);      Serial.print(" Pa     ");
        Serial.print(P_H2O);  Serial.println(" mm H2O");
    }    
    return P_H2O;
}
////////////////////////////////////////////////////
void display_lcd(unsigned short a, unsigned short b, unsigned short c)
{
  unsigned short ng, tr, ch, dv, temp;
    lcd.setCursor(0,0);   lcd.print("Vt");
    lcd.setCursor(4,0);   lcd.print("Ti");
    lcd.setCursor(9,0);   lcd.print("F");
    lcd.setCursor(12,0);  lcd.print("Mode");
    if      (mode == CMV) {lcd.setCursor(12,1); lcd.print("CMV ");}
    else if (mode == CPAP){lcd.setCursor(12,1); lcd.print("CPAP");}
    else if (mode == VAC) {lcd.setCursor(12,1); lcd.print("VAC ");}
    else if (mode == TEST) {lcd.setCursor(12,1); lcd.print("TEST");}

    ng = a/1000; 
    temp = a%1000;
    tr = temp/100;                       
    temp = a%100; ch = temp/10;       
    dv = a%10;
    if (mode == CPAP) { lcd.setCursor(0,1);   lcd.print("   ");}
    else {              lcd.setCursor(0,1);   /*lcd.write(ng+0x30);*/ lcd.write(tr+0x30); lcd.write(ch+0x30); lcd.write(dv+0x30);}   
    ng = b/1000; 
    temp = b%1000;
    tr = temp/100;                       
    temp = b%100; ch = temp/10;       
    dv = b%10;
    if (mode == CPAP) { lcd.setCursor(4,1);   lcd.print("    ");}
    else {              lcd.setCursor(4,1);   lcd.write(ng+0x30); lcd.write(tr+0x30); lcd.write(ch+0x30); lcd.write(dv+0x30);}  
    tr = c/100;                       
    temp = c%100; ch = temp/10;       
    dv = c%10;
    if (mode == CPAP) { lcd.setCursor(9,1);   lcd.print("   ");}
    else {              lcd.setCursor(9,1);  /*lcd.write(tr+0x30);*/ lcd.write(ch+0x30); lcd.write(dv+0x30);}  
}
////////////////////////////////////////////////////
unsigned long calculate_pulse(unsigned long Vt)
{
    float Vt_p = Vt/800.0;
    Vt_p = sqrt(Vt_p)*450;
    
    return (unsigned long)(Vt_p) + 150;  //offset 150*2 pulse
}
////////////////////////////////////////////////////
void run_CMV(unsigned long Vt_p, unsigned long ti, unsigned long te)
{  
        g_run_motor = ON;
        digitalWrite(EN, EN_ON);
    //ON
        unsigned long pulse =Vt_p, pulse_200 = Vt_p/3, pulse_400 = (Vt_p*2)/3, t_pulse, t1 = ti/2, t2 = (ti*3)/2;
        digitalWrite(DIR, DIR_ON); 
        digitalWrite(L_GREEN, L_ON);
        while(pulse--){
            if      (pulse < pulse_200)   t_pulse = t2;   //<200  slow
            else if (pulse < pulse_400)   t_pulse = ti;   //<400  medium
            else                          t_pulse = t1;   //<600  fast
            delay_u_s(t_pulse); 
            digitalWrite(PWM, ON);
            delay_u_s(t_pulse); 
            digitalWrite(PWM, OF);
        }
        digitalWrite(L_GREEN, L_OF);
        
    //OFF
        digitalWrite(DIR, DIR_OF);
        unsigned char sum_sensor = 0;
        while(1){
            if(digitalRead(IN) == ON) {sum_sensor++;}
            else                      {sum_sensor = 0;}
            if(sum_sensor >= 5){
                Serial.print("a\n");
                if(g_start == OF) {digitalWrite(EN, EN_OF);  Serial.print("b\n");} //???
                break;
            }
            delay_u_s(te);
            digitalWrite(PWM, ON);
            delay_u_s(te);
            digitalWrite(PWM, OF);
        }   
        g_run_motor = OF; 
}
////////////////////////////////////////////////////
void run_CPAP(void)
{
        g_run_motor = ON;
        digitalWrite(EN, EN_ON);
    //ON
        unsigned long pulse = 600, t_pulse = 700;
        digitalWrite(DIR, DIR_ON);
        digitalWrite(L_GREEN, L_ON);
        while(pulse--){
            t_pulse = t_pulse + 12;
            delay_u_s(t_pulse); 
            digitalWrite(PWM, ON);
            delay_u_s(t_pulse); 
            digitalWrite(PWM, OF);
        }
        digitalWrite(L_GREEN, L_OF);
        
     //OFF
        go_home();
        Serial.print("a\n");
        if(g_start == OF) {digitalWrite(EN, EN_OF);  Serial.print("b\n");} //???
        g_run_motor = OF;
}
////////////////////////////////////////////////////
void run_VAC(unsigned long Vt_p, unsigned long ti)
{
  long Pressure_H2O = read_update_P_sensor(1) - g_threshold_P_H2O;    
  
    if( (Pressure_H2O > 0) && (g_active_VAC > 0) ) return;
    g_active_VAC = g_active_VAC_bk;

        g_run_motor = ON;
        digitalWrite(EN, EN_ON);
    //ON
        unsigned long pulse =Vt_p, pulse_200 = Vt_p/3, pulse_400 = (Vt_p*2)/3, t_pulse, t1 = ti/2, t2 = (ti*3)/2;
        digitalWrite(DIR, DIR_ON); 
        digitalWrite(L_GREEN, L_ON);
        while(pulse--){
            if      (pulse < pulse_200)   t_pulse = t2;   //<200  slow
            else if (pulse < pulse_400)   t_pulse = ti;   //<400  medium
            else                          t_pulse = t1;   //<600  fast
            delay_u_s(t_pulse); 
            digitalWrite(PWM, ON);
            delay_u_s(t_pulse); 
            digitalWrite(PWM, OF);
        }
        digitalWrite(L_GREEN, L_OF);

    read_update_P_sensor(NUM);

    //OFF
        go_home();     
        Serial.print("a\n");
        if(g_start == OF) {digitalWrite(EN, EN_OF);  Serial.print("b\n");} //???   
        g_run_motor = OF;
}
////////////////////////////////////////////////////
void run_TEST(unsigned long Vt_p, unsigned long ti)
{
  long Pressure_H2O = read_update_P_sensor(1) - g_threshold_P_H2O;    
  
    if(Pressure_H2O > 0) return;

        g_run_motor = ON;
        digitalWrite(EN, EN_ON);
    //ON
        unsigned long pulse =Vt_p, pulse_200 = Vt_p/3, pulse_400 = (Vt_p*2)/3, t_pulse, t1 = ti/2, t2 = (ti*3)/2;
        digitalWrite(DIR, DIR_ON); 
        digitalWrite(L_GREEN, L_ON);
        while(pulse--){
            if      (pulse < pulse_200)   t_pulse = t2;   //<200  slow
            else if (pulse < pulse_400)   t_pulse = ti;   //<400  medium
            else                          t_pulse = t1;   //<600  fast
            delay_u_s(t_pulse); 
            digitalWrite(PWM, ON);
            delay_u_s(t_pulse); 
            digitalWrite(PWM, OF);
        }
        digitalWrite(L_GREEN, L_OF);

    read_update_P_sensor(NUM);

    //OFF
        go_home();     
        //Serial.print("a\n");
        if(g_start == OF) {digitalWrite(EN, EN_OF);  Serial.print("b\n");} //???   
        g_run_motor = OF;     
}
/////////////////////////////////////////////////////////////////////////////////////////
// Timer int ////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
ISR (TIMER1_OVF_vect) 
{
    TCNT1 = 45535;            //10ms

    g_time_led++;
    if(g_time_led >= 25) {
        g_time_led = 0;
        g_Led = !g_Led;
        if(g_start == ON) digitalWrite(L_RED, g_Led);
    }

    g_but_red_1 = g_but_red_2;
    g_but_red_2 = digitalRead(B_RED);
    if((g_but_red_1 == ON) && (g_but_red_2 == OF) ) {
        mode++;
        if(mode == MODE) mode = 0;
    }
    
    g_but_green_1 = g_but_green_2;
    g_but_green_2 = digitalRead(B_GREEN);
    if((g_but_green_1 == ON) && (g_but_green_2 == OF) ) {
        if(g_start == OF) {g_start = ON;}
        else {
            g_start = OF; 
            digitalWrite(L_RED, L_OF);
            if(g_run_motor == OF){digitalWrite(EN, EN_OF);}
        }
    }

    if(g_active_VAC > 0) g_active_VAC--;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup & Main /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    lcd.init();                      // initialize the lcd 
    pinMode(DIR, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(L_GREEN, OUTPUT);
    pinMode(B_GREEN, INPUT_PULLUP);
    pinMode(L_RED, OUTPUT);
    pinMode(B_RED, INPUT_PULLUP);
    pinMode(IN, INPUT_PULLUP);
    pinMode(BUZ, OUTPUT);

    digitalWrite(BUZ, B_OF);
    digitalWrite(L_GREEN, L_OF);
    digitalWrite(L_RED, L_OF);
    digitalWrite(EN, EN_OF);

    Serial.begin(115200);
    Serial.println("Go Home");
    go_home(); digitalWrite(EN, EN_OF);
    
    // Print a message to the LCD.
    lcd.backlight();
    display_lcd(0, 0, 0);

    //init BMP180
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
    memset(g_arr, 0, NUM);
    g_threshold_P_H2O = read_update_P_sensor(NUM);
    Serial.print("Threshold Pressure H2O = ");
    Serial.print(g_threshold_P_H2O);      Serial.println(" H2O");

    /* Reset Timer/Counter1 */
    TCCR1A = 0; TCCR1B = 0; TIMSK1 = 0;
    /* Setup Timer/Counter1 */
    TCCR1B |= (1 << CS11);    // prescale = 8
    TCNT1 = 45535;            //10ms
    TIMSK1 = (1 << TOIE1);    // Overflow interrupt enable 
    sei();                    // cho phép ngắt toàn cục
}
/////////////////////////////////////////////////////////////////////////
void loop()
{
    //////////Read volume & display LCD for Vt, ti, F
    g_Vt = analogRead(0);     //200 -> 800
        g_Vt = (g_Vt*6)/10    + 200;         
        if(g_Vt > 800)  g_Vt  = 800; 
    g_Ti = analogRead(1);     //500 -> 2000
        g_Ti = (g_Ti*3)/2     + 500;     
        if(g_Ti > 2000)  g_Ti = 2000;
    g_F = analogRead(2);      //10 -> 40
        g_F = g_F/34          + 10; 
        if(g_F > 40)     g_F  = 40;

        if((g_Ti >= 1000) && (g_F > 35) ) g_F  = 35;
        if((g_Ti >= 1200) && (g_F > 30) ) g_F  = 30;
        if((g_Ti >= 1400) && (g_F > 25) ) g_F  = 25;
        if((g_Ti >= 1600) && (g_F > 22) ) g_F  = 22;
        if((g_Ti >= 1800) && (g_F > 20) ) g_F  = 20;
    display_lcd(g_Vt, g_Ti, g_F);
    
    //////////Calculate pulse & timer for Vt_p, ti, te
        g_Vt_p = calculate_pulse(g_Vt);
        //Serial.print("\n"); Serial.print(g_Vt_p);
        
        g_ti = (g_Ti*1000)/g_Vt_p;
        g_ti = g_ti/2;                            //2T

        g_te = 60000000/g_F;
        g_te = g_te - g_ti*2*g_Vt_p;
        g_te = (g_te/g_Vt_p)/2;                   //2T

        g_active_VAC_bk = 6000/g_F;
        
    //////////Control Step motor
    if(g_start == ON) {
        if      (mode == 0) run_CMV(g_Vt_p, g_ti, g_te);
        else if (mode == 1) run_CPAP();
        else if (mode == 2) run_VAC(g_Vt_p, g_ti);
        else if (mode == 3) run_TEST(g_Vt_p, g_ti);
    }
   
}
