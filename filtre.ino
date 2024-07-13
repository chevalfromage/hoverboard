#include "filter.hpp"
typedef float REAL_TYPE;
const unsigned int sample_frequence = 1000; // In Hz
const unsigned int output_frequence = 100; // In hz
const unsigned int decimation_factor = sample_frequence / output_frequence;
// Butterworth filter :
//    Parameter :
//        order : 5
//        cut frequency : 23.0 Hz,
//        sample frequency : 1.0 kHz
//    Analisys :
//        Maximal expected delay : 25.92 ms
const int order = 5;
RII_filter<REAL_TYPE, order> filterR{
    {1.5722185e-06, 7.861093e-06, 1.5722186e-05, 1.5722186e-05, 7.861093e-06, 1.5722185e-06, }, // b : Numerator
    {1.0, -4.532463, 8.237289, -7.5021424, 3.423521, -0.6261542, } // a : Denominator
};
RII_filter<REAL_TYPE, order> filterL = filterR;

#include "Wire.h"

#define address 0x68

#define SDA_R 21
#define SCL_R 22

#define SDA_L 27
#define SCL_L 26

TwoWire I2CR = TwoWire(0);
TwoWire I2CL = TwoWire(1);

bool mode_silencieu = true;
const int MAX_LENGTH = 20; char incoming_Byte = '\r'; String text("");

bool is_end_of_line(char c){ return c == '\r' or c == '\n'; }
bool read_line(){
  if(is_end_of_line(incoming_Byte) or text.length() == MAX_LENGTH){
    text = ""; incoming_Byte = 'a';
  }
  while(Serial.available() and text.length() < MAX_LENGTH){
    incoming_Byte = Serial.read();
    if( !mode_silencieu ){
    Serial.print(incoming_Byte);
    }
    if( is_end_of_line(incoming_Byte) ){  if( !mode_silencieu ){Serial.println("FIN");} break; }
    text += incoming_Byte;
  }
  return is_end_of_line(incoming_Byte) or text.length() == MAX_LENGTH;
}

uint16_t command_L = 0;
uint16_t command_R = 0;
uint16_t command_L_r = 0;
uint16_t command_R_r = 0;
uint8_t i = 0;
uint8_t j = 0;
int pota=0;
int octo1 = 5;
int octo2 = 13;
int pin_pota = 32;
int ch2 = 35;
int ch1 = 34;
int command_radio;
int interval_depart=-1; //-1 pour désacctiver, 20 pour bien bloquer les roues en statique
int sens_R = NULL;
int sens_L = NULL;
int centre_command[] = {65535,65535};  //65280,65417

int hallR1=18;
int hallR2=17;
int hallR3=16;

int hallL1=2;
int hallL2=4;
int hallL3=15;

uint8_t etatR = 0b0;
uint8_t etatL = 0b0;

float tab_vitesse[64]= {0};

volatile int position_datas=0;
int datas[2000]={0};

const int MIN_CH_L = 1294;
const int CENTER_CH_L = 1530;
const int MAX_CH_L = 1789;

const int MIN_CH_R = 1309;
const int CENTER_CH_R = 1540;
const int MAX_CH_R = 1800;

int vitesseR=CENTER_CH_R;
int vitesseL=CENTER_CH_L;

void onRequestR(){
  i=command_R;
  j=command_R>>8;
  I2CR.write(j);
  I2CR.write(i);
  I2CR.write(0x00);
  I2CR.write(0x88);
  I2CR.write(0xBD);
  I2CR.write(0x10);
  I2CR.write(0x00);
  I2CR.write(0x22);
  I2CR.write(0x00);
  I2CR.write(0x04);
  I2CR.write(0xFF);
  I2CR.write(0xD3);
  I2CR.write(0x0A);
  I2CR.write(0x00);
}

void onRequestL(){
  //Serial.println(command_L);
  i=command_L;
  j=command_L>>8;
  I2CL.write(j);
  I2CL.write(i);
  I2CL.write(0x00);
  I2CL.write(0x88);
  I2CL.write(0xBD);
  I2CL.write(0x10);
  I2CL.write(0x00);
  I2CL.write(0x22);
  I2CL.write(0x00);
  I2CL.write(0x04);
  I2CL.write(0xFF);
  I2CL.write(0xD3);
  I2CL.write(0x0A);
  I2CL.write(0x00);
}

void onReceiveR(int len){
  while(I2CR.available()){
    I2CR.read();
  }
}

void onReceiveL(int len){
  while(I2CL.available()){
    I2CL.read();
  }
}



const unsigned int queue_capacity = 4; // have to be a power of two
Decimation_queue<REAL_TYPE, queue_capacity> output_queueR(decimation_factor);
//Decimation_queue<REAL_TYPE, queue_capacity> output_queue_raw(decimation_factor);
Decimation_queue<REAL_TYPE, queue_capacity> output_queueL(decimation_factor);
Decimation_queue<REAL_TYPE, queue_capacity> output_queueUL(decimation_factor);
Decimation_queue<REAL_TYPE, queue_capacity> output_queueUR(decimation_factor);

TimerHandle_t timer;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

int radio(int ch, int octo ,int min_ch, int max_ch, int centre_ch, int* psens, int command_serial){
  int command_ch = command_serial;
  
  int command;
  if(command_ch>=1530){ //1530 position centrale pota telecommande
    command=map(command_ch,centre_ch,min_ch,65280,49408);//49408
    *psens=1;
  }
  else{
    command=map(command_ch,centre_ch,max_ch,0,17408);
    *psens=0;
  }
  if(command_ch==0){
    command=65535;
  }

  if((centre_ch-interval_depart)<=command_ch && (centre_ch+interval_depart)>=command_ch){
    digitalWrite(octo,HIGH); //HIGH pour couper les moteurs lorsque ceux ci ont un ordre neutre, LOW pour le pas avoir d'effet de coupure lors des tests
  }
  else{
    digitalWrite(octo,LOW); 
  }
  
  return command;
}


uint8_t test_hall(int hall1, int hall2, int hall3){
  uint8_t res=0b0;
  if(digitalRead(hall1)){
    res+=0b00000100;
  }
  if(digitalRead(hall2)){
    res+=0b00000010;
  }
  if(digitalRead(hall3)){
    res+=0b00000001;
  }
  return res;
}

void construct_tab_vitesse(float* tab_vitesse){
  //+1
  tab_vitesse[(int)0b001101]=1;
  tab_vitesse[(int)0b101100]=1;
  tab_vitesse[(int)0b100110]=1;
  tab_vitesse[(int)0b110010]=1;
  tab_vitesse[(int)0b010011]=1;
  tab_vitesse[(int)0b011001]=1;

  //-1
  tab_vitesse[(int)0b101001]=-1;
  tab_vitesse[(int)0b100101]=-1;
  tab_vitesse[(int)0b110100]=-1;
  tab_vitesse[(int)0b010110]=-1;
  tab_vitesse[(int)0b011010]=-1;
  tab_vitesse[(int)0b001011]=-1;

  //+2
  tab_vitesse[(int)0b011101]=2;
  tab_vitesse[(int)0b001100]=2;
  tab_vitesse[(int)0b101110]=2;
  tab_vitesse[(int)0b100010]=2;
  tab_vitesse[(int)0b110011]=2;
  tab_vitesse[(int)0b010001]=2;

  //-2
  tab_vitesse[(int)0b101011]=-2;
  tab_vitesse[(int)0b100001]=-2;
  tab_vitesse[(int)0b110101]=-2;
  tab_vitesse[(int)0b010100]=-2;
  tab_vitesse[(int)0b011110]=-2;
  tab_vitesse[(int)0b001010]=-2;
  

}

void IRAM_ATTR make_a_sample(TimerHandle_t pxTimer) {

  etatR &= 0b00000111;
  etatR <<= 3;
  etatR += test_hall(hallR1,hallR2,hallR3);

  etatL &= 0b00000111;
  etatL <<= 3;
  etatL += test_hall(hallL1,hallL2,hallL3);
  
    REAL_TYPE raw_valueR = tab_vitesse[etatR];
    REAL_TYPE raw_valueL = tab_vitesse[etatL];
    
    // raw_value *= (((REAL_TYPE) 1.0)/max_adc_value);
    filterR.append(raw_valueR);
    filterL.append(raw_valueL);
    portENTER_CRITICAL_ISR(&timerMux);
    output_queueR.append(filterR.get_value());
    output_queueL.append(filterL.get_value());
    output_queueUR.append(vitesseR);
    output_queueUL.append(vitesseL);
    //output_queue_raw.append(raw_value);
    portEXIT_CRITICAL_ISR(&timerMux);
}


const unsigned long int ESP32_clock = 80000000; // 80 MHz
const unsigned int timer_id = 0; // Esp32 have 4 avalaible timers : timer 0, 1, 2 and 3
const unsigned long int timer_resolution = 1000;
const unsigned long int timer_prescalar = ESP32_clock/(timer_resolution*sample_frequence);
static_assert(2 <= timer_prescalar );
static_assert(timer_prescalar <= 65536);
static_assert(timer_prescalar*timer_resolution*sample_frequence == ESP32_clock); //Adapt timer_resolution

unsigned int new_time, last_time;
void setup() {

  I2CR.begin(address,SDA_R, SCL_R, 140000); 
  I2CL.begin(address,SDA_L, SCL_L, 140000);
  I2CR.setTimeOut(200);
  I2CL.setTimeOut(200);

  pinMode((uint8_t)pin_pota, INPUT);
  pinMode((uint8_t)octo1, OUTPUT);
  pinMode((uint8_t)octo2, OUTPUT);
  pinMode((uint8_t)ch2, INPUT);
  pinMode((uint8_t)ch1, INPUT);
  
  pinMode((uint8_t)hallR1, INPUT);
  pinMode((uint8_t)hallR2, INPUT);
  pinMode((uint8_t)hallR3, INPUT);
  pinMode((uint8_t)hallL1, INPUT);
  pinMode((uint8_t)hallL2, INPUT);
  pinMode((uint8_t)hallL3, INPUT);
  
  Serial.begin(115200);
  
  I2CR.onReceive(onReceiveR);
  I2CR.onRequest(onRequestR);

  I2CL.onReceive(onReceiveL);
  I2CL.onRequest(onRequestL);

  construct_tab_vitesse(tab_vitesse);


  timer = xTimerCreate("Timer", pdMS_TO_TICKS(2), pdTRUE, (void *)0, make_a_sample);
  xTimerStart(timer, 0);

  filterR.reset();
  filterL.reset();
  last_time = micros();
}

REAL_TYPE output_valueR, output_value_raw, output_valueL, output_valueUL, output_valueUR;
bool loss_data, loss_data_raw;
const unsigned int sample_period = 1000000 / output_frequence;

const float nb_pair_of_pole = 15;
const float cycle_hall_sie = 6;
const float velocity_factor = output_frequence/(nb_pair_of_pole*cycle_hall_sie);

bool loglog = true;

void loop(){
  command_L = radio(ch2, octo2, MIN_CH_L, MAX_CH_L, CENTER_CH_L, &sens_L, vitesseL);
  command_R = radio(ch1, octo1, MIN_CH_R, MAX_CH_R, CENTER_CH_R, &sens_R, vitesseR);
  new_time = micros();
  if( new_time - last_time > (sample_period/4) ){
    portENTER_CRITICAL(&timerMux);
    output_queueUL.get_avalaible_value(output_valueUL, loss_data);
    output_queueUR.get_avalaible_value(output_valueUR, loss_data);
    bool have_new_valueR = output_queueR.get_avalaible_value(output_valueR, loss_data);
    bool have_new_valueL = output_queueL.get_avalaible_value(output_valueL, loss_data);

    // output_queue_raw.get_avalaible_value(output_value_raw, loss_data_raw);
    portEXIT_CRITICAL(&timerMux);
    if( have_new_valueR  ){
      if(loglog){
        Serial.print(output_valueR * velocity_factor, 6); Serial.print(","); Serial.print(output_valueL * velocity_factor, 6); Serial.print(","); Serial.print(output_valueUR); Serial.print(","); Serial.println(output_valueUL);
        //Serial.print(", ");
        //Serial.println(output_value_raw, 6);
      }
      if(loss_data) Serial.println("Data lost");
    }
    last_time = new_time;
  }
  while( read_line() ){
    if(!mode_silencieu){
      Serial.print( "Received line : "); Serial.println(text);
    }
    if (text.startsWith("em")){
      vitesseR = CENTER_CH_R;
    }else if(text.startsWith("velR ")){
      text.remove(0, text.indexOf(' ')+1);
      int valueR = text.toInt();
      if( valueR < MIN_CH_R or valueR > MAX_CH_R ){
        Serial.println("Bad value !");
      }else{
        vitesseR = valueR;
      }
      if (!mode_silencieu){
        Serial.print("Setting velocity to "); Serial.print(vitesseR);
      }
      }else if(text.startsWith("velL ")){
      text.remove(0, text.indexOf(' ')+1);
      int valueL = text.toInt();
      if( valueL < MIN_CH_L or valueL > MAX_CH_L ){
        Serial.println("Bad value !");
      }else{
        vitesseL = valueL;
      }
      if (!mode_silencieu){
        Serial.print("Setting velocity to "); Serial.print(vitesseL);
      }
    }else if(text.startsWith("log")){
      loglog = ! loglog;
    }else if(text.startsWith("silence")){
      mode_silencieu = ! mode_silencieu;
    }else if(text.startsWith("help")) {
      Serial.println("");
      Serial.println("log : start or end log.");
      Serial.println("");
      Serial.println("velR VALUE : set velocity R to VALUE.");
      Serial.println("");
      Serial.println("em : Emergency STOP !");
      Serial.println("");
      Serial.println("silence : start or end mode silencieux.");    
    }
  }
}
