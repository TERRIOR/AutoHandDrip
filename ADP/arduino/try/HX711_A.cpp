#include <Arduino.h>
#include "HX711_A.h"

HX711::HX711(byte sck, byte dout, byte amp, double co) {
    SCK = sck;
    DOUT = dout;
    set_amp(amp);
    COEFFICIENT = co;
    pinMode(SCK, OUTPUT);
    pinMode(DOUT, INPUT);
    digitalWrite(SCK, LOW);
    read();
}

void HX711::set_amp(byte amp) {
    switch (amp) {
        case 32: AMP = 2; break;
        case 64: AMP = 3; break;
        case 128: AMP = 1; break;
    }
}

bool HX711::is_ready() {
    return digitalRead(DOUT) == LOW;
}

long HX711::read() {
    long val = 0;
    while (!is_ready());
    for (int i = 0; i < 24; i++) {
        pulse1(SCK);
        val <<= 1;
        if (digitalRead(DOUT) == HIGH) val++;
    }
    for (int i = 0; i < AMP; i++) {
        pulse1(SCK);
    }
    return val & (1L << 23) ? val | ((-1L) << 24) : val;
}

double HX711::bias_read() {
    return (read() - OFFSET) * COEFFICIENT;
}

double HX711::tare(int t) {
    double sum = 0;
    for (int i = 0; i < t; i++) {
        sum += read();
    }
    set_offset(sum / t);
     return sum / t;
}

void HX711::set_offset(long offset) {
    OFFSET = offset;
}

void HX711::set_co(double co) {
    COEFFICIENT = co;
}
void HX711::average_filter(float c,float *input){
    int i;
    int error = 0.5;
    float temp;
      temp = c;
      Buffer[TFILTER_N - 1] = temp;
  if(((Buffer[TFILTER_N - 1] - Buffer[TFILTER_N - 2]) < error) || ((Buffer[TFILTER_N - 2] - Buffer[TFILTER_N - 1]) < error))
    {//超出范围
      Buffer[TFILTER_N - 1] = Buffer[TFILTER_N - 2];//当作误差舍弃
      over_cnt++;//每次超出范围就加一
      if(over_cnt>=6){
        Buffer[TFILTER_N - 1] = temp;//当四次都超出范围后，重新更新buffer[TFILTER_N - 1],把当前温度作为有效值
        over_cnt=0;//计数器清0
      }
    }
    else {
      over_cnt=0;//计数器清0
    }
    for(i = 0; i < TFILTER_N - 1; i++) {
    Buffer[i] =Buffer[i + 1];
    filter_sum += Buffer[i]; 
  }
    //tp->buffer[TFILTER_N - 2]=filter_sum / (TFILTER_N - 1);
    *input= filter_sum / (TFILTER_N - 1);
    filter_sum = 0;//filter_sum清零
}
