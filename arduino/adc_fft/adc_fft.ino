#include <Arduino.h>
#include "STMSpeeduino.h"

#define D86 LED_RED
#define D87 LED_GREEN
#define D88 LED_BLUE

#define NSAMPLES 16384
#define NBUF 10
#define P 14 // NSAMPLES = 2**P

#define DATA_SIZE 32768 // tableau de 8192 float
#define GET_DATA 10
#define SET_DATA 11
uint8_t data[DATA_SIZE]; // buffer used to send the floats of fft_amp over serial

float A_re[NSAMPLES];
float A_im[NSAMPLES];
float B_re[NSAMPLES];
float B_im[NSAMPLES];

float offset = 65536/2;
float num2volt = 3.217/65536;

uint16_t indice;
float fft_amp[NSAMPLES/2];

uint16_t FrameBuffer1[NSAMPLES]; // I
uint16_t FrameBuffer2[NSAMPLES]; // Q

int ADC1_channel  = A0;
int ADC2_channel  = A1;
int resolution    = 16;
int clk_speed_MHz = 40;
int sample_time   = 0;  // measure fs is around 4.3MSPS (per ADC)
int sample_num    = 0;

int inversion(uint16_t i) {
  uint8_t bi;
  int8_t b;
  uint8_t bits[P];
  for (b=0; b<P; b++) {
    bi = i & 0b1;
    i = i >> 1;
    bits[b] = bi;
  }
  uint16_t facteur = 1;
  uint16_t j = 0;
  for (b=P-1; b>=0; b--) {
    j += bits[b]*facteur;
    facteur *= 2;
  }
  return j;
} 

void fft() {
  uint16_t j;
  for (int k=0; k<NSAMPLES; k++) {
      j = inversion(k);
      B_re[j] = A_re[k];
      B_im[k] = A_im[k];
  }
  uint16_t taille = 1;
  uint16_t taille_precedente;
  uint16_t nombre_tfd = NSAMPLES;
  uint16_t position;
  float phi, W_re, W_im;;
  float x,y;
  uint8_t parite = 0;
  for (uint16_t q=0; q<P; q++) {
      taille_precedente = taille;
      taille *= 2;
      nombre_tfd /= 2;
      for (uint16_t m=0; m<nombre_tfd; m++) {
          phi = -2*PI/taille;
          position = m*taille;
          uint16_t i,j,k;
          for (i=0; i<taille_precedente; i++) {
              W_re = cos(phi*i);
              W_im = sin(phi*i);
              j = position+taille_precedente+i;
              k = position +i;
              if (parite==0) {
                x = B_re[j];
                y = B_im[j];
                A_re[k] = B_re[k] + x*W_re-y*W_im;
                A_im[k] = B_im[k] + x*W_im+y*W_re;
              }
              else {
                x = A_re[j];
                y = A_im[j];
                B_re[k] = A_re[k] + x*W_re-y*W_im;
                B_im[k] = A_im[k] + x*W_im+y*W_re;
              }
          }
          for (i=taille_precedente; i<taille; i++) {
              W_re = cos(phi*i);
              W_im = sin(phi*i);
              j = position+i;
              k = position+i-taille_precedente;
              if (parite==0) {
                x = B_re[j];
                y = B_im[j];
                A_re[j] = B_re[k] + x*W_re-y*W_im;
                A_im[j] = B_im[k] + x*W_im+y*W_re;
              }
              else {
                x = A_re[j];
                y = A_im[j];
                B_re[j] = A_re[k] + x*W_re-y*W_im;
                B_im[j] = A_im[k] + x*W_im+y*W_re;
              }   
          }  
      }
      parite = ~parite;
  }
  if (parite) {
      for (int k=0; k<NSAMPLES; k++) {B_re[k] = A_re[k]; B_im[k] = A_im[k];}
  }
}             


void setup() 
{
  pinMode(13, OUTPUT); // D13
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Serial.begin(115200);
  while(!Serial);
  Serial.flush();

  uint8_t c = 0;
  Serial.write(c); 

  for(int i = 0; i < NSAMPLES; i++) A_im[i] = 0.0;

  ADCSimultaneous(ADC1_channel, ADC2_channel, resolution, clk_speed_MHz, sample_time, sample_num); 

  AttachADC_DMA(ADC1DMA, NSAMPLES, (uint16_t *) FrameBuffer1, DMAS5);
  AttachADC_DMA(ADC2DMA, NSAMPLES, (uint16_t *) FrameBuffer2, DMAS6);

  ADC_Start(ADC1);
}

void loop() 
{
  GPIOH->BSRR |= GPIO_BSRR_BS6; // digitalWrite(13, HIGH)

  while(!TransferADCComplete(ADC1DMA) || !TransferADCComplete(ADC2DMA)) {}; 

  for(uint16_t i = 0; i < NSAMPLES; i++)
  {
    A_re[i] = (FrameBuffer1[i] - offset) * num2volt;
    A_im[i] = 0.0;
  }

  fft();

  for(uint16_t i = 0; i < NSAMPLES / 2; i++)
  {
    fft_amp[i] = sqrt(B_re[i] * B_re[i] + B_im[i] * B_im[i]) * 2.0 / NSAMPLES;
  }

  if (Serial.available()) 
  {
    char c = Serial.read();
    if (c == 'G') {
      // send raw bytes of the float array
      Serial.write((uint8_t*)fft_amp, sizeof(fft_amp)); // 8192 * 4 = 32768 bytes
      Serial.flush();  // block until everything is sent
    }
  }

  recaptureADCvalues(ADC2DMA); 
  recaptureADCvalues(ADC1DMA); 

  GPIOH->BSRR |= GPIO_BSRR_BR6; // digitalWrite(13, LOW)

  delay(10); 
  digitalWrite(LED_BLUE, LOW);
  delay(10);
  digitalWrite(LED_BLUE, HIGH);
}
