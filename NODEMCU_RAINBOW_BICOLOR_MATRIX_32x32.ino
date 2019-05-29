//************************************************************************************************************// 
//*********************** NODEMCU ESP8266 WITH THE 32x32 BICOLOR MATRIX USING B.A.M METHOD *******************//
//************************************************************************************************************//

#include <Arduino.h>
#include <SPI.h>
#include "bitmap.h"
#include "font8x8.h"
#include "font8x16.h"
#include "vnfont8x16.h"

// REAL PIN ON NODEMCU

#define blank_pin       D8  // BLANK PIN - 74HC595
#define latch_pin       D4  // LATCH PIN - 74HC595
#define clock_pin       D5  // CLOCK PIN - 74HC595
#define data_pin        D7  // DATA PIN - 74HC595

#define RowA_Pin        D0  // A PIN - 74HC238
#define RowB_Pin        D1  // B PIN - 74HC238
#define RowC_Pin        D2  // C PIN - 74HC238
#define RowD_Pin        D3  // D PIN - 74HC238
//#define OE_Pin        D8  // ENABLE OUTPUT PIN - 74HC238 (Connect to blank_pin).

//MAPPING TO PORT

#define Blank_Pin_Bit     15  // GPIO15
#define Latch_Pin_Bit     2   // GPIO2
#define Clock_Pin_Bit     14  // GPIO14
#define Data_Pin_Bit      13  // GPIO13

#define RowA_Pin_Bit      16  // GPIO16
#define RowB_Pin_Bit      5   // GPIO5
#define RowC_Pin_Bit      4   // GPIO4
#define RowD_Pin_Bit      0   // GPIO0
//#define OE_Pin_Bit      15  // GPIO15

//***************************************************CONST************************************************//

#define myPI                  3.14159265358979323846
#define myDPI                 1.2732395
#define myDPI2                0.40528473
#define dist(a, b, c, d)      sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))
#ifndef _swap_int16_t
#define _swap_int16_t(a, b)   { int16_t t = a; a = b; b = t; }
#endif
//*************************************************ColorWheel******************************************//

#define COLOUR_WHEEL_LENGTH 128

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G;

//***************************************************B.A.M**************************************************//
#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G (256 colours)

const  byte Size_Y = 32;    //Number of LEDS in Y axis (Top to Bottom)
const  byte Size_X = 32;    //Number of LEDs in X axis (Left to Right)

byte red[4][128];
byte green[4][128];

int level=0;                  //Keeps tracking of which level we are shifting data to
int row=0;
int BAM_Bit, BAM_Counter=0;   // Bit Angle Modulation variables to keep track of things

/********************************************* AN RG COLOR TEMPLATE **********************************************/
struct Color
{
  unsigned char red, green;

  Color(int r, int g) : red(r), green(g) {}
  Color() : red(0), green(0) {}
};

const Color redcolor       = Color(0x0F,0x00);
const Color orangecolor    = Color(0x0F,0x0F);
const Color yellowcolor    = Color(0x0F,0x09);
const Color greencolor     = Color(0x00,0x0F);
const Color clearcolor     = Color(0x00,0x00);

#define RED     0x0F,0x00
#define ORANGE  0x0F,0x04
#define YELLOW  0x0F,0x09
#define GREEN   0x00,0x0F
#define CLEAR   0x00,0x00

//*************************************************ALL FUNNCTIONS******************************************//

void LED(int X, int Y, int R, int G);
void timer1_ISR(void);
void clearfast();
void fill_colour_wheel(void);
void get_colour(int16_t p, uint8_t *R, uint8_t *G);
void get_next_colour(uint8_t *R, uint8_t *G);
void increment_colour_pos(uint8_t i);

void hScroll(uint8_t y, Color For_color, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir);
void hScroll_colorwheel(uint8_t y, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir);
void HScrollBigImageL(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, Color For_color, Color Bk_color, uint16_t delaytime );
void HScrollBigImageL_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, uint16_t delaytime );
void drawImage(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, Color For_color, Color Bk_color);
void drawImage_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image);
void drawImage_colorwheelX(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image);
void drawImage_colorwheelY(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image);
void drawImage_inv_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image);
void drawImage_inv_colorwheelY(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image);

void EffectMarquee(uint8_t y, wchar_t *mystring);
byte getPixelHString(uint16_t x, uint16_t y, wchar_t *p,uint8_t font);
unsigned int lenString(wchar_t *p);

void fillTable(byte R, byte G);
void fillTable_colorwheel();
void fillTable_colorwheelXY();

float mySin(float x);
float myCos(float x);
float myTan(float x);
float mySqrt(float in);
float myMap(float in, float inMin, float inMax, float outMin, float outMax);
int16_t myRound(float in);
float myAbs(float in);

void drawVLine(uint16_t x, uint16_t y1, uint16_t y2, Color color);
void drawHLine(uint16_t x1, uint16_t x2, uint16_t y, Color color);
void drawFastVLine(int16_t x, int16_t y,int16_t h, Color color);
void drawFastHLine(int16_t x, int16_t y,int16_t w, Color color);
void drawLine_colorwheel(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color);
void drawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, Color color);
void fillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, Color color);
void drawCircle(int xCenter, int yCenter, int radius, Color color);

//**********************************************************************************************************//

void setup()
{
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setFrequency(4000000);
  noInterrupts();
  
  timer1_isr_init();
  timer1_attachInterrupt(timer1_ISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(60);
  
  pinMode(latch_pin, OUTPUT);
  //pinMode(blank_pin, OUTPUT);
  pinMode(data_pin, OUTPUT);
  pinMode(clock_pin, OUTPUT);
  
  pinMode(RowA_Pin, OUTPUT);
  pinMode(RowB_Pin, OUTPUT);
  pinMode(RowC_Pin, OUTPUT);
  pinMode(RowD_Pin, OUTPUT);
  //digitalWrite(blank_pin, HIGH);
  interrupts();
  SPI.begin();
  fill_colour_wheel();
}

void loop()
{
  wchar_t scrolltext_1[]=L"    * WELCOME TO BICOLOR LED MATRIX 32x32-B.A.M 4 BIT WITH NODEMCU ESP8266, 74HC595 & 74HC238*    ";
  wchar_t scrolltext_2[]=L"    * Welcome to TUENHIDIY doityourself channel *    ";
  wchar_t scrolltext_3[]=L"    * Red Green ~ Colorwheel Matrix ~ Testing ! *    ";

  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 400, 32, INSTRUCTABLES, 20); 
  clearfast(); 
  drawImage_inv_colorwheel(0, 0, 32, 32, BUG) ;    
  delay(5000);
  clearfast();  
  hScroll_colorwheel(7, clearcolor, scrolltext_1, 0, 50, 1, 1); 
  clearfast();
  drawImage_inv_colorwheel(0, 0, 32, 32, Facebook) ;    
  delay(5000); 
  clearfast();
  hScroll_colorwheel(7, clearcolor, scrolltext_3, 0, 40, 1, 1); 
  clearfast();
  for(int i=0; i<50; i++)
  {
  drawImage_colorwheelX(0, 0, 32, 32, TIGER01);
  delay(110-2*i);
  drawImage_colorwheelX(0, 0, 32, 32, TIGER02);
  delay(110-2*i);
  drawImage_colorwheelX(0, 0, 32, 32, TIGER03);
  delay(110-2*i);
  drawImage_colorwheelX(0, 0, 32, 32, TIGER04);
  delay(110-2*i);
  drawImage_colorwheelX(0, 0, 32, 32, TIGER05);
  delay(110-2*i);
  drawImage_colorwheelX(0, 0, 32, 32, TIGER06);
  delay(110-2*i); 
  }
  clearfast();
  drawImage_colorwheel(0, 0, 32, 32, Git_Hub);  
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, Whatsapp, yellowcolor, greencolor);     
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, Twitter, greencolor, orangecolor) ;   
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, Apple, greencolor, orangecolor);   
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, Git_Hub, redcolor, clearcolor) ;     
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, Facebook, yellowcolor, greencolor) ;    
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, Instagram, greencolor, orangecolor) ;    
  delay(5000);
  clearfast();
  for(int i=0; i<20; i++){
  get_colour(colourPos + 10*i, &R, &G);
  drawImage(0, 0, 32, 32, ROT01, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT02, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT03, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT04, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT05, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT06, clearcolor, Color(R,G));
  delay(10);  
  drawImage(0, 0, 32, 32, ROT07, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT08, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT09, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT10, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT11, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT12, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT13, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT14, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT15, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT16, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT17, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT18, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT19, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT20, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT21, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT22, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT23, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT24, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT25, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT26, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT27, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT28, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT29, clearcolor, Color(R,G));
  delay(10);
  drawImage(0, 0, 32, 32, ROT30, clearcolor, Color(R,G));
  delay(10); 
};  
  clearfast();
  for(int i=0; i<10; i++){
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE00);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE01);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE02);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE03);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE04);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE05);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE06);
  delay(20);  
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE07);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE08);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE09);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE10);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE11);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE12);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE13);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE14);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE15);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE16);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE17);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE18);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE19);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE20);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE21);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE22);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE23);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE24);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE25);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE26);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE27);
  delay(20);
  drawImage_inv_colorwheelY(0, 0, 32, 32, FACE28);
  delay(20);
};
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 576, 32, CALL, 30); 
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 624, 32, YOUTUBE, 30); 
  clearfast();
  EffectMarquee(7, scrolltext_1); 
  clearfast();
  fillTable(2,6);
  hScroll_colorwheel(7, Color(2,6), scrolltext_2, 0, 40, 1, 1); 
  clearfast();
  drawImage(0, 0, 32, 32, BLACK_TREE, greencolor, orangecolor);
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, DARK_SKULL, greencolor, orangecolor);
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, WHITE_HOME, greencolor, orangecolor);
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, BOOK, greencolor, orangecolor);
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, CHAIR, greencolor, orangecolor);
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, INSTAGRAMM, greencolor, orangecolor);
  delay(5000);
  clearfast();
  drawImage(0, 0, 32, 32, TRASH, greencolor, orangecolor);
  delay(5000);  
  clearfast();  
}

void LED(int X, int Y, int R, int G)
{
  X = constrain(X, 0, Size_X - 1); 
  Y = constrain(Y, 0, Size_Y - 1);
  
  R = constrain(R, 0, (1 << BAM_RESOLUTION) - 1);
  G = constrain(G, 0, (1 << BAM_RESOLUTION) - 1); 

  int WhichByte = int(Y*4+ X/8);
  int WhichBit = 7-(X%8);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][WhichByte], WhichBit, bitRead(R, BAM));

    bitWrite(green[BAM][WhichByte], WhichBit, bitRead(G, BAM));
  }
}

void rowScan(byte row)
{  
  if (row & 0x01) digitalWrite(RowA_Pin,HIGH);
    else          digitalWrite(RowA_Pin,LOW);
  if (row & 0x02) digitalWrite(RowB_Pin,HIGH);   
    else          digitalWrite(RowB_Pin,LOW);          
  if (row & 0x04) digitalWrite(RowC_Pin,HIGH);   
    else          digitalWrite(RowC_Pin,LOW);          
  if (row & 0x08) digitalWrite(RowD_Pin,HIGH);   
    else          digitalWrite(RowD_Pin,LOW);  
}

void ICACHE_RAM_ATTR timer1_ISR(void)
{
  digitalWrite(blank_pin, HIGH);  // Set BLANK PIN high - 74HC595 & 74HC238               
  if(BAM_Counter==6)    // Bit weight 2^0 of BAM_Bit, lasting time = 6 ticks x interrupt interval time
  BAM_Bit++;
  else
  if(BAM_Counter==18)   // Bit weight 2^1 of BAM_Bit, lasting time = 18 ticks x interrupt interval time
  BAM_Bit++;
  else
  if(BAM_Counter==42)   // Bit weight 2^3 of BAM_Bit, lasting time = 42 ticks x interrupt interval time
  BAM_Bit++;
  BAM_Counter++;
  switch (BAM_Bit)
    {
    case 0:
      //Red     
        SPI.transfer(red[0][level + 0]);
        SPI.transfer(red[0][level + 1]);
        SPI.transfer(red[0][level + 2]);
        SPI.transfer(red[0][level + 3]);
        SPI.transfer(red[0][level + 64]);
        SPI.transfer(red[0][level + 65]);
        SPI.transfer(red[0][level + 66]);
        SPI.transfer(red[0][level + 67]);
      //Green       
        SPI.transfer(green[0][level + 0]);
        SPI.transfer(green[0][level + 1]);
        SPI.transfer(green[0][level + 2]);
        SPI.transfer(green[0][level + 3]);
        SPI.transfer(green[0][level + 64]);
        SPI.transfer(green[0][level + 65]);
        SPI.transfer(green[0][level + 66]);
        SPI.transfer(green[0][level + 67]);      
      break;
    case 1:      
      //Red
        SPI.transfer(red[1][level + 0]);
        SPI.transfer(red[1][level + 1]);
        SPI.transfer(red[1][level + 2]);
        SPI.transfer(red[1][level + 3]);  
        SPI.transfer(red[1][level + 64]);
        SPI.transfer(red[1][level + 65]);
        SPI.transfer(red[1][level + 66]);
        SPI.transfer(red[1][level + 67]);             
      //Green
        SPI.transfer(green[1][level + 0]);
        SPI.transfer(green[1][level + 1]);
        SPI.transfer(green[1][level + 2]);
        SPI.transfer(green[1][level + 3]);        
        SPI.transfer(green[1][level + 64]);
        SPI.transfer(green[1][level + 65]);
        SPI.transfer(green[1][level + 66]);
        SPI.transfer(green[1][level + 67]);
      break;
    case 2:     
      //Red
        SPI.transfer(red[2][level + 0]);
        SPI.transfer(red[2][level + 1]);
        SPI.transfer(red[2][level + 2]);
        SPI.transfer(red[2][level + 3]);      
        SPI.transfer(red[2][level + 64]);
        SPI.transfer(red[2][level + 65]);
        SPI.transfer(red[2][level + 66]);
        SPI.transfer(red[2][level + 67]);                    
       //Green
        SPI.transfer(green[2][level + 0]);
        SPI.transfer(green[2][level + 1]);
        SPI.transfer(green[2][level + 2]);
        SPI.transfer(green[2][level + 3]);       
        SPI.transfer(green[2][level + 64]);
        SPI.transfer(green[2][level + 65]);
        SPI.transfer(green[2][level + 66]);
        SPI.transfer(green[2][level + 67]);
      break;
    case 3:
      //Red
        SPI.transfer(red[3][level + 0]);
        SPI.transfer(red[3][level + 1]);
        SPI.transfer(red[3][level + 2]);
        SPI.transfer(red[3][level + 3]);               
        SPI.transfer(red[3][level + 64]);
        SPI.transfer(red[3][level + 65]);
        SPI.transfer(red[3][level + 66]);
        SPI.transfer(red[3][level + 67]);    
      //Green
        SPI.transfer(green[3][level + 0]);
        SPI.transfer(green[3][level + 1]);
        SPI.transfer(green[3][level + 2]);
        SPI.transfer(green[3][level + 3]);              
        SPI.transfer(green[3][level + 64]);
        SPI.transfer(green[3][level + 65]);
        SPI.transfer(green[3][level + 66]);
        SPI.transfer(green[3][level + 67]);        
    if(BAM_Counter==90)    //Bit weight 2^3 of BAM_Bit, lasting time = 90 ticks x interrupt interval time
    {
    BAM_Counter=0;
    BAM_Bit=0;
    }
    break;
  }
  rowScan(row);
  digitalWrite(latch_pin, HIGH);    // Set LATCH PIN low - 74HC595
  digitalWrite(latch_pin, LOW);     // Set LATCH PIN low - 74HC595
  digitalWrite(blank_pin, LOW);     // Set BLANK PIN low - 74HC595 & 74HC238
  row++;
  level = row<<2;
  if(row==16)
  row=0;
  if(level==64)
  level=0;
  //pinMode(blank_pin, OUTPUT);
  timer1_write(60);     //Interrupt will be called every 60 x 0.2us = 12us
}

void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 128);
    memset(green, 0, sizeof(green[0][0]) * 4 * 128);
}

void fillTable(byte R, byte G)
{
    for (byte x=0; x<32; x++)
    {
      for (byte y=0; y<32; y++)
      {
        LED(x, y, R, G);
      }
    }
}

void EffectMarquee(uint8_t y, wchar_t *mystring)
{
// FONT 8x8
    for (int offset=0; offset <((lenString(mystring)-8)*8-1); offset++)
      {
      
      for (byte xx=0; xx<32; xx++)
        {
          for (byte yy=0; yy<16; yy++)
              {
                Color setcolor, For_color=clearcolor, Bk_color;                 
                get_colour(colourPos + 2*(xx+yy), &Bk_color.red, &Bk_color.green);                
                if (getPixelHString(xx+offset, yy, mystring,0)) 
                {
                  setcolor = For_color;
                } 
                else setcolor = Bk_color;
                  LED(xx, (yy+y), setcolor.red, setcolor.green);
              }  
                                  
        }
      increment_colour_pos(1); 
      delay(50);       
      }
}

byte getPixelChar(uint8_t x, uint8_t y, wchar_t ch,uint8_t font)

{
  if (font==0)
  {
    //ch = ch-32;
    if (x > 7) return 0; // 4 = font Width -1
    
    if ((ch >=32) && (ch <= 127))
    return bitRead(pgm_read_byte(&font8x16[ch-32][y]),7-x);
    
    if ((ch >=192) && (ch <= 273))
    return bitRead(pgm_read_byte(&font8x16[ch-96][y]),7-x);
    
    if ((ch >=296) && (ch <= 297))
    return bitRead(pgm_read_byte(&font8x16[ch-118][y]),7-x);
    
    if ((ch >=360) && (ch <= 361))
    return bitRead(pgm_read_byte(&font8x16[ch-180][y]),7-x);

    if ((ch >=416) && (ch <= 417))
    return bitRead(pgm_read_byte(&font8x16[ch-234][y]),7-x);
     
    if ((ch >=431) && (ch <= 432))
    return bitRead(pgm_read_byte(&font8x16[ch-247][y]),7-x);
    
    if ((ch >=7840) && (ch <= 7929))
    return bitRead(pgm_read_byte(&vnfont8x16[ch-7840][y]),7-x);  
  }
  else if (font==1)
  {
    //ch = ch-32;
    if (x > 7) return 0; // 4 = font Width -1
    return bitRead(pgm_read_byte(&font8x8[ch-32][y]),7-x);  
  }
  else if (font==2)
  {
    //ch = ch-32;
    if (x > 7) return 0; // 4 = font Width -1
    return bitRead(pgm_read_byte(&font8x16[ch-32][y]),7-x);  
  }
  
}

byte getPixelHString(uint16_t x, uint16_t y, wchar_t *p,uint8_t font)

{
  if (font==0)
  {
    p=p+x/8;
    return getPixelChar(x%8,y,*p,0);
  }
  else if (font==1)
  {
    p=p+x/7;
    return getPixelChar(x%7,y,*p,1);  
  }

  else if (font==2)
  {
    p=p+x/8;
    return getPixelChar(x%8,y,*p,2); 
  }
}

unsigned int lenString(wchar_t *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}

void hScroll(uint8_t y, Color For_color, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
{
  //int offset =0;
  // FONT 5x7
  int offset;
  Color color;
  if (font == 0)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*8-1) ; (dir) ? offset <((lenString(mystring)-8)*8-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<32; xx++)
        {
        for (byte yy=0; yy<16; yy++)
            {            
              if (getPixelHString(xx+offset,yy,mystring,0))
              {
                color = For_color;                
              }
              else 
              {
                color=Bk_color;
              }
                LED(xx,(yy+y),color.red, color.green);
            }
        }
        delay(delaytime);  
      }
    times--;
    }
  } 
// FONT 8x8
  else if (font == 1)
    {
    while (times)
      {
      for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*7-1); (dir) ? offset <((lenString(mystring)-8)*7-1): offset >0; (dir) ? offset++ : offset--)
        {
        for (byte xx=0; xx<32; xx++)
          {
            for (byte yy=0; yy<8; yy++)
              {
                if (getPixelHString(xx+offset,yy,mystring,1)) 
                  {
                  color = For_color;
                  }
                else 
                {
                  color = Bk_color;
                }
                  LED(xx,(yy+y),color.red, color.green);
              }          
            }
      delay(delaytime);  
        }
      times--;
      }
    }
// FONT 8x16  
   else if (font == 2)
    {
    while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*8-1); (dir) ? offset <((lenString(mystring)-8)*8-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<32; xx++)
        {     
            for (byte yy=0; yy<16; yy++)
              {
                if (getPixelHString(xx+offset,yy,mystring,2)) 
                  {
                  color = For_color;
                  }
                else 
                {
                  color=Bk_color;
                }
                  LED(xx,(yy+y),color.red, color.green);
              }   
          }
          delay(delaytime);  
        }
        times--;
      } 
    }
 }

void hScroll_colorwheel(uint8_t y, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
{
  //int offset =0;
  // FONT 5x7
  int offset;
  Color setcolor, For_color;
  if (font == 0)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*8-1) ; (dir) ? offset <((lenString(mystring)-8)*8-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<32; xx++)
        {
        for (byte yy=0; yy<16; yy++)
            {
              get_colour(colourPos + 3*(xx), &For_color.red, &For_color.green);
              if (getPixelHString(xx+offset,yy,mystring,0))
              {
                setcolor = For_color;
              }                
            
              else 
              {
                setcolor=Bk_color;
              }
                LED(xx,(yy+y),setcolor.red, setcolor.green);
            }
        }
        delay(delaytime); 
      }
    times--;
    }
  } 
// FONT 8x8
  else if (font == 1)
    {
    while (times)
      {
      for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*7-1); (dir) ? offset <((lenString(mystring)-8)*7-1): offset >0; (dir) ? offset++ : offset--)
        {
        for (byte xx=0; xx<32; xx++)
          {
            for (byte yy=0; yy<8; yy++)
              {
                get_colour(colourPos + 8*(xx), &For_color.red, &For_color.green);
                if (getPixelHString(xx+offset,yy,mystring,1)) 
                  {
                  setcolor = For_color;
                  }
                else 
                {
                  setcolor = Bk_color;
                }
                  LED(xx,(yy+y), setcolor.red, setcolor.green);
                }          
            }
      delay(delaytime); 
        }
      times--;
      }
    }

// FONT 8x16  
   else if (font == 2)
    {
    while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*8-1); (dir) ? offset <((lenString(mystring)-8)*8-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<32; xx++)
        {     
            for (byte yy=0; yy<16; yy++)
              {
                get_colour(colourPos + 3*(xx), &For_color.red, &For_color.green);
                if (getPixelHString(xx+offset,yy,mystring,2)) 
                  {
                  setcolor = For_color;
                  }
                else 
                {
                  setcolor=Bk_color;
                }
                  LED(xx,(yy+y),setcolor.red, setcolor.green);
              }   
          }
          delay(delaytime); 
        }
        times--;
      } 
    }
 }

 void HScrollBigImageL(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, Color For_color, Color Bk_color, uint16_t delaytime )
{
    for (uint16_t i= 0; i < width-32; i++)
    {
    for (uint16_t y = 0; y < height; y++)
      {
        for (uint16_t x = 0; x < 32; x++)
        {
            uint16_t   myindex = (i+x)/8 + y * (width / 8);
            uint16_t   mybitmask = 7-((i+x) % 8);
            uint16_t   colorImage = bitRead(pgm_read_byte(&image[myindex]), mybitmask) & 1;
              if (colorImage)
                {
                LED(x+xoffset, (y+yoffset), For_color.red, For_color.green);                             
                }
              else
                {
                LED(x+xoffset, (y+yoffset), Bk_color.red, Bk_color.green);        
                }
          }
      }
  delay(delaytime);
  }  
}

 void HScrollBigImageL_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, uint16_t delaytime )
{
    Color For_color;
    for (uint16_t i= 0; i < width-32; i++)
    {
    for (uint16_t y = 0; y < height; y++)
      {
        for (uint16_t x = 0; x < 32; x++)
        {
            uint16_t   myindex = (i+x)/8 + y * (width / 8);
            uint16_t   mybitmask = 7-((i+x) % 8);
            uint16_t   colorImage = bitRead(pgm_read_byte(&image[myindex]), mybitmask) & 1;
              if (colorImage)
                {      

                  get_colour(colourPos + 2*(x+y), &For_color.red, &For_color.green);
                      
                  LED(x+xoffset, (y+yoffset), For_color.red, For_color.green);
                }                                                                       
              else
                {
                LED(x+xoffset, (y+yoffset), 0, 0);        
                }
          }
      }
  increment_colour_pos(1); 
  delay(delaytime);

  }
  
}

void drawImage(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, Color For_color, Color Bk_color) 
{
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*4;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[myindex]),mybitmask) & 1;
              if (colorImage)
                {
                LED(x+xoffset,y+yoffset,For_color.red, For_color.green);
                }
              else
                {
                LED(x+xoffset,y+yoffset,Bk_color.red, Bk_color.green);        
                }
          }
    }
}

void drawImage_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image)
{
    Color For_color;
    
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*4;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[myindex]),mybitmask) & 1;
              if (colorImage)
                {
                get_colour(colourPos + 2*(x + y), &For_color.red, &For_color.green);
                LED(x+xoffset,y+yoffset,For_color.red, For_color.green);
                }
              else
                {
                LED(x+xoffset,y+yoffset,0, 0);        
                }
          }
          
    }
    increment_colour_pos(1); 
}

void drawImage_inv_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image)
{
    Color For_color;
    
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*4;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[myindex]),mybitmask) & 1;
            get_colour(colourPos + 2*(x + y), &For_color.red, &For_color.green);
              if (colorImage)
                {
                LED(x+xoffset,y+yoffset, 0, 0);
                }
              else
                {                

                LED(x+xoffset, y+yoffset, For_color.red, For_color.green);     
                }
          }          
    }
}

void drawImage_colorwheelX(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image)
{
    Color For_color;
    
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*4;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[myindex]),mybitmask) & 1;
              if (colorImage)
                {
                get_colour(colourPos + 2*x, &For_color.red, &For_color.green);
                LED(x+xoffset,y+yoffset,For_color.red, For_color.green);
                }
              else
                {
                LED(x+xoffset,y+yoffset,0, 0);        
                }
          }           
    }
    increment_colour_pos(1);
}

void drawImage_colorwheelY(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image)
{
    Color For_color;
    
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*4;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[myindex]),mybitmask) & 1;
              if (colorImage)
                {
                get_colour(colourPos + 2*y, &For_color.red, &For_color.green);
                LED(x+xoffset,y+yoffset,For_color.red, For_color.green);
                }
              else
                {
                LED(x+xoffset,y+yoffset,0, 0);        
                }
          }
          
    }
    increment_colour_pos(1); 
}

void drawImage_inv_colorwheelY(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image)
{
    Color For_color;
    
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*4;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[myindex]),mybitmask) & 1;
              if (colorImage)
                {
                LED(x+xoffset,y+yoffset,0, 0);
                }
              else
                {               
                get_colour(colourPos + 2*y, &For_color.red, &For_color.green);
                LED(x+xoffset,y+yoffset,For_color.red, For_color.green);        
                }
          }
          
    }
    increment_colour_pos(1); 
}

void fillTable_colorwheel()
{ 
  uint8_t R, G, B;
    for (byte x=0; x<32; x++)
    {
      for (byte y=0; y<32; y++)
      {
        get_colour(colourPos + x + y, &R, &G);
        LED(x, y, R, G);      
      }
      increment_colour_pos(1);      
    }  
}

void fillTable_colorwheelXY()
{  
  uint8_t R, G, B;
  for (byte inter=0; inter<10; inter++)
  {
    for (byte x=0; x<32; x++)
    {
      for (byte y=0; y<32; y++)
      {
        get_colour(colourPos + x + y, &R, &G);
        LED(x, y, R, G);      
      }
      increment_colour_pos(1);
      delay(30);
    }
  
    for (byte y=0; y<32; y++)
    {
      for (byte x=0; x<32; x++)
      {  
        get_colour(colourPos - 2*y +x, &R, &G);
        LED(x, y, R, G);      
      }
      increment_colour_pos(1);
      delay(45);
    }
  delay(500);
  }
}

//*******************************************************MK4*****************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G)
{
  if (p >= (COLOUR_WHEEL_LENGTH-1))         
    p -= ((p-= (COLOUR_WHEEL_LENGTH-1)==0) ? COLOUR_WHEEL_LENGTH-3 : COLOUR_WHEEL_LENGTH-1);         

    
  *R = colourR[p];
  *G = colourG[p];
}

void get_next_colour(uint8_t *R, uint8_t *G)
{
  if (++ColPos >= (COLOUR_WHEEL_LENGTH))  
    ColPos -= (COLOUR_WHEEL_LENGTH);      

  *R = colourR[ColPos];
  *G = colourG[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= (COLOUR_WHEEL_LENGTH))   
  {
    colourPos -= (COLOUR_WHEEL_LENGTH);       
  }
}

/*--------------------------------------------------------------------------------------
 Draw or clear a line (horizontal and vertical line) from x1, y1 to x2, y2
--------------------------------------------------------------------------------------*/
void drawVLine(uint16_t x, uint16_t y1, uint16_t y2, Color color)
{
    for (uint16_t y = y1; y <= y2; y++) {
        LED(x, y, color.red, color.green);      
    }
}

void drawHLine(uint16_t x1, uint16_t x2, uint16_t y, Color color)
{
    for (uint16_t x = x1; x <= x2; x++) {
          LED(x,y, color.red, color.green);      
    }  
}

void drawFastVLine(int16_t x, int16_t y,int16_t h, Color color) 
{
  // Update in subclasses if desired!
  drawLine(x, y, x, y+h-1, color);
}

void drawFastHLine(int16_t x, int16_t y,int16_t w, Color color) 
{
  // Update in subclasses if desired!
  drawLine(x, y, x+w-1, y, color);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) 
      {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
      }
  
    if (x0 > x1) 
      {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
      }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) 
    {
      ystep = 1;
    } else 
    {
      ystep = -1;
    }

    for (; x0<=x1; x0++) 
    {
      if (steep) 
      {
      LED(y0, x0, color.red, color.green);
      } 
      else 
      {
      LED(x0, y0, color.red, color.green);
      }
      err -= dy;
      if (err < 0) 
      {
        y0 += ystep;
        err += dx;
      }
    }
}

void fillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, Color color)
{
    for (uint16_t x = x1; x <= x2; x++) {
        for (uint16_t y = y1; y <= y2; y++) {
            LED(x,y, color.red, color.green);      
        }
    }
}
void drawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, Color color)
{
    drawHLine(x1,x2,y1,color);
    drawHLine(x1,x2,y2,color);  
    drawVLine(x1,y1,y2,color);
    drawVLine(x2,y1,y2,color);
}

void drawLine_colorwheel(int16_t x0, int16_t y0, int16_t x1, int16_t y1) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) 
      {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
      }
  
    if (x0 > x1) 
      {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
      }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) 
    {
      ystep = 1;
    } else 
    {
      ystep = -1;
    }

    for (; x0<=x1; x0++) 
    {
      if (steep) 
      {
        get_colour(colourPos+ 3*x0, &R, &G);
        LED(y0, x0, R, G);
        
      } 
      else 
      {     
        get_colour(colourPos+63+(x0+y0), &R, &G);
        LED(x0, y0, R, G);
      }
      err -= dy;
      if (err < 0) 
      {
        y0 += ystep;
        err += dx;
      }
      //increment_colour_pos(1);
    }
}

/*--------------------------------------------------------------------------------------
 Draw or clear a circle of radius r at x,y centre
--------------------------------------------------------------------------------------*/
void drawCircle(int xCenter, int yCenter, int radius, Color color)
{   
  // Bresenham's circle drawing algorithm
  int x = -radius;
  int y = 0;
  int error = 2-2*radius;
  while(x < 0) {
    LED(xCenter-x, yCenter+y, color.red, color.green);
    LED(xCenter-y, yCenter-x, color.red, color.green);
    LED(xCenter+x, yCenter-y, color.red, color.green);
    LED(xCenter+y, yCenter+x, color.red, color.green);
    radius = error;
    if (radius <= y) error += ++y*2+1;
    if (radius > x || error > y) error += ++x*2+1;
  }
}
