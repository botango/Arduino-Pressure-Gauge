#include <Wire.h>

#define L_PIN         13
#define BTN_PIN       2

#define LCD_ADRS        0x3E
#define LCD_LOCATE(x,y) LCD_command(0x80 | (0x40 * y + x))
#define LCD_CLS()       LCD_command(0x01)

//#define LPS_ADRS B1011100  // SA0 = GND
#define LPS_ADRS B1011101  // SA0 = VDD_IO
#define LPS_WHOAMI        0x0f
#define LPS_CTRL_REG1     0x20
#define LPS_CTRL_REG2     0x21
#define LPS_CTRL_REG3     0x22
#define LPS_PRESS_OUT_XL  0x28
#define LPS_PRESS_OUT_L   0x29
#define LPS_PRESS_OUT_H   0x2A
#define LPS_TEMP_OUT_L    0x2B
#define LPS_TEMP_OUT_H    0x2C

#define kPa     // kPaで表示する場合
//#define hPa     // hPaで表示する場合

int Lchika_count;
int16_t offset;

void setup()
{
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(L_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Pressure Gauge");

  Wire.begin();
  Wire.setClock(100000);
  LCD_init();
  LCD_CLS();
  
  digitalWrite(L_PIN, HIGH);
  LCD_write("Pressure");
  LCD_LOCATE(0,1);
  LCD_write("Gauge");
  delay(1500);
  digitalWrite(L_PIN, LOW);
  
  byte whoami = LPS_read(LPS_WHOAMI);
  /*
  LPS_write(LPS_CTRL_REG2, 0x04);
  while(LPS_read(LPS_CTRL_REG2));
  delay(100);
  LPS_write(LPS_CTRL_REG2, 0x80);
  while(LPS_read(LPS_CTRL_REG2));
  delay(100);
  */
  LPS_write(LPS_CTRL_REG1, 0x90);
  delay(100);
  
  LCD_CLS();
  LCD_write("Sensor");
  Serial.print("Sensor:");
  LCD_LOCATE(0,1);
  if(whoami==0xBB)
  {
    LCD_write("LPS331AP");
    Serial.println("LPS331AP");
  }
  else if(whoami==0xBD)
  {
    LCD_write("LPS25HB");
    Serial.println("LPS25HB");
  }
  else
  {
    LCD_write("Unknown");
    Serial.println("Unknown");
  }
  delay(1500);
  
  LCD_CLS();
  offset = LPS_readPress()*10;
  LCD_write("Offset");
  LCD_LOCATE(0,1);
  lcd_print_Pa(offset);
  Serial.print("offset:");
  Serial.println(offset);
  delay(1500);
  
  LCD_CLS();
}

void loop()
{
  int16_t pressure = LPS_readPress()*10;
  int16_t diff = pressure-offset;
  
  Serial.print(pressure);
  Serial.print(" ");
  Serial.print(offset);
  Serial.print(" ");
  Serial.print(diff);
  Serial.println();
  
  LCD_LOCATE(0,0);
  lcd_print_Pa(pressure);
  LCD_LOCATE(0,1);
  lcd_print_Pa(diff);

  uint8_t i;
  for(i=0;i<99;i++)
  {
    if (digitalRead(BTN_PIN) == LOW)
    {
      delay(10);
      if (digitalRead(BTN_PIN) == LOW)
      {
        LCD_CLS();
        LCD_LOCATE(0,0);
        LCD_write("Offset");
        offset = LPS_readPress()*10;

        LCD_LOCATE(0,1);
        lcd_print_Pa(offset);

        while(digitalRead(BTN_PIN) == LOW);
        LCD_CLS();
      }      
    }

    delay(10);
  }
  
  digitalWrite(L_PIN, Lchika_count++&1);
}

void lcd_print_Pa(int16_t val)
{
  if(val<0) LCD_write('-');
  else      LCD_write(' ');
  val=abs(val);

  LCD_write('0'+val/10000);
  val%=10000;
  LCD_write('0'+val/1000);
  val%=1000;
  LCD_write('0'+val/100);
  val%=100;

#ifdef kPa
  LCD_write('.');
#endif

  LCD_write('0'+val/10);
  val%=10;

#ifdef hPa
  LCD_write('.');
#endif

  LCD_write('0'+val);

#ifdef kPa
  LCD_write('k');
#elif defined(hPa)
  LCD_write('h');
#else
  LCD_write('0');
  LCD_write('P');
#endif
}

void LCD_init()
{
// 3Vで動かす場合
#define LCD_CONTRAST          (0x23)
#define LCD_CONTRAST_H        (0x5c)

// 3.3Vで動かす場合
//#define LCD_CONTRAST          (0x1a)
//#define LCD_CONTRAST_H        (0x5c)

// 5Vで動かす場合
//#define LCD_CONTRAST        (0x23)
//#define LCD_CONTRAST_H      (0x58)

  delay(50);
  LCD_command(0b00111000);  // ファンクション・セット
  LCD_command(0b00111001);  // ファンクション・セット
  LCD_command(0x14);      // カーソル移動・表示シフト
  
  // Contrast set
  LCD_command(0x70 | (LCD_CONTRAST & 0xF));
  LCD_command(LCD_CONTRAST_H | ((LCD_CONTRAST >> 4) & 0x3));

  LCD_command(0b01101100);  // follower control
  delay(300);

  LCD_command(0b00111000);  // ファンクション・セット
  LCD_command(0b00001100);  // ディスプレイＯＮ

  LCD_command(0x01);      // LCD clear
  delay(2);
}

void LCD_command(uint8_t data)
{
  Wire.beginTransmission(LCD_ADRS);
  Wire.write(0x00);
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}

void LCD_write(const char data)
{
  Wire.beginTransmission(LCD_ADRS);
  Wire.write(0x40);
  Wire.write(data);
  Wire.endTransmission();
  delay(1);
}

void LCD_write(const char *t_data)
{
  Wire.beginTransmission(LCD_ADRS);
  Wire.write(0x40);
  while (*t_data != NULL)  Wire.write(*t_data++);
  Wire.endTransmission();
  delay(1);
}

void LPS_write(byte reg, byte val)
{
  Wire.beginTransmission(LPS_ADRS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();  
}

byte LPS_read(byte reg)
{
  byte ret = 0;
  // request the registor
  Wire.beginTransmission(LPS_ADRS);
  Wire.write(reg);
  Wire.endTransmission();  

  // read
  Wire.requestFrom((unsigned int)LPS_ADRS, 1);
  
  while (Wire.available())
  {
    ret = Wire.read();
  }
  
  return ret;
}
/*
float LPS_readTemp()
{
  short T;
  float t;

  T = LPS_read(LPS_TEMP_OUT_H);
  T = (T << 8) | LPS_read(LPS_TEMP_OUT_L);
  
  t = T;
  t = 42.5 + t/480.0;

  return t;
}
*/
float LPS_readPress()
{
  long P;
  float p;

  P = LPS_read(LPS_PRESS_OUT_H);
  P = (P << 8) | LPS_read(LPS_PRESS_OUT_L);
  P = (P << 8) | LPS_read(LPS_PRESS_OUT_XL);
  
  p = P;
  p = p/4096.0;

  return p;
}
