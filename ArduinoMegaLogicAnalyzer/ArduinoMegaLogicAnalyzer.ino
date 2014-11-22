#include <NilRTOS.h>


#define CHPIN PINA
#define CHDDR DDRA
#define CH_AD 0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define T1_EN_CR  OCR1A
#define T1_EN_SFR  TIMSK1
#define T1_EN_BIT  OCIE1A
#define LOGIC_CAP_EN  sbi(T1_EN_SFR, T1_EN_BIT)
#define LOGIC_CAP_DIS  cbi(T1_EN_SFR, T1_EN_BIT)
#define SET_INTERVAL(us)  (T1_EN_CR = (us << 1) - 1)

#define LED_PIN 13


enum Prescalars
{
  PRESCAL_2 = 1,
  PRESCAL_4,
  PRESCAL_8,
  PRESCAL_16,
  PRESCAL_32,
  PRESCAL_64,
  PRESCAL_128,
};

enum Frequencies
{
  FREQ_8MHz = 1,
  FREQ_4MHz,
  FREQ_2MHz,
  FREQ_1MHz,
  FREQ_500kHz,
  FREQ_250kHz,
  FREQ_125kHz,
};

volatile uint8_t value = 0;
volatile bool rts = false;
volatile bool digitalMode = true;
volatile bool capOnChanged = true;

SEMAPHORE_DECL(semRTS, 0);


NIL_WORKING_AREA(waSendData, 64); 
NIL_THREAD(SendData,arg)
{
  while(true)
  {
    nilSemWait(&semRTS);
    
    cli();  // Disable interrupts
    Serial.write(value);
    sei();  // Enable interrupts
  }
}

NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY("SendData", SendData, NULL, waSendData, sizeof(waSendData))
NIL_THREADS_TABLE_END()

/*
16 MHz / 2 = 8 MHz
16 MHz / 4 = 4 MHz
16 MHz / 8 = 2 MHz
16 MHz / 16 = 1 MHz
16 MHz / 32 = 500 kHz
16 MHz / 64 = 250 kHz
16 MHz / 128 = 125 kHz
*/
void setPrescal(uint8_t scal)
{
  ADCSRA &= B11111000;  // Clear prescale values
  ADCSRA |= scal;
}

void setFreq(uint8_t freq)
{  
  switch((Frequencies)freq)
  {
    case FREQ_8MHz:
      setPrescal(PRESCAL_2);
    break;
    case FREQ_4MHz:
      setPrescal(PRESCAL_4);
    break;
    case FREQ_2MHz:
      setPrescal(PRESCAL_8);
    break;
    case FREQ_1MHz:
      setPrescal(PRESCAL_16);
    break;
    case FREQ_500kHz:
      setPrescal(PRESCAL_32);
    break;
    case FREQ_250kHz:
      setPrescal(PRESCAL_64);
    break;
    case FREQ_125kHz:
      setPrescal(PRESCAL_128);
    break;
  }
}

void setMode(bool digital)
{
  digitalMode = digital;
  if(digitalMode)
    SET_INTERVAL(100);
  else
    SET_INTERVAL(100);
}

void setup()
{
  // Disable interrupts
  cli();
  
  Serial.begin(115200);
  Serial2.begin(115200);
  
  // initial port as input
  CHDDR = 0;
  
  // initial Timer 1 for frequncy control
  TCCR1A = 0;  // reset register
  TCCR1B = 0;  // reset register
  TCNT1 = 0;   // reset counter
  
  TCCR1B |= (1 << WGM12);  // Enable CTC Mode
  TCCR1B |= (1 << CS11);   // Set 8 prescaler (1 count = 0.5 us for 16MHz, 1 us for 8MHz)
  
  // Initial analog sampling rate
  setFreq(FREQ_4MHz);
  
  // make it ready due to first read analog take 25 cycle and 13 cycle for later reading
  value = analogRead(CH_AD) >> 2;  
  
  // initial capture mode
  setMode(true);
  
  // led display
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  //LOGIC_CAP_EN;  // enable logic capture
  
  // Start Nil RTOS.
  nilSysBegin();
  
  // Enable interrupts
  sei();
  
  mainThread();
}

#define BUF_SIZE 32
char buf[BUF_SIZE];
int idx = 0;

void mainThread()
{
  while(true)
  {
    // check received command
    while(Serial.available())
    {
      buf[idx] = Serial.read();
      Serial2.write(buf[idx]);
      
      if(buf[idx] == '\n')
      {
        buf[idx + 1] = '\0';
        String cmd = String(buf);
        cmd.toLowerCase();
        cmd.replace("\n","");
        
        if(cmd == "start")
        {
          digitalWrite(LED_PIN, HIGH);
          
          LOGIC_CAP_EN;  // enable logic capture
        }
        else if(cmd == "stop")
        {
          LOGIC_CAP_DIS;  // disable logic capture
          digitalWrite(LED_PIN, LOW);
        }
        else
        {
          int last = cmd.indexOf("=");
          String cmd1 = cmd.substring(0, last);
          String cmd2 = cmd.substring(last + 1);
          if(cmd1 == "cfg")
          {
            last = cmd2.indexOf(",");
            cmd1 = cmd2.substring(0, last);
            setMode(cmd1 != "0");
            
            cmd1 = cmd2.substring(last + 1);
            capOnChanged = (cmd1 != "0");
          }
        }
        
        idx = 0;
      }
      else
      {
        idx++;
        if(idx >= sizeof(buf))
          idx = 0;
      }
    }
  }
}

void loop()
{   
  // do nothing
}

// Capture digital value and triger ready to send flag
ISR(TIMER1_COMPA_vect)
{
  if(digitalMode)
  {
    // logic capture
    value = CHPIN;
  }
  else
  {
    // analog capture for 8 bit resolution
    value = analogRead(CH_AD) >> 2;
  }
  
  // set ready to send flag
  nilSemSignal(&semRTS);
}

