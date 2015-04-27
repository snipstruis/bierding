#include <OneWire.h>
#define DEBUG if(0)

#define PIN_THERMOMETER   7
#define PIN_MOTOR_BLUE   12
#define PIN_MOTOR_PINK   11
#define PIN_MOTOR_YELLOW 10
#define PIN_MOTOR_ORANGE  9
#define PIN_LED          13
// PIN_INTERRUPT is defined by INT0_vec
#define PIN_INTERRUPT     2

// RESISTORS between:
// VCC [2.70K] PIN_THERMOMETER 
// GND [dunno] PIN_INTERRUPT

#define CYCLES_PER_SECOND 16000000

// MEASURED DIFFERENCE: 0.49910 Hz ~ 0.49920 Hz
// 0.49910 * 
// (16000000*256)/ 62500
// 0.4

OneWire ds(PIN_THERMOMETER);

int16_t measure_temperature(void);
void motor_cw(int);
void motor_ccw(int);
void emergency_stop(void) __attribute__((noreturn));
void timer_interrupt(uint16_t);
void initialize_timer(void);

uint32_t volatile timer_seconds = 0;
int8_t   volatile table_index   = 0;
int8_t            table_size    = 0;
int16_t           table_temp[64]={0}; 
uint32_t          table_time[64]={0}; // seconds to wait before next interrupt

bool burner_state;
bool timer_active;

void get_user_input(void){
    char c;
input_retry:
  Serial.println("==== please input parameters ====");
  table_size=0;
input_loop:
  Serial.print("Temperature #");
  Serial.print(table_size);
  Serial.print(" (celcius): ");
  
  while(Serial.available()==0);
  table_temp[table_size] = int16_t(Serial.parseFloat());
  while(Serial.read()!=-1);
  
  Serial.println(float(table_temp[table_size]));
    
  Serial.print("Time        #");
  Serial.print(table_size);
  Serial.print(" (seconds): ");
  
  while(Serial.available()==0);
  table_time[table_size] = Serial.parseInt();
  while(Serial.read()!=-1);
  
  Serial.println(table_time[table_size]);
input_ask_again:
  Serial.print("more?(Yes/No/Undo/Restart)");

  while(Serial.available()==0);
  c = Serial.peek()&char(0xDF);
  while(Serial.read()!=-1);
  if(c!=-1) Serial.println(c); else Serial.println();
  
       if(c=='R') goto input_retry;
  else if(c=='N') {table_size++; goto input_continue;}
  else if(c=='U') goto input_loop;
  else if(c=='Y') {table_size++; goto input_loop;}
  else goto input_ask_again;
  
input_continue:
  Serial.println("received input:");
  int i;
  for(i=0; i<table_size; i++){
    Serial.print(float(table_temp[i]));
    Serial.print("\t");
    Serial.println(table_time[i]);
  }
  Serial.println("input understood");
  Serial.flush();
  timer_active = true;
}

void setup(void){
  pinMode(PIN_MOTOR_BLUE, OUTPUT);
  pinMode(PIN_MOTOR_PINK, OUTPUT);
  pinMode(PIN_MOTOR_YELLOW, OUTPUT);
  pinMode(PIN_MOTOR_ORANGE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  attachInterrupt (0, emergency_stop, RISING);
  // measured 0.49910 ~ 0.49920
  initialize_timer(62500-90); // 62500=16000000/256  90~=106.25=(2*0.49915-1)*(16000000/256)
  Serial.begin(9600);
  burner_state = false;
  timer_active = false; // to make sure the time interupt doesn't interfere with the input
  get_user_input();
  timer_seconds = 0;
}

void loop(void){
  if(table_size<0){
    Serial.println("done!");
    Serial.flush();
    if(burner_state == true) motor_ccw(45);
    asm volatile("jmp 0"); // soft-reset
  }else{
    // TODO correction software here
    Serial.println(table_size);
    Serial.println(table_index);
   
    int16_t temp = measure_temperature();
    Serial.println(float(temp)/16.0);
    if(float(temp)/16.0 < table_temp[table_index] && burner_state == false){
      motor_cw(45);
      burner_state = true;
      Serial.println(table_temp[table_index]);
    }
    if(float(temp)/16.0 > table_temp[table_index] && burner_state == true){
      motor_ccw(45);
      burner_state = false;
    }
    delay(1000);
    
    //asm volatile("nop"); // stop optimizing my buzy-waiting loops away! >.<
  }
}

void initialize_timer(uint16_t ticks){
  noInterrupts(); // disable interrupts while setting up timer
  // clear settings registers
  TCCR1A = 0;
  TCCR1B = 0;
  //initialize counter value to 0
  TCNT1  = 0;
  // set compare match register for 1hz increments
  OCR1A = ticks-1;//62500; // = (16000000/256) -1
  // turn on CTC mode (tell it to use compare match register)
  TCCR1B |= (1 << WGM12);
  // Set CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  interrupts(); // enable interrupts again
}


// timer-interrupt, called once per second
ISR(TIMER1_COMPA_vect){
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  
  if(timer_active){
    // correct for drift
    if(timer_seconds==table_time[table_index]){
      table_index++;
      timer_seconds=0;
      if(table_index>=table_size){
        table_size = -1;
      }
    }else{
      timer_seconds++;
    }
  }
  /*
  DEBUG Serial.print(timer_seconds/3600);
  DEBUG Serial.print(":");
  DEBUG Serial.print((timer_seconds/60)%60);
  DEBUG Serial.print(":");
  DEBUG Serial.println(timer_seconds%60);
  DEBUG Serial.flush();
  */
}

// when everything goes wrong, turn down the gas and soft-reset.
void emergency_stop(void){
   digitalWrite(PIN_LED, HIGH);
   int i;
   volatile uint16_t j; // volatile: don't optimize my wait loop away!
   
   if(burner_state == true) motor_ccw(45);
   
   burner_state = false;
   Serial.println("\nperformed emergency stop\n");
   Serial.flush();
   asm volatile("jmp 0"); // soft-reset
}


 // TODO: input from serial
  //int16_t raw = measure_temperature();
  //Serial.print("Temperature: ");
  //Serial.print(raw);
  //Serial.print(" (");
  //Serial.print((float)raw / 16.0);
  //Serial.println("*C)");
  
void motor_cw(int steps){
  volatile uint16_t j; // volatile: don't optimize my wait loop away!
  if(steps < 0.0) steps *= -1;
  int i;
  for(i = 0; i < steps; i++){
    digitalWrite(PIN_MOTOR_BLUE,   HIGH);
    digitalWrite(PIN_MOTOR_PINK,   HIGH);
    digitalWrite(PIN_MOTOR_YELLOW, LOW);
    digitalWrite(PIN_MOTOR_ORANGE, LOW);
    for(j=100000;--j!=0;); //delay does not work in interrupts
    digitalWrite(PIN_MOTOR_BLUE,   LOW);
    digitalWrite(PIN_MOTOR_PINK,   LOW);
    digitalWrite(PIN_MOTOR_YELLOW, HIGH);
    digitalWrite(PIN_MOTOR_ORANGE, HIGH);
    for(j=100000;--j!=0;); //delay does not work in interrupts
  }
}

void motor_ccw(int steps){
  volatile uint16_t j; // volatile: don't optimize my wait loop away!
  if(steps < 0.0) steps *= -1;
  int i;
  for(i = 0; i < steps; i++){
    digitalWrite(PIN_MOTOR_BLUE,   HIGH);
    digitalWrite(PIN_MOTOR_PINK,   LOW);
    digitalWrite(PIN_MOTOR_YELLOW, LOW);
    digitalWrite(PIN_MOTOR_ORANGE, HIGH);
    for(j=100000;--j!=0;); //delay does not work in interrupts
    digitalWrite(PIN_MOTOR_BLUE,   LOW);
    digitalWrite(PIN_MOTOR_PINK,   HIGH);
    digitalWrite(PIN_MOTOR_YELLOW, HIGH);
    digitalWrite(PIN_MOTOR_ORANGE, LOW);
    for(j=100000;--j!=0;); //delay does not work in interrupts
  }
}

int16_t measure_temperature(void) {
  byte i;
  byte type_s;
  byte data[12];
  byte addr[8];
try_again:
  byte present = 0;
  if ( !ds.search(addr)) {
    ds.reset_search();
    goto try_again;
  }
  
  DEBUG Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    DEBUG Serial.write(' ');
    DEBUG Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    DEBUG Serial.print("CRC is not valid!");
      goto try_again;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      DEBUG Serial.print("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      DEBUG Serial.print("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      DEBUG Serial.print("  Chip = DS1822");
      type_s = 0;
      break;
    default:
//      Serial.println("Device is not a DS18x20 family device.");
      goto try_again;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end
  
  // maybe delay one second here?
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  DEBUG Serial.print("  Data = ");
  DEBUG Serial.print(present, HEX);
  DEBUG Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    DEBUG Serial.print(data[i], HEX);
    DEBUG Serial.print(" ");
  }
  DEBUG Serial.print(" CRC=");
  DEBUG Serial.print(OneWire::crc8(data, 8), HEX);


  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return raw;
}

