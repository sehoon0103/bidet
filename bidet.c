#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

typedef enum { 
  STATE_IDLE,
  STATE_NOZZLE_FORWARD,
  STATE_PUMP_ON,
  STATE_NOZZLE_BACK,
  STATE_PUMP_WEAK,
  STATE_DRY,
} BidetState; 

volatile BidetState current_state = STATE_IDLE; 
volatile uint32_t state_start_time = 0;
void enter_state(BidetState s);
void update_state(void);

volatile uint8_t blink_inc_active = 0;
volatile uint8_t blink_dec_active = 0;
uint32_t blink_inc_start = 0;
uint32_t blink_dec_start = 0;
static inline void start_inc_blink(void) {
    blink_inc_active   = 1;
    blink_inc_start    = millis();
    PORTD |=  (1<<PD7);    // 증가 LED ON
}
static inline void start_dec_blink(void) {
    blink_dec_active   = 1;
    blink_dec_start    = millis();
    PORTB |=  (1<<PB0);    // 감소 LED ON
}
void blink_update(void) {
    uint32_t now = millis();
    // 증가 LED(PD7) 깜박임 처리 (예: 100ms 후 꺼짐)
    if (blink_inc_active && (now - blink_inc_start >= 100)) {
        PORTD &= ~(1<<PD7);
        blink_inc_active = 0;
    }
    // 감소 LED(PB0) 깜박임 처리 (예: 100ms 후 꺼짐)
    if (blink_dec_active && (now - blink_dec_start >= 100)) {
        PORTB &= ~(1<<PB0);
        blink_dec_active = 0;
    }
}

// 노즐 이동 시간 테이블 (ms)
const uint16_t nozzle_fwd_time[3] = {1000, 1500, 2000};
const uint16_t nozzle_back_time[3] = {1000, 1500, 2000};


// -------------------------------------------------------------------------------------------

// 타이머 함수들 (millis 사용) 

// 타이머 관련 전역 변수
volatile uint32_t timer0_overflow_count = 0;     // 타이머 오버플로우 횟수
volatile uint32_t timer0_millis = 0;             // 밀리초 카운터
static uint8_t timer0_fract = 0;                 // 분수 밀리초 (정밀도 향상용)

// 상수 정의 (16MHz, 64 분주비 기준)
// 타이머0 오버플로우 시간 = 256 틱 * (64 / 16000000) = 1.024ms
#define MICROSECONDS_PER_TIMER0_OVERFLOW 1024UL  // 1.024ms = 1024μs per overflow
#define MILLIS_INC 1
#define FRACT_INC 3       // 1024 % 1000 = 24 -> 24 >> 3 = 3
#define FRACT_MAX 125     // 1000 >> 3 = 125                                // 분수 밀리초 최대값 (125)

// Timer0 오버플로우 인터럽트 서비스 루틴
ISR(TIMER0_OVF_vect) {
	// 현재 밀리초와 분수 밀리초 값 가져오기
	uint32_t m = timer0_millis;
	uint8_t f = timer0_fract;
	// 오버플로우당 증가량 더하기
	m += MILLIS_INC;
	f += FRACT_INC;
	
	// 분수 밀리초가 최대값을 초과하면 밀리초 1 추가
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}
	// 값 업데이트
	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

// Timer0 초기화 함수
void timer0_init(void) {
	// 인터럽트 비활성화
	cli();
	
	// 타이머/카운터 0 레지스터 초기화
	TCCR0A = 0;  // 일반 모드
	
	// 분주비 64 설정 (CS01=1, CS00=1)
	// 16MHz에서 분주비 64일 때 타이머 클럭은 250kHz (4μs 간격)
	TCCR0B = (1 << CS01) | (1 << CS00);
	
	// 타이머 오버플로우 인터럽트 활성화
	TIMSK0 |= (1 << TOIE0);
	
	// 카운터와 플래그 초기화
	timer0_overflow_count = 0;
	timer0_millis = 0;
	timer0_fract = 0;
	
	// 인터럽트 활성화
	sei();
}


// 밀리초 반환 함수
uint32_t millis(void) {
	uint32_t m;
	
	// 원자적 블록 내에서 값 읽기 (인터럽트 영향 없이)
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m = timer0_millis;
	}
	
	return m;
}

// 마이크로초 반환 함수
uint32_t micros(void) {
	uint32_t m;
	uint8_t t;
	
	// 원자적 블록 내에서 오버플로우 카운트와 현재 타이머 값 읽기
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m = timer0_overflow_count;
		t = TCNT0;
		
		// 오버플로우 플래그가 설정되었지만 아직 인터럽트가 처리되지 않았는지 확인
		// 이 경우 수동으로 오버플로우 카운트 증가 필요
		if ((TIFR0 & (1 << TOV0)) && (t < 255)) {
			m++;
		}
	}
	
	// 총 마이크로초 계산
	// (오버플로우 횟수 * 256 + 현재 타이머값) * 타이머 틱당 μs
	return ((m << 8) + t) * (64 / (F_CPU / 1000000UL));
}

// 딜레이 함수 (밀리초)
void delay(uint32_t ms) {
	uint32_t start = millis();
	
	while ((millis() - start) < ms) {
		// 대기
	}
}

// -------------------------------------------------------------------------------------------

// Tone(비프음) 함수들

volatile uint8_t isToneActive = 0;

void tone(unsigned long frequency) {
    if (frequency == 0 || frequency > 10000) {
        tone_stop();
        return;
    }
    DDRB |= (1 << PB1);
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);
    TCCR1B |= (1 << CS11);
    uint16_t ocr_val = (2000000 / (2 * frequency)) - 1;
    OCR1A = ocr_val;
    TCCR1A |= (1 << COM1A0);
    isToneActive = 1;
}

void tone_stop() {
    TCCR1A = 0;
    TCCR1B = 0;
    isToneActive = 0;
}

volatile uint8_t isToneOn = 0;
uint32_t tone_start_time = 0;

void tone_on() {
    tone(440);
    isToneOn = 1;
    tone_start_time = millis();
}

void tone_update() {
    if (isToneOn && (millis() - tone_start_time >= 100)) {
        tone_stop();
        isToneOn = 0;
    }
}

// -------------------------------------------------------------------------------------------

// uart 함수

void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud for 16MHz
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void uart_print_key(const char* keyname, const char* label) {
    uart_transmit('[');
    uart_print(keyname);
    uart_transmit(']');
    uart_transmit(' ');
    uart_print(label);
    uart_transmit('\n');
}

// -------------------------------------------------------------------------------------------

// led 매트릭스, 키패드 매트릭스 함수

void led_matrix_enter() {
    DDRB |= 0b1111<<2;
    DDRC |= 0b1111<<0;
}

uint8_t led_idx=0;
uint8_t led_pattern[4]={
    0b0111,
    0b1111,
    0b0111,
    0b1111
};

void led_matrix_loop() {
    PORTB &= ~(0b1111<<2);
    PORTC |= 0b1111;
	PORTC = (PORTC & 0b11110000) | (led_pattern[led_idx] & 0x0F); // 기존 0b11100000이었는데 먼가 시트 깜빡임 잘 안돼서 수정

    PORTB |= 1<<(5-led_idx);
    if(++led_idx>3) led_idx = 0;
}

void led_matrix_exit() {
    DDRB &= ~(0b1111<<2);
    DDRC &= ~(0b1111<<0);
}

void keypad_matrix_enter() {
    DDRB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    PORTB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    DDRD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0));
    PORTD |=  (1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0);
}

void keypad_matrix_exit() {
    DDRB &= ~((1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2));
    DDRD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0));
    PORTB &= ~((1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2));
    PORTD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0));
}

const char key_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

uint8_t prev_key[4] = {0, 0, 0, 0};

void led_set(uint8_t row, uint8_t col, uint8_t on) {
    if (row > 3 || col > 3) return;
    if (on) led_pattern[row] &= ~(1 << (3 - col));
    else    led_pattern[row] |=  (1 << (3 - col));
}

uint8_t led_get(uint8_t row, uint8_t col) {
    if (row > 3 || col > 3) return 0;
    return !(led_pattern[row] & (1 << (3 - col)));
}

// -------------------------------------------------------------------------------------------

// 상태변화 관련

uint8_t water_pressure = 1;
uint8_t seat_temperature = 0;
uint8_t nozzle_phase = 1;
uint8_t seat_sensor = 0;

volatile uint8_t sequence_active = 0;  // 비데/세정 시퀀스 실행 중 플래그


volatile uint8_t stop_dry_flag = 0;
volatile uint8_t dry_mode_active = 0;
uint32_t dry_mode_start_time = 0;

volatile uint8_t stop_requested = 0;

void stop() {
   stop_requested = 1;
   tone_on();  // 비프음 먼저

  if (dry_mode_active) {
    PORTC &= ~(1 << PC5);
    led_set(0, 3, 0);
    dry_mode_active = 0;
  }

  if (sequence_active) {
    sequence_active = 0;

    // 현재 상태에 따라 전이 판단
    if (current_state == STATE_NOZZLE_FORWARD || current_state == STATE_PUMP_ON) {
      enter_state(STATE_NOZZLE_BACK);  
    } else {
      enter_state(STATE_IDLE);  // 나머지는 그냥 바로 종료
    }
  }
}
           


// —— 펌프 초기화 (OC0A=D6) —— 
void pump_init(void) {
    DDRD  |= (1<<PD5);                          // D6 출력
    TCCR0A = (1<<COM0B1)|(1<<WGM01)|(1<<WGM00); // Fast PWM, 비반전
    TCCR0B = (1<<CS01)|(1<<CS00);               // 분주비 64
}

// —— 수압 단계별 PWM 설정 —— 
void pump_set(uint8_t level) {
    static const uint8_t tbl[4] = {0, 64, 128, 255};
    OCR0B = (level < 4) ? tbl[level] : 0;
}
// —— 노즐 설정 —— 
static inline void nozzle_led_forward(void) {
    PORTB |=  (1<<PB0);
}
static inline void nozzle_led_backward(void) {
    PORTD |=  (1<<PD7);
}
static inline void nozzle_led_off(void) {
    if (!blink_inc_active) PORTD &= ~(1<<PD7);
    if (!blink_dec_active) PORTB &= ~(1<<PB0);
}
// 노즐설정 
const uint16_t nozzle_fwd_time_bidet[3]  = {1000, 1500, 2000};
const uint16_t nozzle_back_time_bidet[3] = {1000, 1500, 2000};

const uint16_t nozzle_fwd_time_wash[3]   = {2000, 2500, 3000};
const uint16_t nozzle_back_time_wash[3]  = {2000, 2500, 3000};

const uint16_t *current_nozzle_fwd_time = nozzle_fwd_time_bidet;
const uint16_t *current_nozzle_back_time = nozzle_back_time_bidet;


void update_dry_mode() {
    if (dry_mode_active && (millis() - dry_mode_start_time >= 3000)) {
        uart_print("dry complete\n");
        PORTC &= ~(1 << PC5);
        led_set(0, 3, 0); 
        dry_mode_active = 0;
    }
}

void seat_heater_blink_update() {
    static uint8_t state = 0; // 0: off, 1: on
    static uint32_t last_time = 0;

    uint32_t now = millis();

    if (seat_temperature == 0) {
        PORTC &= ~(1 << PC4);
        state = 0;
        return;
    }

    if (state == 0 && (now - last_time >= seat_temperature * 1000)) {
        PORTC |= (1 << PC4);
        state = 1;
        last_time = now;
    }
    else if (state == 1 && (now - last_time >= 1000)) {
        PORTC &= ~(1 << PC4);
        state = 0;
        last_time = now;
    }
}

void enter_state(BidetState s) { 
    current_state = s;
    state_start_time = millis();
    // 초기화 작업
    switch(s) {
      case STATE_IDLE:
        nozzle_led_off();
        pump_set(0); /*펌프 끄기*/
        PORTC       &= ~(1 << PC5);  // 팬 OFF
        led_set(0, 3, 0);            // 건조 LED OFF
        // motor_stop(); fan_off();
        uart_print("STATE: IDLE\n");
        break;
      case STATE_NOZZLE_FORWARD:
        nozzle_led_forward();
        // motor_forward();
        uart_print("STATE: NOZZLE_FORWARD\n");
        break;
      case STATE_PUMP_ON:
        nozzle_led_off();
        pump_set(water_pressure);
        uart_print("STATE: PUMP_ON\n");
        break;
      case STATE_NOZZLE_BACK:
        nozzle_led_backward();
        // motor_backward();
        uart_print("STATE: NOZZLE_BACK\n");
        break;
      case STATE_PUMP_WEAK:
        nozzle_led_off();
        pump_set(1);
        uart_print("STATE: PUMP_WEAK\n");
        break;
      case STATE_DRY:
        nozzle_led_off();
        pump_set(0);
        DDRC       |=  (1 << PC5);   // 팬 ON
        PORTC       |=  (1 << PC5);
        led_set(0, 3, 1);           
        // fan_on();
        uart_print("STATE: DRY\n");
        break;
    }
}

void update_state(void) { 
    uint32_t e = millis() - state_start_time;
    switch(current_state) {
      case STATE_NOZZLE_FORWARD:
        if (e >= current_nozzle_fwd_time[nozzle_phase-1]) enter_state(STATE_PUMP_ON);
        break;
      case STATE_PUMP_ON:
        if (e >= 5000) {
        enter_state(STATE_NOZZLE_BACK);
      }
    break;
      case STATE_NOZZLE_BACK:
        if (e >= current_nozzle_back_time[nozzle_phase - 1]) {
        if (stop_requested) {
            enter_state(STATE_IDLE);
            stop_requested = 0; // IDLE에서만 리셋
        } else {
            enter_state(STATE_PUMP_WEAK);
        }
    }
        break;
      case STATE_PUMP_WEAK:
        if (stop_requested) {
        enter_state(STATE_IDLE);
        stop_requested = 0;
    } else if (e >= 1000) {
        enter_state(STATE_DRY);
    }
        break;
      case STATE_DRY:
        if (stop_requested || e >= 3000) {
        enter_state(STATE_IDLE);
        stop_requested = 0;
    }
        break;
      default: break;
    }
}


// -------------------------------------------------------------------------------------------

// 키패드 매트릭스 입력받으면 동작하는 부분

void keypad_matrix_input(void) {
    for (uint8_t row = 0; row < 4; row++) {
        PORTB |= (0b1111 << 2);
        PORTB &= ~(1 << (5 - row));
        _delay_us(2);

        uint8_t key = 0;
        if (!(PIND & (1 << 4))) key |= (1 << 0);
        if (!(PIND & (1 << 3))) key |= (1 << 1);
        if (!(PIND & (1 << 2))) key |= (1 << 2);
        if (!(PIND & (1 << 0))) key |= (1 << 3);

        uint8_t rising = (key ^ prev_key[row]) & key;
        prev_key[row] = key;

        for (uint8_t col = 0; col < 4; col++) {
            if (rising & (1 << col)) {
                char key_char = key_map[row][col];
              
                if (key_char == '4') { // 노즐위치 후진
                    if (seat_sensor 
                        && sequence_active 
                        && (current_state == STATE_PUMP_ON || current_state == STATE_PUMP_WEAK)) {
                        tone_on();
                        PORTB |=  (1<<PB0);
                        PORTD &= ~(1<<PD7);
                    if (led_get(2, 2)) {
                        uart_print_key("4", "nozzle phase - (2)");
                        led_set(2, 2, 0);
                        led_set(2, 1, 1);
                        nozzle_phase = 2;
                        start_dec_blink();
                    } else if (led_get(2, 1)) {
                        uart_print_key("4", "nozzle phase - (1)");
                        led_set(2, 1, 0);
                        led_set(2, 0, 1);
                        nozzle_phase = 1;
                        start_dec_blink();
                    } else {
                        uart_print_key("4", "nozzle phase - (1)");
                    }
                      		
                  }
                } else if (key_char == '5') { // 노즐위치 전진
                    if (seat_sensor 
                        && sequence_active 
                        && (current_state == STATE_PUMP_ON || current_state == STATE_PUMP_WEAK)) {
                        tone_on();
                        PORTB |=  (1<<PB7);
                        PORTD &= ~(1<<PD0);                      
                    if (led_get(2, 1)) {
                        uart_print_key("5", "nozzle phase + (3)");
                        led_set(2, 1, 0);
                        led_set(2, 2, 1);
                        nozzle_phase = 3;
                        start_inc_blink(); 
                    } else if (led_get(2, 0)) {
                        uart_print_key("5", "nozzle phase + (2)");
                        led_set(2, 0, 0);
                        led_set(2, 1, 1);
                        nozzle_phase = 2;
                        start_inc_blink();
                    } else {
                        uart_print_key("5", "nozzle phase + (3)");
                    }
                  }
                } else if (key_char == '7') { // 수압감소
                  if (!dry_mode_active && seat_sensor) {
                    tone_on();
                    if (led_get(0, 0) && led_get(0, 1) && led_get(0, 2)) {
                        uart_print_key("7", "water pressure - (2)");
                        led_set(0, 2, 0);
                        water_pressure = 2;
                    } else if (led_get(0, 0) && led_get(0, 1)) {
                        uart_print_key("7", "water pressure - (1)");
                        led_set(0, 1, 0);
                        water_pressure = 1;
                    } else {
                        uart_print_key("7", "water pressure - (1)");
                    }
                  }
                } else if (key_char == '8') { // 수압증가
                  if (!dry_mode_active && seat_sensor) {
                    tone_on();
                    if (led_get(0, 0) && led_get(0, 1)) {
                        uart_print_key("8", "water pressure + (3)");
                        led_set(0, 2, 1);
                        water_pressure = 3;
                    } else if (led_get(0, 0)) {
                        uart_print_key("8", "water pressure + (2)");
                        led_set(0, 1, 1);
                        water_pressure = 2;
                    } else {
                        uart_print_key("8", "water pressure + (3)");
                    }
                  }
                } else if (key_char == 'B') { //시트 온도
                  if (!dry_mode_active) {               
                    tone_on();
                    if (led_get(1, 0) && led_get(1, 1) && led_get(1, 2)) {
                        uart_print_key("B", "change seat temperature (off)");
                        led_set(1, 0, 0);
                        led_set(1, 1, 0);
                        led_set(1, 2, 0);
                        seat_temperature = 0;
                    } else if (led_get(1, 0) && led_get(1, 1)) {
                        uart_print_key("B", "change seat temperature (on : 3)");
                        led_set(1, 2, 1);
                        seat_temperature = 3;
                    } else if (led_get(1, 0)) {
                        uart_print_key("B", "change seat temperature (on : 2)");
                        led_set(1, 1, 1);
                        seat_temperature = 2;
                    } else {
                        uart_print_key("B", "change seat temperature (on : 1)");
                        led_set(1, 0, 1);
                        seat_temperature = 1;
                    }
                  }
                } else if (key_char == 'A') { //건조
                  if (!dry_mode_active && seat_sensor) {
                    uart_print_key("A", "dry");
                    tone_on();
                    DDRC |= (1 << PC5);
                    PORTC |= (1 << PC5);
                    led_set(0, 3, 1);
                    dry_mode_active = 1;
                    dry_mode_start_time = millis(); 
                  }
                } else if (key_char == '1') { //정지
                  if (seat_sensor) {
                    uart_print_key("1", "STOP");
                    tone_on();
                    stop(); 
                  }
                } else if (key_char == '2')  { //세정
                  if (seat_sensor && current_state==STATE_IDLE) {
                    tone_on();
                    uart_print_key("2", "wash start");
                    sequence_active = 1;
                    current_nozzle_fwd_time  = nozzle_fwd_time_wash;
                    current_nozzle_back_time = nozzle_back_time_wash;
                    enter_state(STATE_NOZZLE_FORWARD);
                  }  
                } else if (key_char == '3') { //비데
                  if (seat_sensor && current_state==STATE_IDLE) {
                    tone_on();
                    uart_print_key("3", "bidet start");
                    sequence_active = 1;
                    current_nozzle_fwd_time  = nozzle_fwd_time_bidet;
                    current_nozzle_back_time = nozzle_back_time_bidet;
                    enter_state(STATE_NOZZLE_FORWARD);
                  }  
            }

              
  else if (key_char == '*') {		// 디버깅용 * 눌렀을 때 수압, 시트온도, 노즐위치, 시트센터상태 표시
  tone_on();						// 디버깅용
  debug_status();  // 상태 출력		// 디버깅용
  }									// 디버깅용
              
              else {
                if (!dry_mode_active && seat_sensor) {
                    uart_transmit('[');
                    uart_transmit(key_char);
                    uart_transmit(']');
                    uart_transmit('\n');
                }

                }
            }
        }
        PORTB |= (0b1111 << 2);
    }
}

// -------------------------------------------------------------------------------------------

// 슬라이드 스위치(시트 센서)

uint8_t slide_switch_flag = 0;

void slide_switch_init() {
    DDRD &= ~(1 << PD6);
    PORTD |= (1 << PD6);
}

uint8_t slide_switch_state() {
    return !(PIND & (1 << PD6));
}

uint8_t prev_slide_state = 0;

void handle_slide_switch() {
    uint8_t curr_state = slide_switch_state();
    slide_switch_flag = curr_state;
    if (curr_state && !prev_slide_state) {
        tone_on();
        uart_print("[SW] seat sensor on\n");
        seat_sensor = 1;
    } else if (!curr_state && prev_slide_state) {
        tone_on();
        uart_print("[SW] seat sensor off\n");
        stop();
        seat_sensor = 0;
    }
    prev_slide_state = curr_state;
}

// -------------------------------------------------------------------------------------------

// 디버깅용 함수들
void debug_status() {	// 디버깅용
    char buffer[32];

    uart_print("=== Status ===\n");

    // 수압 상태 출력
    snprintf(buffer, sizeof(buffer), "Water Pressure: %d\n", water_pressure);
    uart_print(buffer);

    // 시트 온도 상태 출력
    snprintf(buffer, sizeof(buffer), "Seat Temp: %d\n", seat_temperature);
    uart_print(buffer);
  
    // 노즐 위치 상태 출력
    snprintf(buffer, sizeof(buffer), "Nozzle Phase: %d\n", nozzle_phase);
    uart_print(buffer);
      
    // 시트 센서 상태 출력
    snprintf(buffer, sizeof(buffer), "Seat Sensor: %d\n", seat_sensor);
    uart_print(buffer);

    uart_print("===============\n");
}						// 디버깅용

// -------------------------------------------------------------------------------------------


int main() {
    uart_init();
    slide_switch_init();
    timer0_init();
    pump_init(); 
    DDRD  |= (1<<PD7);  
    PORTD &= ~(1<<PD7);
    DDRB  |= (1<<PB0);  
    PORTB &= ~(1<<PB0);

 
    prev_slide_state = slide_switch_state();
    slide_switch_flag = prev_slide_state;
    seat_sensor = prev_slide_state;
  
    while(1) {
        handle_slide_switch();
        update_state(); 
        update_dry_mode();
        tone_update();
        _delay_ms(1);
        led_matrix_enter();
        led_matrix_loop();
        _delay_ms(1);
        led_matrix_exit();
        keypad_matrix_enter();
        keypad_matrix_input();
        keypad_matrix_exit();
        blink_update(); 
        seat_heater_blink_update();
        
    }
}  
