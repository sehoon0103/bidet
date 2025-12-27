#define F_CPU 16000000UL           // CPU 동작 주파수(16MHz) - 시간 계산에 쓰임
#include <avr/io.h>                 // PORTB, DDRC 같은 레지스터 이름을 쓰기 위한 헤더
#include <util/delay.h>             // _delay_ms(), _delay_us() 같은 짧은 대기 함수
#include <stdio.h>                  // snprintf_P 등 문자열 포맷 함수
#include <avr/interrupt.h>          // ISR(인터럽트 서비스 루틴), sei()/cli()
#include <util/atomic.h>            // ATOMIC_BLOCK: 인터럽트 도중 값이 바뀌지 않게 보호
#include <avr/pgmspace.h>           // PROGMEM, PSTR, pgm_read_* : 데이터를 RAM 대신 플래시에 저장

// 비데가 세정 중 거치는 단계들. 순서대로
// 대기 -> 노즐 전진 -> 강하게 세정 -> 노즐 복귀 -> 약하게 헹굼 -> 건조 -> (다시 대기)
typedef enum {
  STATE_IDLE,            // 대기 상태
  STATE_NOZZLE_FORWARD,  // 노즐이 앞으로 나가는 중
  STATE_PUMP_ON,         // 설정한 수압으로 세정 중 (5초)
  STATE_NOZZLE_BACK,     // 노즐이 원위치로 들어가는 중
  STATE_PUMP_WEAK,       // 약한 수압으로 헹굼 (1초)
  STATE_DRY,             // 건조팬 동작 (3초)
} BidetState;

volatile BidetState current_state = STATE_IDLE;  // 지금 어떤 단계인지
volatile uint32_t state_start_time = 0;          // 현재 단계에 들어온 시각(ms) - 경과시간 계산용
void enter_state(uint8_t s);      // 상태를 바꿔주는 함수 (아래에서 정의) - Tinkercad/Arduino IDE가 자동으로
                                   // 만드는 함수 원형이 파일 맨 위(#include 직후, BidetState 정의보다 앞)에
                                   // 끼워들어가기 때문에 매개변수를 BidetState로 두면 "BidetState was not
                                   // declared in this scope" 에러가 남 -> uint8_t로 우회
void update_state(void);          // 매 루프마다 상태를 갱신하는 함수 (아래에서 정의)

// 아래쪽에서 정의되지만 더 앞에서 쓰이므로 전방 선언(원형만 미리 알려줌) 필요
// (없으면 암묵적 함수 선언으로 처리되어 32비트 반환값이 16비트로 잘릴 수 있음)
uint32_t millis_custom(void);   // 전원 켜진 뒤 지난 시간(ms)을 알려주는 함수
uint32_t micros_custom(void);   // 전원 켜진 뒤 지난 시간(us, 더 정밀함)을 알려주는 함수
void fan_on(void);                // 건조팬 켜기
void fan_off(void);               // 건조팬 끄기

// 노즐 위치를 4/5번 키로 바꿀 때, LED를 100ms만 반짝였다가 자동으로 꺼서
// "지금 바뀌었다"는 걸 눈으로 보여주기 위한 타이머 (delay 없이 동작)
volatile uint8_t blink_inc_active = 0;  // 증가(PD7) LED가 지금 깜빡이는 중인지
volatile uint8_t blink_dec_active = 0;  // 감소(PB0) LED가 지금 깜빡이는 중인지
uint32_t blink_inc_start = 0;           // 증가 LED가 켜진 시각(ms)
uint32_t blink_dec_start = 0;           // 감소 LED가 켜진 시각(ms)

// 증가 LED를 켜고 100ms 타이머 시작
static inline void start_inc_blink(void) {
    blink_inc_active   = 1;
    blink_inc_start    = millis_custom();
    PORTD |=  (1<<PD7);    // 증가 LED ON
}
// 감소 LED를 켜고 100ms 타이머 시작
static inline void start_dec_blink(void) {
    blink_dec_active   = 1;
    blink_dec_start    = millis_custom();
    PORTB |=  (1<<PB0);    // 감소 LED ON
}
// 메인 루프에서 계속 호출: 100ms가 지난 깜빡임 LED를 꺼준다
void blink_update(void) {
    uint32_t now = millis_custom();
    if (blink_inc_active && (now - blink_inc_start >= 100)) {
        PORTD &= ~(1<<PD7);   // 100ms 지났으니 소등
        blink_inc_active = 0;
    }
    if (blink_dec_active && (now - blink_dec_start >= 100)) {
        PORTB &= ~(1<<PB0);
        blink_dec_active = 0;
    }
}

// -------------------------------------------------------------------------------------------

// 타이머 함수들 (millis_custom / micros_custom)
// 아두이노의 millis()/micros()와 같은 역할을 직접 만든 것.
// "프로그램이 시작된 후 몇 ms(또는 us)가 지났는가"를 알려줘서,
// _delay_ms()처럼 CPU를 멈춰 세우지 않고도 "몇 초 후에 다음 동작"을 만들 수 있게 해준다.
// Timer2를 이 용도로 전용 사용 (Timer0은 pump_init()에서 펌프 PWM 전용으로 사용 -> 서로 간섭 없음)

volatile uint32_t systick_overflow_count = 0;  // 타이머가 0으로 넘어간(오버플로우) 누적 횟수
volatile uint32_t systick_millis = 0;          // 지금까지 흐른 시간(ms)
static uint8_t systick_fract = 0;              // 1ms 단위로 딱 안 떨어지는 오차를 보정하는 값

#define MILLIS_INC 1
#define FRACT_INC 3       // 1024 % 1000 = 24 -> 24 >> 3 = 3
#define FRACT_MAX 125     // 1000 >> 3 = 125

// Timer2가 0xFF(255)를 넘어갈 때마다(약 1.024ms마다) 자동으로 실행되는 인터럽트.
// 여기서 "시간이 흘렀다"는 걸 systick_millis에 계속 누적시킨다.
ISR(TIMER2_OVF_vect) {
	uint32_t m = systick_millis;
	uint8_t f = systick_fract;
	m += MILLIS_INC;
	f += FRACT_INC;

	if (f >= FRACT_MAX) {   // 오차가 쌓여 1ms가 되면 millis에 1을 더 더해줌
		f -= FRACT_MAX;
		m += 1;
	}
	systick_fract = f;
	systick_millis = m;
	systick_overflow_count++;
}

// Timer2를 "1.024ms마다 한 번씩 인터럽트 발생"하도록 설정하고 millis 카운트를 시작
void systick_init(void) {
	cli();                       // 설정 도중 다른 인터럽트가 끼어들지 않게 잠깐 금지
	TCCR2A = 0;                  // 일반 모드(그냥 계속 세기만 함)
	TCCR2B = (1 << CS22);       // 클럭을 64분주 (16MHz/64) 해서 타이머에 넣음
	TIMSK2 |= (1 << TOIE2);     // 오버플로우 인터럽트 켜기 (다 세면 위 ISR이 호출됨)

	systick_overflow_count = 0;
	systick_millis = 0;
	systick_fract = 0;
	sei();                       // 인터럽트 다시 허용
}

// 지금까지 흐른 시간을 ms 단위로 반환 (아두이노 millis()와 같은 역할)
uint32_t millis_custom(void) {
	uint32_t m;
	// ISR이 값을 바꾸는 도중에 읽어서 이상한 값이 나오지 않도록 잠깐 보호하며 읽음
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m = systick_millis;
	}
	return m;
}

// 지금까지 흐른 시간을 us(마이크로초) 단위로 반환 (millis_custom보다 더 정밀함)
uint32_t micros_custom(void) {
	uint32_t m;
	uint8_t t;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m = systick_overflow_count;
		t = TCNT2;   // 타이머가 지금 세고 있는 값(0~255)

		// 막 오버플로우가 났는데 아직 카운트에 반영 안 됐으면 여기서 보정
		if ((TIFR2 & (1 << TOV2)) && (t < 255)) {
			m++;
		}
	}
	return ((m << 8) + t) * (64 / (F_CPU / 1000000UL));
}

// -------------------------------------------------------------------------------------------

// 부저 함수 (일반 GPIO 토글 방식 - 타이머 하드웨어 파형생성 미사용)
// 원리: 스피커 핀을 아주 빠르게 켰다/껐다(High/Low) 반복하면 그 주파수만큼 소리가 난다.
// 여기서는 약 2kHz로 250us마다 핀을 반전시켜서 "삑" 소리를 100ms 동안 낸다.

#define BUZZER_DDR             DDRB
#define BUZZER_PORT            PORTB
#define BUZZER_PIN              PB1
#define BUZZER_HALF_PERIOD_US  250UL   // 약 2kHz (250us마다 반전 = 500us가 한 주기)
#define BUZZER_DURATION_MS     100UL   // 총 울리는 시간

volatile uint8_t buzzer_active = 0;     // 지금 부저가 울리는 중인지
uint32_t buzzer_start_time = 0;         // 울리기 시작한 시각(ms)
uint32_t buzzer_last_toggle_us = 0;     // 마지막으로 핀을 반전시킨 시각(us)

// 부저 울리기 시작 (버튼 누를 때마다 이 함수를 호출)
void buzzer_start(void) {
    BUZZER_DDR |= (1 << BUZZER_PIN);   // 부저 핀을 출력으로 설정
    buzzer_active = 1;
    buzzer_start_time = millis_custom();
    buzzer_last_toggle_us = micros_custom();
}

// 부저 끄기
void buzzer_stop(void) {
    BUZZER_PORT &= ~(1 << BUZZER_PIN);
    buzzer_active = 0;
}

// 메인 루프에서 계속 호출: 시간이 됐으면 핀을 반전시키거나, 100ms 지났으면 꺼준다
// (delay 없이 동작하므로 다른 작업을 막지 않음)
void buzzer_update(void) {
    if (!buzzer_active) return;   // 울리는 중이 아니면 할 일 없음

    if (millis_custom() - buzzer_start_time >= BUZZER_DURATION_MS) {
        buzzer_stop();   // 100ms 다 채웠으면 종료
        return;
    }

    uint32_t now_us = micros_custom();
    if (now_us - buzzer_last_toggle_us >= BUZZER_HALF_PERIOD_US) {
        BUZZER_PORT ^= (1 << BUZZER_PIN);   // 핀 상태 반전 (High<->Low)
        buzzer_last_toggle_us = now_us;
    }
}

// -------------------------------------------------------------------------------------------

// uart 함수
// UART는 컴퓨터(시리얼 모니터)로 텍스트를 보내서 디버깅하기 위한 통신 방법.
// 여기서는 TX(보내기)만 쓰고 RX(받기)는 사용하지 않는다.

// UART를 9600bps 속도로 초기화
void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud for 16MHz
    UCSR0B = (1 << TXEN0);   // TX만 사용, RX 비활성화
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 데이터 8비트 전송
}

// 문자 1개를 시리얼로 전송
void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));   // 이전 전송이 끝날 때까지 대기
    UDR0 = data;
}

// RAM에 있는 문자열(예: snprintf로 만든 문자열)을 한 글자씩 전송
void uart_print(const char* str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

// 플래시(PROGMEM)에 저장된 문자열을 한 글자씩 읽어와 전송
// - PSTR("...")로 감싼 문자열은 RAM이 아니라 플래시에 저장되어 SRAM을 절약할 수 있다
// - pgm_read_byte()로 플래시에서 한 바이트씩 읽어야 한다 (AVR은 플래시를 직접 못 읽음)
void uart_print_P(const char* str_P) {
    char c;
    while ((c = pgm_read_byte(str_P++))) {
        uart_transmit(c);
    }
}

// "[키이름] 동작설명" 형태로 시리얼에 출력 (예: "[4] nozzle phase - (2)")
void uart_print_key_P(const char* keyname_P, const char* label_P) {
    uart_transmit('[');
    uart_print_P(keyname_P);
    uart_transmit(']');
    uart_transmit(' ');
    uart_print_P(label_P);
    uart_transmit('\n');
}

// "[키이름] 동작설명 (단계)" 형태로 출력 (예: "[7] water pressure - (2)")
// level은 1~9 사이 한 자리 숫자만 쓴다고 가정 (여기서는 항상 1~3단계)
void uart_print_key_level_P(const char* keyname_P, const char* label_P, uint8_t level) {
    uart_transmit('[');
    uart_print_P(keyname_P);
    uart_transmit(']');
    uart_transmit(' ');
    uart_print_P(label_P);
    uart_transmit(' ');
    uart_transmit('(');
    uart_transmit('0' + level);   // 숫자를 문자로 변환 (예: 2 -> '2')
    uart_transmit(')');
    uart_transmit('\n');
}

// -------------------------------------------------------------------------------------------

// led 매트릭스, 키패드 매트릭스 함수
//
// LED 16개(4x4)와 키패드 16개(4x4 버튼)는 핀을 16개씩 쓰지 않고
// "행(row) 4개 + 열(column) 4개"로 배선해서 8개 핀만으로 제어한다 (매트릭스 방식).
// - LED: 한 번에 한 행만 켜고 나머지는 꺼서, 아주 빠르게 행을 바꿔가며 반복하면
//        사람 눈에는 잔상 때문에 여러 LED가 동시에 켜진 것처럼 보인다 (멀티플렉싱).
// - 키패드: 한 행씩 순서대로 LOW로 끌어내리고, 그 상태에서 각 열을 읽어서
//           어떤 버튼이 눌렸는지 알아낸다.
// - LED 매트릭스와 키패드 매트릭스는 "행" 핀(PB2~PB5)을 공유하고 "열" 핀은 따로 쓴다.
//   그래서 LED를 켤 때와 키패드를 스캔할 때 핀의 입출력 방향을 서로 바꿔가며 사용한다.

// LED 매트릭스를 쓰기 위해 관련 핀들을 출력으로 설정
void led_matrix_enter() {
    DDRB |= 0b1111<<2;   // PB2~PB5 : 행 선택 핀, 출력으로
    DDRC |= 0b1111<<0;   // PC0~PC3 : 열 선택 핀, 출력으로
}

uint8_t led_idx=0;   // 지금 몇 번째 행을 켜고 있는지 (0~3)
// LED 4x4의 각 행에서 어떤 열을 켤지 나타내는 패턴 (해당 비트가 0이면 켜짐)
uint8_t led_pattern[4]={
    0b0111,
    0b1111,
    0b0111,
    0b1111
};

// 메인 루프에서 계속 호출됨: 호출될 때마다 "다음 한 행"만 켜준다
// (여러 번 빠르게 호출되면서 행이 계속 바뀌어 잔상 효과가 생김)
void led_matrix_loop() {
    PORTB &= ~(0b1111<<2);     // 일단 모든 행을 끔
	PORTC = (PORTC & 0b11110000) | (led_pattern[led_idx] & 0x0F);  // 이번 행에서 켤 열 설정

    PORTB |= 1<<(5-led_idx);   // 이번 행만 켜기
    if(++led_idx>3) led_idx = 0;   // 다음 호출을 위해 행 번호 증가 (3 다음엔 0으로)
}

// LED 매트릭스 사용을 마치고 핀을 원래대로 되돌림 - 키패드 스캔과의 충돌 방지
void led_matrix_exit() {
    PORTB &= ~(0b1111<<2);   // 행을 먼저 꺼서 전환 중 불필요한 점등 방지
    PORTC |= 0b1111;         // 열을 전부 OFF(1)로 만들어둠. 이걸 안 하면 방금 켜져
                              // 있던 열만 PORTC 비트가 0으로 남은 채 DDRC가 입력으로
                              // 바뀌어 풀업 없이 완전히 플로팅됨 -> 곧바로 keypad_matrix_enter()가
                              // 공유된 행(PB2~PB5) 핀을 전부 HIGH로 만들 때 그 LED에
                              // 순간적으로 전류가 흘러 매 루프마다 깜빡이는 것처럼 보임
    DDRB &= ~(0b1111<<2);
    DDRC &= ~(0b1111<<0);
}

// 키패드 매트릭스를 쓰기 위해 핀 방향을 설정
// - 행(PB2~PB5): 출력, 평소엔 HIGH로 뒀다가 한 행씩 LOW로 내려서 스캔
// - 열(PD0,2,3,4): 입력 + 내부 풀업 (버튼 안 눌렸을 땐 자동으로 HIGH가 되게 함)
void keypad_matrix_enter() {
    DDRB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    PORTB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    DDRD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0));
    PORTD |=  (1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0);
}

// 키패드 스캔을 마치고 핀을 다시 원래대로(입력, 풀업 없음) 되돌림 - LED 매트릭스와의 충돌 방지
void keypad_matrix_exit() {
    DDRB &= ~((1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2));
    DDRD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0));
    PORTB &= ~((1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2));
    PORTD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2) | (1 << PD0));
}

// 키패드의 (행,열) 위치에 어떤 문자가 있는지 나타내는 표. PROGMEM이라 플래시에 저장됨
const char key_map[4][4] PROGMEM = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

uint8_t prev_key[4] = {0, 0, 0, 0};  // 직전 스캔에서 각 행의 어떤 열이 눌려있었는지 기억 (눌리는 순간만 감지하기 위함)

// (row, col) 위치의 LED를 켜거나(on=1) 끔(on=0)
void led_set(uint8_t row, uint8_t col, uint8_t on) {
    if (row > 3 || col > 3) return;
    if (on) led_pattern[row] &= ~(1 << (3 - col));   // 0으로 만들면 켜짐 (배선이 그렇게 되어 있음)
    else    led_pattern[row] |=  (1 << (3 - col));
}

// (row, col) 위치의 LED가 켜져 있는지 확인
uint8_t led_get(uint8_t row, uint8_t col) {
    if (row > 3 || col > 3) return 0;
    return !(led_pattern[row] & (1 << (3 - col)));
}

// 누적 막대(bar) 방식 3단계 LED 증가/감소 (수압처럼 1단계=●, 2단계=●●, 3단계=●●●)
// level: 갱신할 변수의 주소(포인터). 함수 안에서 *level 값을 직접 바꿔준다.
// row: LED 매트릭스에서 이 게이지가 있는 행 번호
// increase: 1이면 증가, 0이면 감소
static void bar_level_change(uint8_t *level, uint8_t row, uint8_t increase) {
    if (increase) {
        if (*level < 3) {
            (*level)++;
            led_set(row, *level - 1, 1);   // 새로 채워진 칸만 켜면 됨
        }
    } else {
        if (*level > 1) {
            led_set(row, *level - 1, 0);   // 제일 끝 칸만 끄면 됨
            (*level)--;
        }
    }
}

// 단일 점(dot) 방식 3단계 LED 증가/감소 (노즐위치처럼 항상 LED 1개만 켜져서 단계를 표시)
static void dot_level_change(uint8_t *level, uint8_t row, uint8_t increase) {
    if (increase) {
        if (*level < 3) {
            led_set(row, *level - 1, 0);   // 이전 위치 끄기
            (*level)++;
            led_set(row, *level - 1, 1);   // 새 위치 켜기
        }
    } else {
        if (*level > 1) {
            led_set(row, *level - 1, 0);
            (*level)--;
            led_set(row, *level - 1, 1);
        }
    }
}

// -------------------------------------------------------------------------------------------

// 상태변화 관련

uint8_t water_pressure = 1;     // 수압 단계 1~3
uint8_t seat_temperature = 0;   // 시트 히터 단계 0(꺼짐)~3
uint8_t nozzle_phase = 1;       // 노즐 위치 단계 1~3
uint8_t seat_sensor = 0;        // 사람이 앉아있으면 1, 아니면 0

volatile uint8_t sequence_active = 0;  // 비데/세정 시퀀스(2,3번 키로 시작한 전체 동작)가 진행 중인지

volatile uint8_t dry_mode_active = 0;  // A키로 시작한 "수동 건조"가 진행 중인지
uint32_t dry_mode_start_time = 0;      // 수동 건조를 시작한 시각(ms)

volatile uint8_t stop_requested = 0;   // "정지" 요청이 들어와서 처리 대기 중인지

// 정지(취소) 버튼 또는 시트센서 이탈 시 호출되는 함수.
// 어떤 상태에 있든 안전하게 대기 상태로 되돌리는 역할을 한다.
void stop() {
  buzzer_start();  // 비프음 먼저

  if (dry_mode_active) {
    fan_off();
    led_set(0, 3, 0);
    dry_mode_active = 0;
  }

  if (!sequence_active) return;   // 진행 중인 시퀀스가 없으면 stop_requested를 세우지 않는다 (다음 동작 오염 방지)

  sequence_active = 0;
  stop_requested = 1;

  // 이동 타이머가 아직 걸리지 않는 상태(전진/강세정)는 즉시 원위치 전이를 걸어준다.
  // NOZZLE_BACK/PUMP_WEAK/DRY는 update_state()의 stop_requested 처리에 맡긴다.
  // (특히 NOZZLE_BACK 도중이면 복귀 이동 시간을 다 채운 뒤에 IDLE로 가야 "노즐원위치" 요구사항을 만족)
  if (current_state == STATE_NOZZLE_FORWARD || current_state == STATE_PUMP_ON) {
    enter_state(STATE_NOZZLE_BACK);
  }
}


// —— 펌프 초기화 (OC0B=D5, PWM으로 수압 세기를 조절) ——
// 처음엔 TCCR0A를 건드리지 않는다 (COM0B1 미설정) - 타이머를 OC0B 핀에서
// 아예 분리해둔 채로 시작해서, 평범한 GPIO LOW로 꺼진 상태를 만든다.
// (OCR0B=0으로 "0% 듀티"를 표현하는 방식은 Tinkercad 시뮬레이터가 Fast PWM의
//  OCR0B=0 특수 케이스를 정확히 재현하지 못해 매 PWM 주기마다 짧은 펄스가 새어나가
//  펌프 LED가 계속 깜빡이는 것처럼 보이는 문제가 있었음 -> pump_set()에서
//  레벨 0일 때는 TCCR0A를 완전히 초기화해 핀을 타이머와 분리하고 GPIO로 강제 LOW)
void pump_init(void) {
    DDRD  |= (1<<PD5);                          // D5 출력
    PORTD &= ~(1<<PD5);                         // 시작은 확실한 GPIO LOW
    TCCR0B = (1<<CS01)|(1<<CS00);               // 분주비 64 (핀 연결과 무관하게 타이머는 계속 동작)
}

// —— 수압 단계별 PWM 설정 ——
// PWM 값이 클수록 평균적으로 핀이 HIGH인 시간이 길어져서 "더 세게" 켜진 것처럼 동작한다
// level: 0=꺼짐, 1=약, 2=중, 3=강
void pump_set(uint8_t level) {
    static const uint8_t tbl[4] = {0, 64, 128, 255};   // 각 단계에 해당하는 PWM 값(0~255)
    uint8_t val = (level < 4) ? tbl[level] : 0;
    OCR0B = val;
    if (val == 0) {
        TCCR0A = 0;              // OC0B를 타이머에서 완전히 분리 (일반 GPIO로 복귀)
        PORTD &= ~(1<<PD5);      // 흔들리지 않게 확실히 LOW로 고정
    } else {
        TCCR0A = (1<<COM0B1)|(1<<WGM01)|(1<<WGM00); // Fast PWM, 비반전 - 다시 연결
    }
}
// —— 노즐 방향 표시 LED ——
static inline void nozzle_led_forward(void) {
    PORTB |=  (1<<PB0);   // "노즐 전진" LED 켜기
}
static inline void nozzle_led_backward(void) {
    PORTD |=  (1<<PD7);   // "노즐 후진" LED 켜기
}
static inline void nozzle_led_off(void) {
    // 깜빡이는 중(blink_*)이 아닐 때만 꺼준다 - 안 그러면 깜빡임 효과와 서로 충돌함
    if (!blink_inc_active) PORTD &= ~(1<<PD7);
    if (!blink_dec_active) PORTB &= ~(1<<PB0);
}
// —— 건조팬 설정 (수동 건조와 시퀀스 건조 단계가 공유) ——
void fan_on(void) {
    DDRC  |= (1 << PC5);
    PORTC |= (1 << PC5);
}
void fan_off(void) {
    PORTC &= ~(1 << PC5);
}
// 노즐 위치(1/2/3단계)에 따라 전진/후진에 걸리는 시간(ms).
// 세정(wash)과 비데(bidet)가 서로 다른 이동 시간을 쓴다. PROGMEM으로 플래시에 저장해 SRAM 절약.
const uint16_t nozzle_fwd_time_bidet[3]  PROGMEM = {1000, 1500, 2000};
const uint16_t nozzle_back_time_bidet[3] PROGMEM = {1000, 1500, 2000};

const uint16_t nozzle_fwd_time_wash[3]   PROGMEM = {2000, 2500, 3000};
const uint16_t nozzle_back_time_wash[3]  PROGMEM = {2000, 2500, 3000};

// 지금 어떤 이동시간 표를 쓸지 가리키는 포인터 (2번/3번 키에 따라 바뀜)
const uint16_t *current_nozzle_fwd_time = nozzle_fwd_time_bidet;
const uint16_t *current_nozzle_back_time = nozzle_back_time_bidet;


// 수동 건조(A키) 시작 후 3초가 지나면 자동으로 꺼주는 함수. 메인 루프에서 계속 호출됨
void update_dry_mode() {
    if (dry_mode_active && (millis_custom() - dry_mode_start_time >= 3000)) {
        uart_print_P(PSTR("dry complete\n"));
        fan_off();
        led_set(0, 3, 0);
        dry_mode_active = 0;
    }
}

// 시트 히터를 설정 단계(N)에 맞춰 N초 간격으로 깜빡여주는 함수. 메인 루프에서 계속 호출됨
void seat_heater_blink_update() {
    static uint8_t state = 0; // 0: off, 1: on
    static uint32_t last_time = 0;

    uint32_t now = millis_custom();

    if (seat_temperature == 0) {   // 히터가 꺼져있으면 그냥 꺼진 상태 유지
        PORTC &= ~(1 << PC4);
        state = 0;
        return;
    }

    uint32_t interval = (uint32_t)seat_temperature * 1000UL;  // N초마다 토글 (on/off 대칭)
    if (now - last_time >= interval) {
        state = !state;   // 켜져있으면 끄고, 꺼져있으면 켬
        if (state) PORTC |= (1 << PC4);
        else       PORTC &= ~(1 << PC4);
        last_time = now;
    }
}

// 상태를 s로 바꾸고, 그 상태에 처음 들어갈 때 한 번만 해야 할 초기화 작업을 수행
void enter_state(uint8_t s) {
    current_state = (BidetState)s;
    state_start_time = millis_custom();   // 경과시간을 0부터 다시 재기 시작
    switch(s) {
      case STATE_IDLE:      // 대기: 모든 동작 정지
        nozzle_led_off();
        pump_set(0);
        fan_off();
        led_set(0, 3, 0);            // 건조 LED OFF
        uart_print_P(PSTR("STATE: IDLE\n"));
        break;
      case STATE_NOZZLE_FORWARD:   // 노즐 전진 시작
        nozzle_led_forward();
        uart_print_P(PSTR("STATE: NOZZLE_FORWARD\n"));
        break;
      case STATE_PUMP_ON:   // 설정된 수압으로 세정 시작
        nozzle_led_off();
        pump_set(water_pressure);
        uart_print_P(PSTR("STATE: PUMP_ON\n"));
        break;
      case STATE_NOZZLE_BACK:   // 노즐 복귀 시작
        nozzle_led_backward();
        uart_print_P(PSTR("STATE: NOZZLE_BACK\n"));
        break;
      case STATE_PUMP_WEAK:   // 약하게 헹구기 시작
        nozzle_led_off();
        pump_set(1);
        uart_print_P(PSTR("STATE: PUMP_WEAK\n"));
        break;
      case STATE_DRY:   // 건조 시작
        nozzle_led_off();
        pump_set(0);
        fan_on();
        led_set(0, 3, 1);
        uart_print_P(PSTR("STATE: DRY\n"));
        break;
    }
}

// 메인 루프에서 계속 호출됨: 현재 상태에서 얼마나 시간이 지났는지 확인하고,
// 정해진 시간(또는 조건)이 되면 다음 상태로 자동으로 넘어가게 한다
void update_state(void) {
    uint32_t e = millis_custom() - state_start_time;  // 이 상태에 들어온 지 얼마나 지났는지(ms)
    switch(current_state) {
      case STATE_NOZZLE_FORWARD:
        // 설정된 노즐 단계만큼 전진 시간이 다 지나면 세정 시작
        if (e >= pgm_read_word(&current_nozzle_fwd_time[nozzle_phase-1])) enter_state(STATE_PUMP_ON);
        break;
      case STATE_PUMP_ON:
        if (e >= 5000) {   // 강한 세정은 5초간 진행
          enter_state(STATE_NOZZLE_BACK);
        }
        break;
      case STATE_NOZZLE_BACK:
        if (e >= pgm_read_word(&current_nozzle_back_time[nozzle_phase - 1])) {
          if (stop_requested) {
              enter_state(STATE_IDLE);   // 정지 요청이 있었다면 복귀가 끝났으니 바로 대기로
              stop_requested = 0;
          } else {
              enter_state(STATE_PUMP_WEAK);   // 정상 진행이면 약하게 헹구는 단계로
          }
        }
        break;
      case STATE_PUMP_WEAK:
        if (stop_requested) {
          enter_state(STATE_IDLE);
          stop_requested = 0;
        } else if (e >= 1000) {   // 약한 헹굼은 1초간 진행
          enter_state(STATE_DRY);
        }
        break;
      case STATE_DRY:
        if (stop_requested || e >= 3000) {   // 건조는 3초, 또는 정지요청 시 바로 종료
          enter_state(STATE_IDLE);
          stop_requested = 0;
        }
        break;
      default: break;
    }
}


// -------------------------------------------------------------------------------------------

// 키패드 매트릭스 입력받으면 동작하는 부분

void debug_status(void);

// 4x4 키패드를 한 행씩 스캔해서, 눌린 키가 있으면 그 키에 해당하는 동작을 실행한다
void keypad_matrix_input(void) {
    for (uint8_t row = 0; row < 4; row++) {
        PORTB |= (0b1111 << 2);       // 모든 행을 일단 HIGH로
        PORTB &= ~(1 << (5 - row));   // 이번에 확인할 행만 LOW로 내림
        _delay_us(2);                 // 전압이 안정될 때까지 아주 잠깐 대기

        // 이번 행에서 어떤 열이 눌려있는지 확인 (풀업이라 눌리면 LOW가 됨)
        uint8_t key = 0;
        if (!(PIND & (1 << 4))) key |= (1 << 0);
        if (!(PIND & (1 << 3))) key |= (1 << 1);
        if (!(PIND & (1 << 2))) key |= (1 << 2);
        if (!(PIND & (1 << 0))) key |= (1 << 3);

        // 직전 스캔과 비교해서 "새로 눌린" 열만 골라냄 (누르고 있는 동안 계속 반응하지 않게)
        uint8_t rising = (key ^ prev_key[row]) & key;
        prev_key[row] = key;

        for (uint8_t col = 0; col < 4; col++) {
            if (rising & (1 << col)) {   // 이 열의 키가 이번에 새로 눌렸다면
                char key_char = pgm_read_byte(&key_map[row][col]);   // 눌린 키가 어떤 문자인지 찾기

                if (key_char == '4') {   // 노즐 위치 - (낮은 단계로)
                    if (seat_sensor) {
                        buzzer_start();
                        if (current_state == STATE_PUMP_ON || current_state == STATE_PUMP_WEAK) {
                            start_dec_blink();
                            dot_level_change(&nozzle_phase, 2, 0);
                            uart_print_key_level_P(PSTR("4"), PSTR("nozzle phase -"), nozzle_phase);
                        }
                    }
                } else if (key_char == '5') {   // 노즐 위치 + (높은 단계로)
                    if (seat_sensor) {
                        buzzer_start();
                        if (current_state == STATE_PUMP_ON || current_state == STATE_PUMP_WEAK) {
                            start_inc_blink();
                            dot_level_change(&nozzle_phase, 2, 1);
                            uart_print_key_level_P(PSTR("5"), PSTR("nozzle phase +"), nozzle_phase);
                        }
                    }
                } else if (key_char == '7') {   // 수압 -
                    if (seat_sensor) {
                        buzzer_start();
                        if (!dry_mode_active) {
                            bar_level_change(&water_pressure, 0, 0);
                            uart_print_key_level_P(PSTR("7"), PSTR("water pressure -"), water_pressure);
                        }
                    }
                } else if (key_char == '8') {   // 수압 +
                    if (seat_sensor) {
                        buzzer_start();
                        if (!dry_mode_active) {
                            bar_level_change(&water_pressure, 0, 1);
                            uart_print_key_level_P(PSTR("8"), PSTR("water pressure +"), water_pressure);
                        }
                    }
                } else if (key_char == 'B') {   // 시트 온도 단계 순환 (시트센서와 무관하게 항상 동작)
                    if (!dry_mode_active) {
                        buzzer_start();
                        if (led_get(1, 0) && led_get(1, 1) && led_get(1, 2)) {
                            uart_print_key_P(PSTR("B"), PSTR("change seat temperature (off)"));
                            led_set(1, 0, 0);
                            led_set(1, 1, 0);
                            led_set(1, 2, 0);
                            seat_temperature = 0;
                        } else if (led_get(1, 0) && led_get(1, 1)) {
                            uart_print_key_P(PSTR("B"), PSTR("change seat temperature (on : 3)"));
                            led_set(1, 2, 1);
                            seat_temperature = 3;
                        } else if (led_get(1, 0)) {
                            uart_print_key_P(PSTR("B"), PSTR("change seat temperature (on : 2)"));
                            led_set(1, 1, 1);
                            seat_temperature = 2;
                        } else {
                            uart_print_key_P(PSTR("B"), PSTR("change seat temperature (on : 1)"));
                            led_set(1, 0, 1);
                            seat_temperature = 1;
                        }
                    }
                } else if (key_char == 'A') {   // 수동 건조 시작
                    if (seat_sensor) {
                        buzzer_start();
                        if (!dry_mode_active) {
                            uart_print_key_P(PSTR("A"), PSTR("dry"));
                            fan_on();
                            led_set(0, 3, 1);
                            dry_mode_active = 1;
                            dry_mode_start_time = millis_custom();
                        }
                    }
                } else if (key_char == '1') {   // 정지
                    if (seat_sensor) {
                        uart_print_key_P(PSTR("1"), PSTR("STOP"));
                        stop();
                    }
                } else if (key_char == '2') {   // 세정(wash) 시작
                    if (seat_sensor) {
                        buzzer_start();
                        if (current_state == STATE_IDLE) {
                            uart_print_key_P(PSTR("2"), PSTR("wash start"));
                            sequence_active = 1;
                            current_nozzle_fwd_time  = nozzle_fwd_time_wash;
                            current_nozzle_back_time = nozzle_back_time_wash;
                            enter_state(STATE_NOZZLE_FORWARD);
                        }
                    }
                } else if (key_char == '3') {   // 비데(bidet) 시작
                    if (seat_sensor) {
                        buzzer_start();
                        if (current_state == STATE_IDLE) {
                            uart_print_key_P(PSTR("3"), PSTR("bidet start"));
                            sequence_active = 1;
                            current_nozzle_fwd_time  = nozzle_fwd_time_bidet;
                            current_nozzle_back_time = nozzle_back_time_bidet;
                            enter_state(STATE_NOZZLE_FORWARD);
                        }
                    }
                }

                else if (key_char == '*') {   // 디버깅용: 현재 상태를 시리얼로 출력
                    uart_print_key_P(PSTR("*"), PSTR("debug"));
                    buzzer_start();
                    debug_status();
                }

                else {   // 그 외 정의되지 않은 키는 눌린 문자만 시리얼로 출력
                    if (!dry_mode_active && seat_sensor) {
                        uart_transmit('[');
                        uart_transmit(key_char);
                        uart_transmit(']');
                        uart_transmit('\n');
                    }
                }
            }
        }
        PORTB |= (0b1111 << 2);   // 이번 행 스캔 끝, 다시 모두 HIGH로 되돌림
    }
}

// -------------------------------------------------------------------------------------------

// 슬라이드 스위치(시트 센서)
// 사람이 변기에 앉으면 눌리는 스위치. 내부 풀업을 사용하므로
// 안 눌렸을 땐 HIGH, 눌리면 LOW가 된다.

// 시트센서 핀을 입력 + 내부풀업으로 설정
void slide_switch_init() {
    DDRD &= ~(1 << PD6);
    PORTD |= (1 << PD6);
}

// 지금 사람이 앉아있으면 1, 아니면 0을 반환
uint8_t slide_switch_state() {
    return !(PIND & (1 << PD6));
}

uint8_t prev_slide_state = 0;   // 바로 이전에 읽었던 착석 상태 (변화를 감지하기 위함)

// 메인 루프에서 계속 호출됨: 착석 상태가 바뀌는 "순간"을 감지해서 처리
void handle_slide_switch() {
    uint8_t curr_state = slide_switch_state();
    if (curr_state && !prev_slide_state) {        // 방금 앉았다면
        buzzer_start();
        uart_print_P(PSTR("[SW] seat sensor on\n"));
        seat_sensor = 1;
    } else if (!curr_state && prev_slide_state) {  // 방금 일어났다면
        buzzer_start();
        uart_print_P(PSTR("[SW] seat sensor off\n"));
        stop();          // 안전을 위해 진행 중이던 동작을 모두 정지
        seat_sensor = 0;
    }
    prev_slide_state = curr_state;
}

// -------------------------------------------------------------------------------------------

// 디버깅용 함수들
// '*' 키를 누르면 현재 설정값들을 시리얼 모니터로 출력해서 상태를 확인할 수 있게 해준다
void debug_status() {	// 디버깅용
    char buffer[32];   // snprintf로 만든 문자열을 임시로 담아두는 공간

    uart_print_P(PSTR("=== Status ===\n"));

    snprintf_P(buffer, sizeof(buffer), PSTR("Water Pressure: %d\n"), water_pressure);
    uart_print(buffer);

    snprintf_P(buffer, sizeof(buffer), PSTR("Seat Temp: %d\n"), seat_temperature);
    uart_print(buffer);

    snprintf_P(buffer, sizeof(buffer), PSTR("Nozzle Phase: %d\n"), nozzle_phase);
    uart_print(buffer);

    snprintf_P(buffer, sizeof(buffer), PSTR("Seat Sensor: %d\n"), seat_sensor);
    uart_print(buffer);

    uart_print_P(PSTR("===============\n"));
}						// 디버깅용

// -------------------------------------------------------------------------------------------


int main() {
    // 각 기능별 초기화
    uart_init();
    slide_switch_init();
    systick_init();     // millis_custom/micros_custom 사용 준비
    pump_init();
    DDRD  |= (1<<PD7);  // 노즐 후진 LED 핀 출력
    PORTD &= ~(1<<PD7);
    DDRB  |= (1<<PB0);  // 노즐 전진 LED 핀 출력
    PORTB &= ~(1<<PB0);
    DDRC  |= (1<<PC4);  // 시트히터 LED 핀 출력 - 이게 없으면 seat_heater_blink_update()가
    PORTC &= ~(1<<PC4); // PORTC를 계속 토글해도 핀이 계속 입력 상태라 풀업만 켜졌다 꺼졌다
                         // 할 뿐 실제로는 절대 구동되지 않음(시트히터 LED가 켜지지 않음)


    // 전원을 켰을 때 이미 앉아있는 상태일 수도 있으니 초기값을 실제 센서값으로 맞춰줌
    prev_slide_state = slide_switch_state();
    seat_sensor = prev_slide_state;

    // 메인 루프: 아래 작업들을 계속 반복
    while(1) {
        handle_slide_switch();      // 착석 상태 확인
        update_state();             // 비데 시퀀스 상태 진행
        update_dry_mode();          // 수동 건조 타이머 확인
        buzzer_update();            // 부저 소리 재생 처리
        _delay_ms(1);
        led_matrix_enter();         // LED 매트릭스용 핀 설정
        led_matrix_loop();          // LED 한 행 갱신
        _delay_ms(1);
        led_matrix_exit();          // 핀을 키패드용으로 넘기기 전 정리
        keypad_matrix_enter();      // 키패드 매트릭스용 핀 설정
        keypad_matrix_input();      // 키패드 스캔 및 입력 처리
        keypad_matrix_exit();       // 핀을 LED용으로 넘기기 전 정리
        blink_update();             // 노즐 위치 변경 LED 깜빡임 처리
        seat_heater_blink_update(); // 시트 히터 깜빡임 처리

    }
}
