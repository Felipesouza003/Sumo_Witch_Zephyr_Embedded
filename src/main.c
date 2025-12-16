#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/timer/system_timer.h>

void tempo_ms(uint16_t);
void parar(void);
void atacar(void);
void frente(void);
void re(void);
void giro_eixo_D(void);
void giro_eixo_E(void);
void giro_travado_D(void);
void giro_travado_E(void);
void giro_re_E(void);
void giro_re_D(void);
void giro_passinho_D(void);
void giro_passinho_E(void);
void giro_bandeira_D(void);
void giro_bandeira_E(void);
void giro_desempate(void);

//Defines
// botoes do controle padrao sony
#define botao_0 1160
#define botao_1 8
#define botao_2 1032
#define botao_3 520
#define botao_4 1544
#define botao_5 264
#define botao_6 1288
#define botao_7 776
#define botao_8 1800
#define botao_9 136



// Dados do aut�mato (N�o pode ser declarado dentro da fun��o main por ser const)
#define NTRANS 80	// N�mero de Transi��es
#define NESTADOS 32 // N�mero de Estados
#define BUFFER 10	// M�ximo N�mero de Eventos no Buffer

// Mapeamento de eventos n�o control�veis como entradas
#define SF_D 0		   // Entrada 0
#define SF_E 1		   // Entrada 1
#define SP_FD_achou 2  // Entrada 2
#define SP_FD_perdeu 3 // Entrada 3
#define SP_FE_achou 4  // Entrada 4
#define SP_FE_perdeu 5 // Entrada 5
#define SP_LD 6		   // Entrada 6
#define SP_LE 7		   // Entrada 7

// Mapeamento de eventos control�veis
#define est_1 8
#define est_2 9
#define est_3 10
#define est_4 11
#define est_5 12
#define est_6 13
#define iniciar 14
#define prepara 15
#define t 16

/*Pwms para os motores*/
static const struct pwm_dt_spec pwm_a = PWM_DT_SPEC_GET(DT_ALIAS(pwm_a));
static const struct pwm_dt_spec pwm_b = PWM_DT_SPEC_GET(DT_ALIAS(pwm_b));

/* Ponte H */
static const struct gpio_dt_spec ain_frente =
	GPIO_DT_SPEC_GET(DT_ALIAS(ain1_frente), gpios);
static const struct gpio_dt_spec ain_re =
	GPIO_DT_SPEC_GET(DT_ALIAS(ain1_re), gpios);

static const struct gpio_dt_spec bin_frente =
	GPIO_DT_SPEC_GET(DT_ALIAS(bin1_frente), gpios);
static const struct gpio_dt_spec bin_re =
	GPIO_DT_SPEC_GET(DT_ALIAS(bin1_re), gpios);

static const struct gpio_dt_spec stby =
	GPIO_DT_SPEC_GET(DT_ALIAS(stby), gpios);

/* Sensores */
static const struct gpio_dt_spec sp_le =
    GPIO_DT_SPEC_GET(DT_ALIAS(sp_le), gpios);

static const struct gpio_dt_spec sp_ld =
    GPIO_DT_SPEC_GET(DT_ALIAS(sp_ld), gpios);

static const struct gpio_dt_spec sp_fe =
    GPIO_DT_SPEC_GET(DT_ALIAS(sp_fe), gpios);

static const struct gpio_dt_spec sp_fd =
    GPIO_DT_SPEC_GET(DT_ALIAS(sp_fd), gpios);

/* ===== LEDs ===== */
static const struct gpio_dt_spec led1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

static const struct gpio_dt_spec led2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static const struct gpio_dt_spec led3 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

static const struct gpio_dt_spec led4 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led4), gpios);

static const struct gpio_dt_spec led5 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led5), gpios);


#define AI2_re_ON gpio_pin_set_dt(&ain_re, 1); // Saida ON AIN
#define AI2_re_OFF gpio_pin_set_dt(&ain_re, 0);	// Saida 0 OFF
#define AI1_frente_ON gpio_pin_set_dt(&ain_frente, 1);	  // Saida 0 ON
#define AI1_frente_OFF gpio_pin_set_dt(&ain_frente, 0); // Saida 0 OFF
#define BI1_frente_ON gpio_pin_set_dt(&bin_frente, 0);  // Saida 0 ON
#define BI1_frente_OFF gpio_pin_set_dt(&bin_frente, 1);
#define BI2_re_ON gpio_pin_set_dt(&bin_re, 0); // Saida 0 ON
#define BI2_re_OFF gpio_pin_set_dt(&bin_re, 1);
#define LED1_ON gpio_pin_set_dt(&led1, 1);	  // Saida 0 ON
#define LED1_OFF gpio_pin_set_dt(&led1, 0); // Saida 0 OFF
#define LED2_ON gpio_pin_set_dt(&led2, 1);	  // Saida 0 ON
#define LED2_OFF gpio_pin_set_dt(&led2, 0); // Saida 0 OFF
#define LED3_ON gpio_pin_set_dt(&led3, 1);	  // Saida 0 ON
#define LED3_OFF gpio_pin_set_dt(&led3, 0); // Saida 0 OFF
#define LED4_ON gpio_pin_set_dt(&led4, 1);	  // Saida 0 ON
#define LED4_OFF gpio_pin_set_dt(&led4, 0); // Saida 0 OFF
#define LED5_ON gpio_pin_set_dt(&led5, 1);	  // Saida 0 ON
#define LED5_OFF gpio_pin_set_dt(&led5, 0); // Saida 0 OFF
#define STBY_ON gpio_pin_set_dt(&stby, 1);	  // Saida 0 ON
#define STBY_OFF gpio_pin_set_dt(&stby, 0); // Saida 0 OFF



//Variaveis
// dados do temporizador
volatile uint8_t flag_counter = 1; // timer estourado
volatile uint32_t counter = 0;	   // contagem zerada
volatile uint32_t tempo_estouro;   // tempo de estouro
uint16_t ADC_buffer[3] = {3000, 3000, 3000}; // buffer do ADC

// Variáveis do receptor
uint32_t riseTime = 0, fallTime = 0;
uint32_t highTime = 0, lowTime = 0;
uint32_t period = 0;
uint16_t duty = 50400;

uint16_t estado = 0;   // estado de decodificação
int16_t tmp_data = -1; // dado lido do controle remoto
int16_t data = -1;

// tratamento do sensor de faixa
uint8_t borda_e = 0;
uint8_t borda_e_ant = 0;
uint8_t borda_d = 0;
uint8_t borda_d_ant = 0;

// listas encadeadas do automato
const unsigned int event[NTRANS]={16,16,7,6,5,3,16,3,0,5,1,15,15,15,15,15,15,14,14,14,14,14,14,7,6,4,2,0,1,16,16,16,16,16,0,7,4,6,1,2,1,6,2,4,7,0,16,16,16,2,1,
		0,4,4,1,0,2,3,1,0,4,7,5,1,0,2,6,8,9,10,11,12,13,16,7,0,4,1,2,6};
const unsigned int in_state[NTRANS]={20,19,25,24,28,29,2,29,0,28,1,10,11,12,13,14,15,16,26,27,17,18,23,27,26,29,28,0,1,31,21,22,16,16,0,27,29,26,1,28,1,26,28,29,27,0,16,3,3,28,1,
		0,29,29,1,0,28,26,1,0,2,27,27,1,0,2,26,4,5,6,7,8,9,16,27,0,29,1,28,26};
const unsigned int rfirst[NESTADOS] = {1,2,7,11,12,13,14,15,16,17,18,19,20,21,22,23,30,31,32,33,34,40,46,47,48,49,53,57,62,67,73,80};
const unsigned int rnext[NTRANS] = {0,0,0,3,4,5,6,0,8,9,10,0,0,0,0,0,0,0,0,0,0,0,0,0,24,25,26,27,28,29,0,0,0,0,0,35,36,37,38,39,0,41,42,43,44,45,0,0,0,0,50,
		51,52,0,54,55,56,0,58,59,60,61,0,63,64,65,66,0,68,69,70,71,72,0,74,75,76,77,78,79};

// Declaraçao de variaveis globais
unsigned char buffer[BUFFER]; // Buffer para armazenar a fila de eventos externos
unsigned char n_buffer = 0;	  // N mero de eventos no Buffer

unsigned int k;
int occur_event;				 // Evento ocorrido
unsigned int current_state = 30; // Estado atual inicializado com estado inicial
char g = 0;						 // Flag para gerador aleatório de eventos
char gerar_evento = 1;			 // Flag para habilitar a temporização de eventos controláveis
char moore_output = 0;			 // Inicializa saída periférica
int bandeira_state = 0;

volatile uint32_t tempo_atual = 0;
volatile uint32_t tempo_antigo = 0;
uint8_t count = 1;

/* Stack sizes */
#define STACK_SIZE 1024

/* Thread priorities (lower = higher priority) */
#define PRIO_T1 2
#define PRIO_T2 1
#define PRIO_T3 3
#define PRIO_T4 4

//Callbacks
static struct gpio_callback cb_sp_fe;
static struct gpio_callback cb_sp_fd;
static struct gpio_callback cb_sp_le;
static struct gpio_callback cb_sp_ld;

/* Semaphores */
K_SEM_DEFINE(sem_adc_thread, 1, 1);
K_SEM_DEFINE(sem_automato_thread, 0, 1);
K_SEM_DEFINE(sem_blink_thread, 0, 1);
K_SEM_DEFINE(sem_ir_thread, 0, 1);

/* Timer callback function */
void cb_time_ms(struct k_timer *timer_id);
void tempo_ms(uint16_t time_ms);
/*Timers define*/
K_TIMER_DEFINE(move_timer, cb_time_ms, NULL);


/* Thread prototypes */
void adc_thread(void *, void *, void *);
void automato_thread(void *, void *, void *);
void blink_thread(void *, void *, void *);
void ir_thread(void *, void *, void *);

/* Thread stacks */
K_THREAD_STACK_DEFINE(stack_adc_thread, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_automato_thread, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_blink_thread, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_ir_thread, STACK_SIZE);

/* Thread control blocks */
static struct k_thread adc_thread_data;
static struct k_thread automato_thread_data;
static struct k_thread blink_thread_data;
static struct k_thread ir_thread_data;

/* Thread implementations */
void adc_thread(void *a, void *b, void *c)
{
    while (1) {
        k_sem_take(&sem_adc_thread, K_FOREVER);

        /* Thread 1 code */
        if (ADC_buffer[0] < 400) // achou faixa branca sensor da direita
		{
			if (borda_d == 0)
			{
				borda_d = 1;
				buffer[n_buffer] = SF_D; // Atribuir evento à borda de subida do sensor de faixa direita
				n_buffer++;
			}
		}
		if (ADC_buffer[0] > 2000)
			borda_d = 0; // sensor da direita na faixa preta

		if (ADC_buffer[2] < 400) // achou faixa branca sensor da esquerda
		{
			if (borda_e == 0)
			{
				borda_e = 1;
				buffer[n_buffer] = SF_E; // Atribuir evento à borda de subida do sensor de faixa direita
				n_buffer++;
			}
		}
		if (ADC_buffer[2] > 2000)
			borda_e = 0; // sensor da esquerda na faixa preta
        
        if (ADC_buffer[1] < 2931) // se bateria < xx (colocar valor) desliga o robo
		{
			current_state = 30; // estado inicial
			moore_output = 1;
			LED5_ON;
			LED1_ON;
			STBY_OFF;
		}
		else
		{
			LED5_OFF;
			LED1_OFF;
			STBY_ON;
		}
        k_sem_give(&sem_automato_thread);
        k_sleep(K_MSEC(10));
    }
}

void automato_thread(void *a, void *b, void *c)
{
    while (1) {
        k_sem_take(&sem_automato_thread, K_FOREVER);

        /* Thread 2 code */
        if (n_buffer == 0) // se n o existir evento no buffer ent o gerar um evento interno(evento control vel)
		{
			if (flag_counter) // Se o timer estourar, habilita a gera  o de eventos
			{
				gerar_evento = 1;
			}
			if (gerar_evento == 1)
			{
				switch (g) // Aqui   implementado um gerador autom tico de eventos control veis
				{
				case (0):
					occur_event = t;
					g = 0;
					break;
				}
			}
            if(data == -1)
                data = botao_7; // testar com botao 7
			switch (data) // tabao tabao num vo aguenta
			{
			case (botao_7): // frente pica-pau
				occur_event = est_1;
				data = botao_1;
				break;
			case (botao_8): // gira direita
				occur_event = est_2;
				data = -1;
				break;
			case (botao_9): // gira esquerda
				occur_event = est_3;
				data = -1;
				break;
			case (botao_0): // passinho pra esquerda
				occur_event = est_4;
				data = -1;
				break;
			case (botao_5): // passinho pra direita
				occur_event = est_5;
				data = -1;
				break;
			case (botao_6): // desempate
				occur_event = est_6;
				data = -1;
				break;				
			case (botao_1):
				occur_event = prepara;
				LED3_ON;
				tempo_ms(50);
				LED3_OFF;
				tempo_ms(50);
				LED3_ON;
				tempo_ms(50);
				LED3_OFF;
				tempo_ms(50);
				LED3_ON;
				tempo_ms(50);
				LED3_OFF;
				tempo_ms(50);
				LED3_ON;
				tempo_ms(50);
				LED3_OFF;
				tempo_ms(50);
				LED3_ON;
				tempo_ms(50);
				LED3_OFF;
				tempo_ms(50);
				LED3_ON;
				tempo_ms(50);
				LED3_OFF;
				tempo_ms(50);
				LED3_ON;
				data = botao_2;
				break;
			case (botao_2):
				occur_event = iniciar;
				LED5_OFF;

				if (gpio_pin_get_dt(&sp_fe) && gpio_pin_get_dt(&sp_fd))
				{
					count = 1;
					current_state = 2; // Estado ataca
					moore_output = 1;
                    data = 0;
				}
				break;
            default:
                break;
			}
		}

		else // se existir evento n o control vel pegar do buffer
		{
			occur_event = buffer[0];
			n_buffer--;
			k = 0;
			while (k < n_buffer)
			{
				buffer[k] = buffer[k + 1];
				k += 1;
			}
		}

		// Jogador de aut mato
		k = rfirst[current_state];
		if (k == 0)
		{
			return 1; // Dead Lock!!!
		}
		else
		{
			while (k > 0)
			{
				k -= 1;
				if (event[k] == occur_event)
				{
					current_state = in_state[k];
					moore_output = 1;
					break;
				}
				k = rnext[k];
			}
		}

		if (moore_output) // Se o evento ocorrido for v�lido, ent�o imprimir sa�da f�sica
		{
			gerar_evento = 1;
			switch (current_state)
			{
			case (0): // Adicionar A��o para o Estado RE_1;
				re();
				tempo_ms(1500);
				gerar_evento = 0;
				break;
			case (1): // Adicionar A��o para o Estado RE_2;
				re();
				tempo_ms(1500);
				gerar_evento = 0;
				break;
			case (2): // Adicionar A��o para o Estado ataca;
				atacar();
				//tempo_ms(10);
				//gerar_evento = 0;
				break;
			case (3): // Adicionar A��o para o Estado ataca2;
				LED3_ON;
				break;
			case (4): // Adicionar A��o para o Estado espera;
				LED3_ON;
				break;
			case (5): // Adicionar A��o para o Estado espera2;
				LED3_ON;
				break;
			case (6): // Adicionar A��o para o Estado espera3;
				LED3_ON;
				break;
			case (7): // Adicionar A��o para o Estado espera4;
				LED3_ON;
				break;
			case (8): // Adicionar A��o para o Estado espera5;
				LED3_ON;
				break;
			case(9): // Adicionar A��o para o Estado espera6;
				LED3_ON;
					break;
			case (10): // Adicionar A��o para o Estado espera_ini;
				LED1_ON;
				LED2_ON;
				break;
			case (11): // Adicionar A��o para o Estado espera_ini2;
				LED1_ON;
				LED2_ON;
				break;
			case (12): // Adicionar A��o para o Estado espera_ini3;
				LED1_ON;
				LED2_ON;
				break;
			case (13): // Adicionar A��o para o Estado espera_ini4;
				LED1_ON;
				LED2_ON;
				break;
			case (14): // Adicionar A��o para o Estado espera_ini5;
				LED1_ON;
				LED2_ON;
				break;
				case(15):	//Adicionar A��o para o Estado espera_ini6;
				LED1_ON;
				LED2_ON;
					break;
				case(16):	//Adicionar A��o para o Estado frente;
                    frente();
                    tempo_ms(250);
                    LED2_OFF;
                    LED3_OFF;
                    LED4_OFF;
                    LED5_ON;
                    gerar_evento = 0;
					break;
				case(17):	//Adicionar A��o para o Estado frente_passinho_1;
				atacar();	   // USAR O ATACAR TALVZ?
				tempo_ms(1100); // MEXER NO TEMPO
				gerar_evento = 0;
					break;
				case(18):	//Adicionar A��o para o Estado frente_passinho_2;
				atacar();	   // USAR O ATACAR TALVZ?
				tempo_ms(1100); // MEXER NO TEMPO
				gerar_evento = 0;
					break;
				case(19):	//Adicionar A��o para o Estado gira_e_D;
				giro_re_D();
				tempo_ms(2700);
				gerar_evento = 0;
					break;
				case(20):	//Adicionar A��o para o Estado gira_e_E;
				giro_re_E();
				tempo_ms(2700);
				gerar_evento = 0;
					break;
				case(21):	//Adicionar A��o para o Estado gira_passinho_D_1;
				giro_passinho_D(); // MEXER NA VELOCIDADE DO GIRO E NO TEMPO
				tempo_ms(1000);
				gerar_evento = 0;
					break;
				case(22):	//Adicionar A��o para o Estado gira_passinho_E_2;
				giro_passinho_E(); // MEXER NA VELOCIDADE DO GIRO E TEMPO
				tempo_ms(1000);
				gerar_evento = 0;
					break;
				case(23):	//Adicionar A��o para o Estado giro_180;
				giro_desempate();
				tempo_ms(1500);
				gerar_evento = 0;
					break;
				case(24):	//Adicionar A��o para o Estado giro_bandeira_D;
				giro_bandeira_D();
				tempo_ms(1500);
				gerar_evento = 0;
					break;
				case(25):	//Adicionar A��o para o Estado giro_bandeira_E;
				giro_bandeira_E();
				tempo_ms(1500);
				gerar_evento = 0;
					break;
				case(26):	//Adicionar A��o para o Estado giro_eixo_D;
				giro_eixo_D();
					break;
				case(27):	//Adicionar A��o para o Estado giro_eixo_E;
				giro_eixo_E();
					break;
				case(28):	//Adicionar A��o para o Estado giro_travado_D;
				giro_eixo_D();
					break;
				case(29):	//Adicionar A��o para o Estado giro_travado_E;
				giro_eixo_E();
					break;
				case(30):	//Adicionar A��o para o Estado inicio;
				parar();
					break;
				case(31):	//Adicionar A��o para o Estado para;
                LED4_ON;
                LED3_ON;
                LED2_ON
                LED5_ON;
				parar();
				gerar_evento = 0;
                tempo_ms(5000);
					break;
			} // fim switch
			moore_output = 0;
			occur_event = -1;
		} // fim if(moore_output)
        k_sem_give(&sem_blink_thread);
        k_sleep(K_MSEC(10));
    }
}

void blink_thread(void *a, void *b, void *c)
{
    while (1) {
        k_sem_take(&sem_blink_thread, K_FOREVER);

        /* Thread 3 code */
        gpio_pin_set_dt(&led1, 1);
        k_sleep(K_MSEC(300));

        gpio_pin_set_dt(&led1, 0);
        k_sleep(K_MSEC(300));

        k_sem_give(&sem_ir_thread);
        k_sleep(K_MSEC(10));
    }
}

void ir_thread(void *a, void *b, void *c)
{
    while (1) {
        k_sem_take(&sem_ir_thread, K_FOREVER);

        /* Thread 4 code */

        k_sem_give(&sem_adc_thread);
        k_sleep(K_MSEC(10));
    }
}
// Tratamento das interrupções
/* Callback para sensor de presença frontal direito */
void sp_fd_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&sp_fd))
	{
		buffer[n_buffer] = SP_FD_achou;
		n_buffer++;
	}
	else
	{
		buffer[n_buffer] = SP_FD_perdeu;
		n_buffer++;
	}
}

/* Callback para sensor de presença frontal esquerdo */
void sp_fe_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&sp_fe))
	{
		buffer[n_buffer] = SP_FE_achou;
		n_buffer++;
	}
	else
	{
		buffer[n_buffer] = SP_FE_perdeu;
		n_buffer++;
	}
}

/* Callback para sensor de presença lateral direito */
void sp_ld_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&sp_ld))
	{
		buffer[n_buffer] = SP_LD;
		n_buffer++;
	}
}

/* Callback para sensor de presença lateral esquerdo */
void sp_le_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&sp_le))
	{
		buffer[n_buffer] = SP_LE;
		n_buffer++;
	}
}

int main(void)
{
    /* Verifica se todos os dispositivos estão prontos */
    if (!device_is_ready(sp_le.port) ||
        !device_is_ready(sp_ld.port) ||
        !device_is_ready(sp_fe.port) ||
        !device_is_ready(sp_fd.port) ||
        !device_is_ready(led1.port)  ||
        !device_is_ready(led2.port)  ||
        !device_is_ready(led3.port)  ||
        !device_is_ready(led4.port)  ||
        !device_is_ready(led5.port)) {
        return 0;
    }

    /*ponte h*/
	gpio_pin_configure_dt(&ain_frente, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&ain_re, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&bin_frente, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&bin_re, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&stby, GPIO_OUTPUT_HIGH);

    /* Configura sensores como entrada */
    gpio_pin_configure_dt(&sp_le, GPIO_INPUT);
    gpio_pin_configure_dt(&sp_ld, GPIO_INPUT);
    gpio_pin_configure_dt(&sp_fe, GPIO_INPUT);
    gpio_pin_configure_dt(&sp_fd, GPIO_INPUT);

    /* Configura LEDs como saída */
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led4, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led5, GPIO_OUTPUT_INACTIVE);

    /* Configura interrupções para bordas de subida e descida */
    gpio_pin_interrupt_configure_dt(&sp_fd, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&sp_fe, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&sp_ld, GPIO_INT_EDGE_RISING);
    gpio_pin_interrupt_configure_dt(&sp_le, GPIO_INT_EDGE_RISING);

    /* Inicializa e adiciona callbacks */
    gpio_init_callback(&cb_sp_fd, sp_fd_callback, BIT(sp_fd.pin));
    gpio_init_callback(&cb_sp_fe, sp_fe_callback, BIT(sp_fe.pin));
    gpio_init_callback(&cb_sp_ld, sp_ld_callback, BIT(sp_ld.pin));
    gpio_init_callback(&cb_sp_le, sp_le_callback, BIT(sp_le.pin));

    gpio_add_callback(sp_fd.port, &cb_sp_fd);
    gpio_add_callback(sp_fe.port, &cb_sp_fe);
    gpio_add_callback(sp_ld.port, &cb_sp_ld);
    gpio_add_callback(sp_le.port, &cb_sp_le);

    k_thread_create(&adc_thread_data, stack_adc_thread, STACK_SIZE,
                    adc_thread, NULL, NULL, NULL,
                    PRIO_T1, 0, K_NO_WAIT);

    k_thread_create(&automato_thread_data, stack_automato_thread, STACK_SIZE,
                    automato_thread, NULL, NULL, NULL,
                    PRIO_T2, 0, K_NO_WAIT);

    k_thread_create(&blink_thread_data, stack_blink_thread, STACK_SIZE,
                    blink_thread, NULL, NULL, NULL,
                    PRIO_T3, 0, K_NO_WAIT);

    k_thread_create(&ir_thread_data, stack_ir_thread, STACK_SIZE,
                    ir_thread, NULL, NULL, NULL,
                    PRIO_T4, 0, K_NO_WAIT);

    return 0;
}

void parar()
{
	count = 1;
	AI1_frente_OFF;
	AI2_re_OFF;
	BI1_frente_OFF;
	BI2_re_OFF;
	// STBY_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, 0);//0%
	pwm_set_dt(&pwm_b, pwm_b.period, 0);//0%
}

void atacar()
{
	// STBY_ON;
	AI1_frente_ON;
	AI2_re_OFF;
	BI1_frente_ON;
	BI2_re_OFF;

	switch (count)
	{
	case 1:
        pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/3.3);//30%
	    pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/4);//25%
		tempo_ms(20);
		gerar_evento = 0;
		count = 2;
		break;
	case 2:
		pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/1.65);//60%
		pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/1.8);//55%
		tempo_ms(20);
		gerar_evento = 0;
		count = 3;
		break;
	case 3:
		pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period);//100%
		pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period);//100%
		break;
	}
}
void frente()
{
	count = 1;
	STBY_ON;
	AI1_frente_ON;
	AI2_re_OFF;
	BI1_frente_ON;
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/5);//20%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/5);//20%
}
void re()
{
	count = 1;
	// STBY_ON;
	AI1_frente_OFF;
	AI2_re_ON;
	BI1_frente_OFF;
	BI2_re_ON;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/1.1);//90%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/2.5);//40%
}

void giro_eixo_D()
{
	count = 1;
	// STBY_ON;
	AI1_frente_ON; // A é esquerda
	AI2_re_OFF;
	BI1_frente_OFF; // B é direita
	BI2_re_ON;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/5);//20%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/5);//20%
}
void giro_travado_D()
{
	count = 1;
	// STBY_ON;
	AI1_frente_ON;
	AI2_re_OFF;
	BI1_frente_ON;
	BI2_re_OFF;
    pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/5);//20%
	pwm_set_dt(&pwm_b, pwm_b.period, 0);//0%
}
void giro_eixo_E()
{
	count = 1;
	// STBY_ON;
	AI1_frente_OFF; // A é esqueda
	AI2_re_ON;
	BI1_frente_ON; // B é direita
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/5);//20%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/5);//20%
}
void giro_travado_E()
{
	count = 1;
	// STBY_ON;
	AI1_frente_ON; // A é esqueda
	AI2_re_OFF;
	BI1_frente_ON; // B é direita
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, 0);//0%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/5);//20%
}

void giro_re_E()
{
	count = 1;
	AI1_frente_OFF; // A é esqueda
	AI2_re_ON;
	BI1_frente_ON; // B é direita
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/2.5);//40%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/2.5);//40%
}

void giro_re_D()
{
	count = 1;
	// STBY_ON;
	AI1_frente_ON; // A é esquerda
	AI2_re_OFF;
	BI1_frente_OFF; // B é direita
	BI2_re_ON;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/2.5);//40%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/2.5);//40%
}

void giro_passinho_E()
{
	count = 1;
	// STBY_ON;
	AI1_frente_OFF; // A é esqueda
	AI2_re_ON;
	BI1_frente_ON; // B é direita
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/2.5);//40%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/2.5);//40%
}

void giro_passinho_D()
{
	count = 1;
	// STBY_ON;
	AI1_frente_ON; // A é esquerda
	AI2_re_OFF;
	BI1_frente_OFF; // B é direita
	BI2_re_ON;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/2.5);//40%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/2.5);//40%
}

void giro_bandeira_D()
{
	count = 1;
	// STBY_ON;
	AI1_frente_ON; // A é esquerda
	AI2_re_OFF;
	BI1_frente_OFF; // B é direita
	BI2_re_ON;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/1.5);//60%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/1.5);//60%
}

void giro_bandeira_E()
{
	count = 1;
	// STBY_ON;
	AI1_frente_OFF; // A é esqueda
	AI2_re_ON;
	BI1_frente_ON; // B é direita
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period/1.5);//60%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period/1.5);//60%
}

void giro_desempate()
{
	count = 1;
	// STBY_ON;
	AI1_frente_OFF; // A é esqueda
	AI2_re_ON;
	BI1_frente_ON; // B é direita
	BI2_re_OFF;
	pwm_set_dt(&pwm_a, pwm_a.period, pwm_a.period);//100%
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period);//100%
}

void cb_time_ms(struct k_timer *timer_id)
{
    flag_counter = 1;
    k_sem_give(&sem_automato_thread);
}
void tempo_ms(uint16_t time_ms)
{
    k_timer_start(&move_timer, K_MSEC(time_ms), K_NO_WAIT);
}