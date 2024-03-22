#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#define PWM_MAX_VALUE   1000    // ���PWM���ֵ
#define ENCODER_RESOLUTION  360  // �������ֱ���
#define TARGET_SPEED    100     // Ŀ���ٶ�
#define KP              1.0     // ����ϵ��
#define KI              0.0     // ����ϵ��
#define KD              0.0     // ΢��ϵ��

volatile uint16_t encoder_count1 = 0;
volatile uint16_t encoder_count2 = 0;
volatile int16_t current_speed = 0;
volatile int16_t target_speed = TARGET_SPEED;

enum MotorState {
    STOPPED,
    RUNNING
};

enum MotorState motor1_state = STOPPED;
enum MotorState motor2_state = STOPPED;
enum MotorState motor3_state = STOPPED;

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        // ���㵱ǰ�ٶ�
        current_speed = (encoder_count1 * 60000) / (ENCODER_RESOLUTION * 1000); // rpm

        // PID����
        int16_t error = target_speed - current_speed;
        int16_t output = KP * error + KI * error + KD * error;

        // �޷�
        if (output > PWM_MAX_VALUE)
            output = PWM_MAX_VALUE;
        else if (output < -PWM_MAX_VALUE)
            output = -PWM_MAX_VALUE;

        // ����PWM���
        TIM_SetCompare1(TIM1, abs(output));

        // �������������
        encoder_count1 = 0;
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);

        // ���±���������
        encoder_count1++;
    }
}

void TIM1_PWM_Init(void)
{
    // ��ʼ��PWM���
    // ���1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    // ���2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    // ���3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    // ���1
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    // ���2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    // ����PWM�������
    // ���1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    // ���2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    // ���3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    // ���1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    // ���2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;

    // ��������Ϊ�����������ģʽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM2_Encoder_Init(void)
{
    // ��ʼ���������ӿ�
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM2, ENABLE);
}

void EXTI_Configuration(void)
{
    // �����ⲿ�ж�
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_NVIC_Configuration(void)
{
    // ����TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void USART_Configuration(void)
{
    // ����USARTͨ��
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void USART_SendString(const char* string)
{
    while (*string)
    {
        while (!(USART1->SR & USART_FLAG_TXE));
        USART_SendData(USART1, *string++);
        while (!(USART1->SR & USART_FLAG_TC));
    }
}

void Motor_Control(enum MotorState state)
{
    switch (state)
    {
    case STOPPED:
        // ֹͣ����Ͷ��
        TIM_SetCompare1(TIM1, 0);
        TIM_SetCompare2(TIM1, 0);
        TIM_SetCompare3(TIM1, 0);
        TIM_SetCompare4(TIM1, 0);
        TIM_SetCompare2(TIM3, 0);
        break;

    case RUNNING:
        // ��������Ͷ��
        TIM_SetCompare1(TIM1, PWM_MAX_VALUE); // ���1
        TIM_SetCompare2(TIM1, PWM_MAX_VALUE); // ���2
        TIM_SetCompare3(TIM1, PWM_MAX_VALUE); // ���3
        TIM_SetCompare4(TIM1, PWM_MAX_VALUE); // ���1
        TIM_SetCompare2(TIM3, PWM_MAX_VALUE); // ���2
        break;
    }
}

int main(void)
{
    TIM1_PWM_Init();            // ��ʼ��PWM���
    TIM2_Encoder_Init();        // ��ʼ���������ӿ�
    EXTI_Configuration();       // �����ⲿ�ж�
    TIM2_NVIC_Configuration();  // ����TIM2�ж�
    USART_Configuration();      // ����USARTͨ��

    while (1)
    {
        // ���յ������ݴ洢
        char received_data;

        // ��������
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
        {
            received_data = USART_ReceiveData(USART1);

            // �������յ�������
            switch (received_data)
            {
            case '0':
                motor1_state = STOPPED;
                break;
            case '1':
                motor1_state = RUNNING;
                break;
                // ���Լ�����Ӹ�������������Ʋ�ͬ�ĵ���Ͷ��
            }

            // ���Ƶ��״̬
            Motor_Control(motor1_state);
        }
    }
}
