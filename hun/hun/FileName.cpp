#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#define PWM_MAX_VALUE   1000    // 电机PWM最大值
#define ENCODER_RESOLUTION  360  // 编码器分辨率
#define TARGET_SPEED    100     // 目标速度
#define KP              1.0     // 比例系数
#define KI              0.0     // 积分系数
#define KD              0.0     // 微分系数

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

        // 计算当前速度
        current_speed = (encoder_count1 * 60000) / (ENCODER_RESOLUTION * 1000); // rpm

        // PID控制
        int16_t error = target_speed - current_speed;
        int16_t output = KP * error + KI * error + KD * error;

        // 限幅
        if (output > PWM_MAX_VALUE)
            output = PWM_MAX_VALUE;
        else if (output < -PWM_MAX_VALUE)
            output = -PWM_MAX_VALUE;

        // 设置PWM输出
        TIM_SetCompare1(TIM1, abs(output));

        // 清零编码器计数
        encoder_count1 = 0;
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);

        // 更新编码器计数
        encoder_count1++;
    }
}

void TIM1_PWM_Init(void)
{
    // 初始化PWM输出
    // 电机1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    // 电机2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    // 电机3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    // 舵机1
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    // 舵机2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    // 设置PWM输出引脚
    // 电机1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    // 电机2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    // 电机3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    // 舵机1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    // 舵机2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;

    // 设置引脚为复用推挽输出模式
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
    // 初始化编码器接口
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
    // 配置外部中断
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
    // 配置TIM2中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void USART_Configuration(void)
{
    // 配置USART通信
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
        // 停止电机和舵机
        TIM_SetCompare1(TIM1, 0);
        TIM_SetCompare2(TIM1, 0);
        TIM_SetCompare3(TIM1, 0);
        TIM_SetCompare4(TIM1, 0);
        TIM_SetCompare2(TIM3, 0);
        break;

    case RUNNING:
        // 启动电机和舵机
        TIM_SetCompare1(TIM1, PWM_MAX_VALUE); // 电机1
        TIM_SetCompare2(TIM1, PWM_MAX_VALUE); // 电机2
        TIM_SetCompare3(TIM1, PWM_MAX_VALUE); // 电机3
        TIM_SetCompare4(TIM1, PWM_MAX_VALUE); // 舵机1
        TIM_SetCompare2(TIM3, PWM_MAX_VALUE); // 舵机2
        break;
    }
}

int main(void)
{
    TIM1_PWM_Init();            // 初始化PWM输出
    TIM2_Encoder_Init();        // 初始化编码器接口
    EXTI_Configuration();       // 配置外部中断
    TIM2_NVIC_Configuration();  // 配置TIM2中断
    USART_Configuration();      // 配置USART通信

    while (1)
    {
        // 接收到的数据存储
        char received_data;

        // 接收数据
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
        {
            received_data = USART_ReceiveData(USART1);

            // 解析接收到的命令
            switch (received_data)
            {
            case '0':
                motor1_state = STOPPED;
                break;
            case '1':
                motor1_state = RUNNING;
                break;
                // 可以继续添加更多的命令来控制不同的电机和舵机
            }

            // 控制电机状态
            Motor_Control(motor1_state);
        }
    }
}
