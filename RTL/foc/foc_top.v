
//--------------------------------------------------------------------------------------------------------
// Module: foc_top
// Type    : synthesizable, IP's top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function：FOC algorithm (current loop only) + SVPWM
// Parameters: See notes below for details.
// Input/Output: See notes below for details.
//--------------------------------------------------------------------------------------------------------

module foc_top #(
    // ----------------------------------------------- Module Parameters----------------------------------------------------------------------------------------------------------------------------------
    parameter        INIT_CYCLES  = 16777216,       // This determines the number of clock (clk) cycles for the initialization step, with a valid range of 1 to 4,294,967,294. The value must not be too low, 
                                                    // as sufficient time is required for the rotor to return to an electrical angle of 0. For example, if the clock (clk) frequency is 36.864 MHz and INIT_CYCLES 
                                                    // is 16,777,216, the initialization time is 16,777,216 / 36,864,000 = 0.45 seconds. 
    
    parameter        ANGLE_INV    = 0,              // If the angle sensor is not installed in reverse (i.e., the direction of rotation A→B→C→A matches the direction in which φ increases), this parameter should 
                                                    // be set to 0. If the angle sensor is installed in reverse (i.e., the direction of rotation A→B→C→A is opposite to the direction in which φ increases), this 
                                                    // parameter should be set to 1.
    
    parameter [ 7:0] POLE_PAIR    = 8'd7,           // Motor pole-pair count (abbreviated as N): ranges from 1 to 255, determined by the motor model. (Electrical angle ψ = pole-pair count N × mechanical angle φ)
    
    parameter [ 8:0] MAX_AMP      = 9'd384,         // The maximum amplitude for SVPWM ranges from 1 to 511; a lower value results in a lower maximum motor torque. However, given the use of a three-phase lower-leg 
                                                    // resistor current sampling method, the value cannot be excessively high, as sufficient continuous conduction time for the three lower-leg switches is required for 
                                                    // the ADC to perform sampling.  
    
    parameter [ 8:0] SAMPLE_DELAY = 9'd120          // Sampling delay; the value ranges from 0 to 511. Since the three-phase MOSFETs require a certain amount of time to transition from the onset of conduction to a stable 
                                                    // current state, a delay is necessary between the moment all three low-side switches turn on and the ADC sampling instant. This parameter specifies the duration 
                                                    // of the delay in clock cycles; upon completion of the delay, the module generates a high-level pulse on the `sn_adc` signal to indicate to the external ADC that 
                                                    // it is ready to sample.
) (
    // ----------------------------------------------- Driving Clock and Reset ---------------------------------------------------------------------------------------------------------------------------------------------
    input  wire               rstn,                 // The reset signal should first be pulled low to reset the module, and then held high to allow the module to operate normally.
    
    input  wire               clk,                  // The clock signal frequency can be in the range of tens of MHz. Control frequency = Clock frequency / 2048. For example, if the clock frequency is 36.864 MHz, the control 
                                                    // frequency is 36.864 MHz / 2048 = 18 kHz. (Control frequency = 3-phase current sampling rate = PID algorithm control frequency = SVPWM duty cycle update frequency)    
    
    // ----------------------------------------------- PI parameters ----------------------------------------------------------------------------------------------------------------------------------------------------
    input  wire        [30:0] Kp,
    input  wire        [30:0] Ki,
    
    // ----------------------------------------------- Angle sensor input signal -----------------------------------------------------------------------------------------------------------------------------------------
    input  wire        [11:0] phi,                  // Angle sensor input (mechanical angle, abbreviated as φ) with a value range of 0–4095: 0 corresponds to 0°, 1024 to 90°, 2048 to 180°, and 3072 to 270°.
  
    // --------------------------------3-phase current ADC sampling timing control signals and sampling result input signals --------------------------------------------------------------------------------------
    output wire               sn_adc,               // This is the control signal for the 3-phase current ADC sampling instant; when a sample needs to be taken, a high-level pulse lasting one clock cycle is generated on the 
                                                    // `sn_adc` signal to indicate that the ADC should perform a sampling operation.。
    
    input  wire               en_adc,               // Regarding the valid signal for the 3-phase current ADC sampling results: after `sn_adc` generates a high-level pulse, the external ADC begins sampling the 3-phase currents. 
                                                    // Upon completion of the conversion, a high-level pulse lasting one clock cycle should be generated on the `en_adc` signal, while the ADC conversion results are output on the 
                                                    // `adc_a`, `adc_b`, and `adc_c` signals.
    
    input  wire        [11:0] adc_a, adc_b, adc_c,  // 3-phase current ADC sampling results (denoted as ADCa, ADCb, and ADCc), with a value range of 0 to 4095.
    
    // ----------------------------------------------- 3相 PWM 信号，（包含使能信号） -----------------------------------------------------------------------------------------------------------------------------
    output wire               pwm_en,               // 3相共用的使能信号，当 pwm_en=0 时，6个MOS管全部关断。
    output wire               pwm_a,                // A相PWM信号。当 =0 时。下桥臂导通；当 =1 时，上桥臂导通
    output wire               pwm_b,                // B相PWM信号。当 =0 时。下桥臂导通；当 =1 时，上桥臂导通
    output wire               pwm_c,                // C相PWM信号。当 =0 时。下桥臂导通；当 =1 时，上桥臂导通
    // ----------------------------------------------- d/q轴（转子直角坐标系）的电流监测 --------------------------------------------------------------------------------------------------------------------------
    output wire               en_idq,               // 出现高电平脉冲时说明 id 和 iq 出现了新值，每个控制周期 en_idq 会产生一个高电平脉冲
    output wire signed [15:0] id,                   // d 轴（直轴）的实际电流值（简记为 Id），可正可负
    output wire signed [15:0] iq,                   // q 轴（交轴）的实际电流值（简记为 Iq），可正可负（若正代表逆时针，则负代表顺时针，反之亦然）
    // ----------------------------------------------- d/q轴（转子直角坐标系）的电流控制目标 ----------------------------------------------------------------------------------------------------------------------
    input  wire signed [15:0] id_aim,               // d 轴（直轴）的目标电流值（简记为 Idaim），可正可负，在不使用弱磁控制的情况下一般设为0
    input  wire signed [15:0] iq_aim,               // q 轴（直轴）的目标电流值（简记为 Iqaim），可正可负（若正代表逆时针，则负代表顺时针，反之亦然）
    // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    output reg                init_done             // 初始化结束信号。在初始化结束前=0，在初始化结束后（进入FOC控制状态）=1
);

reg         [31:0] init_cnt;
reg         [11:0] init_phi;      // 初始机械角度（简记为 Φ）。即电角度=0时对应的机械角度，在初始化结束时被确定，用来在之后进行机械角度到电角度的转换。取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。

reg         [11:0] psi;           // 当前电角度（简记为 ψ）。取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。

reg                en_iabc;       // 3相上的电流有效，在产生高电平脉冲时，说明 Ia, Ib, Ic 发生更新
reg  signed [15:0] ia, ib, ic;    // 3相上的电流。为正代表电流从半桥流入电机，为负代表电流从电机流入半桥。ia 是A相的电流（简记为Ia），ib 是B相的电流（简记为Ib），ic 是C相的电流（简记为Ic）

wire               en_ialphabeta; // α/β轴（定子直角坐标系）上的电流矢量有效信号，在产生高电平脉冲时，说明 Iα, Iβ 发生更新
wire signed [15:0] ialpha, ibeta; // α/β轴（定子直角坐标系）上的电流矢量。ialpha 是 α 轴的分量（简记为 Iα），ibeta 是 β 轴的分量（简记为 Iβ）

wire signed [15:0] vd, vq;        // d/q轴（转子直角坐标系）上的电压矢量，是 PID 算法输出的值。Vd 是 d 轴 上的电压分量，Vq 是 q 轴上的电压分量

wire        [11:0] vr_rho;        // 转子极坐标系上的电压矢量的幅值（简记为 Vrρ ），由 Vd 和 Vq 转换到极坐标系得来，具体地讲，Vrρ = √(Vd^2+Vq^2)
wire        [11:0] vr_theta;      // 转子极坐标系上的电压矢量的角度（简记为 Vrθ ），由 vd 和 vq 转换到极坐标系得来，具体地讲，Vrθ = arctan(Vq/Vd) 。取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。

reg         [11:0] vs_rho;        // 定子极坐标系上的电压矢量的幅值（简记为 Vsρ ），由 Vrρ 做旋转变换得来，由于幅值的旋转不变性，实际上 Vsρ = Vrρ。 通过 SVPWM 模块，Vsρ 和 Vsθ 可以产生 3 相 PWM 信号
reg         [11:0] vs_theta;      // 定子极坐标系上的电压矢量的角度（简记为 Vsθ ），由 Vrθ 做旋转变换得来，由于转子极坐标系是定子极坐标系旋转 ψ 得来，所以 Vsθ = Vrθ + ψ。 通过 SVPWM 模块，Vsρ 和 Vsθ 可以产生 3 相 PWM 信号。 Vsθ取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。



// 简介    ：该 always 块负责从机械角度 φ 算出电角度 ψ
// 参数    ：极对数 N （参数POLE_PAIR）
// 输入    ：机械角度 φ
//           初始机械角度 Φ
// 输出    ：电角度 ψ
// 计算公式：   ψ =  N * (φ - Φ)    （若 A->B->C->A 的旋转方向与 φ 增大的方向相同）
//         ：或 ψ = -N * (φ - Φ)    （若 A->B->C->A 的旋转方向与 φ 增大的方向相反，即角度传感器装反了）
// 输出更新：只要 φ 改变，ψ 就在下一周期立即改变
generate if(ANGLE_INV) begin                              // 如果角度传感器装反了
    always @ (posedge clk or negedge init_done) 
        if(~init_done)
            psi <= 0;
        else
            psi <= {4'h0, POLE_PAIR} * (init_phi - phi);  // ψ = -N * (φ - Φ)
end else begin                                            // 如果角度传感器没装反
    always @ (posedge clk or negedge init_done) 
        if(~init_done)
            psi <= 0;
        else
            psi <= {4'h0, POLE_PAIR} * (phi - init_phi);  // ψ =  N * (φ - Φ)
end endgenerate



// 简介    ：该 always 块根据基尔霍夫电流定律(KCL)在 ADC 原始值 (ADCa, ADCb, ADCc) 上减去偏移值，计算出 3 相电流值 Ia, Ib, Ic
// 输入    ：ADC 原始值 ADCa, ADCb, ADCc
// 输出    ：相电流 Ia, Ib, Ic
// 计算公式：Ia = ADCb + ADCc - 2*ADCa
//           Ib = ADCa + ADCc - 2*ADCb
//           Ic = ADCa + ADCb - 2*ADCc
// 输出更新：ADC 每采样完成一次（即en_adc每产生一次高电平脉冲）后更新一次，即更新频率 = 控制周期，更新后 en_iabc 产生一个时钟周期的高电平脉冲
always @ (posedge clk or negedge init_done)
    if(~init_done) begin
        {en_iabc, ia, ib, ic} <= 0;
    end else begin
        en_iabc <= en_adc;
        if(en_adc) begin
            ia <= $signed( {4'b0, adc_b} + {4'b0, adc_c} - {3'b0, adc_a, 1'b0} );   // Ia = ADCb + ADCc - 2*ADCa
            ib <= $signed( {4'b0, adc_a} + {4'b0, adc_c} - {3'b0, adc_b, 1'b0} );   // Ib = ADCa + ADCc - 2*ADCb
            ic <= $signed( {4'b0, adc_a} + {4'b0, adc_b} - {3'b0, adc_c, 1'b0} );   // Ic = ADCa + ADCb - 2*ADCc
        end
    end



// 简介    ：该模块用于进行 clark 变换，根据 3 相电流计算 α/β 轴（定子直角坐标系）的电流矢量
// 输入    ：相电流 Ia, Ib, Ic
// 输出    ：α/β 轴的电流矢量 Iα, Iβ
// 计算公式：Iα = 2 * Ia - Ib - Ic
//           Iβ = √3 * (Ib - Ic)
// 输出更新：en_iabc 每产生一个高电平脉冲后的若干周期后 Iα, Iβ 更新，同时 en_ialphabeta 产生一个时钟周期的高电平脉冲，即更新频率 = 控制周期
clark_tr u_clark_tr (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .i_en         ( en_iabc                  ),
    .i_ia         ( ia                       ),  // input : Ia
    .i_ib         ( ib                       ),  // input : Ib
    .i_ic         ( ic                       ),  // input : Ic
    .o_en         ( en_ialphabeta            ),
    .o_ialpha     ( ialpha                   ),  // output: Iα
    .o_ibeta      ( ibeta                    )   // output: Iβ
);



// 简介    ：该模块用于进行 park 变换，根据 α/β 轴（定子直角坐标系）的电流矢量 计算 d/q 轴（转子直角坐标系）的电流矢量
// 输入    ：电角度 ψ
//           α/β 轴的电流矢量 Iα, Iβ
// 输出    ：d/q 轴的电流矢量 Id, Iq
// 计算公式：Id = Iα * cosψ + Iβ * sinψ;
//           Iq = Iβ * cosψ - Iα * sinψ;
// 输出更新：en_ialphabeta 每产生一个高电平脉冲后的若干周期后 Id, Iq 更新，同时 en_idq 产生一个时钟周期的高电平脉冲，即更新频率 = 控制周期
park_tr u_park_tr (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .psi          ( psi                      ),  // input : ψ
    .i_en         ( en_ialphabeta            ),
    .i_ialpha     ( ialpha                   ),  // input : Iα
    .i_ibeta      ( ibeta                    ),  // input : Iβ
    .o_en         ( en_idq                   ),
    .o_id         ( id                       ),  // output: Id
    .o_iq         ( iq                       )   // output: Iq
);



// 简介    ：该模块用于进行 Id (电流矢量在d轴的分量) 的 PID 控制，根据 Id 的目标值（id_aim）和 Id 的实际值（id），算出执行变量 Vd（电压矢量在d轴上的分量）
// 输入    ：电流矢量在d轴的分量的实际值（id）
//           电流矢量在d轴的分量的目标值（id_aim）
// 输出    ：电压矢量在d轴上的分量（vd）
// 原理    ：PID 控制（实际上没有D，只有P和I）
// 输出更新：en_idq 每产生一个高电平脉冲后的若干周期后 Vd 更新，即更新频率 = 控制周期
pi_controller u_id_pi (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .i_en         ( en_idq                   ),
    .i_Kp         ( Kp                       ),
    .i_Ki         ( Ki                       ),
    .i_aim        ( id_aim                   ),  // input : Idaim
    .i_real       ( id                       ),  // input : Id
    .o_en         (                          ),
    .o_value      ( vd                       )   // output: Vd
);



// 简介    ：该模块用于进行 Iq (电流矢量在q轴的分量) 的 PID 控制，根据 Iq 的目标值（iq_aim）和 Iq 的实际值（iq），算出执行变量 Vq（电压矢量在d轴上的分量）
// 输入    ：电流矢量在q轴的分量的实际值（iq）
//           电流矢量在q轴的分量的目标值（iq_aim）
// 输出    ：电压矢量在q轴上的分量（vq）
// 原理    ：PID 控制（实际上没有D，只有P和I）
// 输出更新：en_idq 每产生一个高电平脉冲后的若干周期后 Vq 更新，即更新频率 = 控制周期
pi_controller u_iq_pi (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .i_en         ( en_idq                   ),
    .i_Kp         ( Kp                       ),
    .i_Ki         ( Ki                       ),
    .i_aim        ( iq_aim                   ),  // input : Iqaim
    .i_real       ( iq                       ),  // input : Iq
    .o_en         (                          ),
    .o_value      ( vq                       )   // output: Vq
);



// 简介    ：该模块用于把电压矢量从转子直角坐标系 (Vd, Vq) 变换到转子极坐标系 (Vrρ, Vrθ)
// 输入    ：电压矢量在转子直角坐标系的 d 轴上的分量（Vd）
//           电压矢量在转子直角坐标系的 q 轴上的分量（Vq）
// 输出    ：电压矢量在转子极坐标系上的幅值（Vrρ）
// 原理    ：电压矢量在转子极坐标系上的角度（Vrθ）
// 输出更新：Vd, Vq 每产生变化的若干周期后 Vrρ 和 Vrθ 更新，更新频率 = 控制周期
cartesian2polar u_cartesian2polar (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .i_en         ( 1'b1                     ),
    .i_x          ( vd                       ),  // input : Vd
    .i_y          ( vq                       ),  // input : Vq
    .o_en         (                          ),
    .o_rho        ( vr_rho                   ),  // output: Vrρ
    .o_theta      ( vr_theta                 )   // output: Vrθ
);



// 简介    ：该 always 块用于进行初始化 和 反park变换
//           一、初始化： 进行初始机械角度标定。首先令 Vsρ 取最大，Vsθ=0，则转子自然会转到电角度 ψ=0 的地方。然后记录下此时的机械角度 φ 作为初始机械角度 Φ 。则之后就可以用公式 ψ = N * (φ - Φ) 计算电角度。
//           二、反park变换： 初始化完成后，持续地把电压矢量从转子极坐标系 (Vrρ, Vrθ) 变换到 定子极坐标系 (Vsρ, Vsθ)
// 输入    ：φ，Vrρ, Vrθ
// 输出    ：Φ，Vsρ, Vsθ，init_done
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {vs_rho, vs_theta} <= 0;
        init_cnt <= 0;
        init_phi <= 0;
        init_done <= 1'b0;
    end else begin
        if(init_cnt<=INIT_CYCLES) begin      // 若 init_cnt 计数变量 <= INIT_CYCLES ，则初始化未完成
            vs_rho <= 12'd4095;              //    初始化阶段令 Vsρ 取最大
            vs_theta <= 12'd0;               //    初始化阶段令 Vsθ = 0
            init_cnt <= init_cnt + 1;
            if(init_cnt==INIT_CYCLES) begin  // 若 init_cnt 计数变量 == INIT_CYCLES , 说明初始化即将完成
                init_phi <= phi;             //    记录当前机械角度φ 作为初始机械角度 Φ
                init_done <= 1'b1;           //    令 init_done = 1 ，指示初始化结束
            end
        end else begin                       // 若 init_cnt 计数变量 > INIT_CYCLES ，则初始化完成
            vs_rho <= vr_rho;                //    反park变换。由于幅值的旋转不变性，Vsρ = Vrρ
            vs_theta <= vr_theta + psi;      //    反park变换。由于转子极坐标系是定子极坐标系旋转 ψ 得来，所以 Vsθ = Vrθ + ψ
        end
    end



// 简介    ：该模块是7段式 SVPWM 发生器，用于生成 3 相上的 PWM 信号。
// 输入    ：定子极坐标系下的电压矢量 Vsρ, Vsθ
// 输出    ：PWM使能信号 pwm_en
//           3相PWM信号 pwm_a, pwm_b, pwm_c
// 说明    ：该模块产生的 PWM 的频率是 clk 频率 / 2048。例如 clk 为 36.864MHz ，则 PWM 的频率为 36.864MHz / 2048 = 18kHz
svpwm u_svpwm (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .v_amp        ( MAX_AMP                  ),
    .v_rho        ( vs_rho                   ),  // input : Vsρ
    .v_theta      ( vs_theta                 ),  // input : Vsθ
    .pwm_en       ( pwm_en                   ),  // output
    .pwm_a        ( pwm_a                    ),  // output
    .pwm_b        ( pwm_b                    ),  // output
    .pwm_c        ( pwm_c                    )   // output
);



// 简介    ：该模块用于控制相电流检测 ADC 的采样时机
// 输入    ：3相PWM信号 pwm_a, pwm_b, pwm_c
// 输出    ：3相电流 ADC 采样时刻控制信号 sn_adc
// 原理    ：该模块检测 pwm_a, pwm_b, pwm_c 均为低电平的时刻，并延迟 SAMPLE_DELAY 的时钟周期，在 sn_adc 信号上产生一个时钟周期的高电平。
hold_detect #(
    .SAMPLE_DELAY ( SAMPLE_DELAY             )
) u_adc_sn_ctrl (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .in           ( ~pwm_a & ~pwm_b & ~pwm_c ),  // input : 当 pwm_a, pwm_b, pwm_c 均为低电平时=1，否则=0
    .out          ( sn_adc                   )   // output: 若输入信号=1并保持 SAMPLE_DELAY 个周期，则 sn_adc 上产生1个周期的高电平脉冲
);


endmodule

