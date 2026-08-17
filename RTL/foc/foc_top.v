
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
    
    // -----------------------------------------------3-phase PWM signals (including enable signal) -----------------------------------------------------------------------------------------------------------------------------
    output wire               pwm_en,               // A shared enable signal for all three phases; when pwm_en = 0, all six MOSFETs are turned off.
    output wire               pwm_a,                // Phase A PWM signal. When the signal is 0, the lower bridge arm conducts; when it is 1, the upper bridge arm conducts.
    output wire               pwm_b,                // Phase B
    output wire               pwm_c,                // Phase C
    
    // ----------------------------------------------- d/q-axis (rotor-based Cartesian coordinate system) current monitoring ----------------------------------------------------------------------------------------------------
    output wire               en_idq,               // The appearance of a high-level pulse indicates that new values ​​for id and iq have become available; a high-level pulse is generated on en_idq during each control cycle.
    output wire signed [15:0] id,                   // The actual current value of the d-axis (direct axis) (abbreviated as Id), which can be positive or negative.
    output wire signed [15:0] iq,                   // The actual current value of the q-axis (quadrature axis), denoted simply as Iq, can be positive or negative (if positive represents counter-clockwise, then negative 
                                                    // represents clockwise, and vice versa).
    
    // ----------------------------------------------- Current control targets for the d/q axes (rotor-fixed coordinate system) ------------------------------------------------------------------------
    input  wire signed [15:0] id_aim,               // The target current value for the d-axis (direct axis)—abbreviated as Idaim—can be positive or negative; it is generally set to zero when field-weakening control is not used.
    input  wire signed [15:0] iq_aim,               // The target current value for the q-axis (denoted as Iqaim) can be positive or negative (if positive represents counter-clockwise, then negative represents clockwise, and vice versa).
    // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    output reg                init_done             // Initialization completion signal. 0 before initialization completes; 1 after initialization completes (upon entering FOC control mode).
);

reg         [31:0] init_cnt;
reg         [11:0] init_phi;      // Initial mechanical angle (abbreviated as Φ). This is the mechanical angle corresponding to an electrical angle of 0°; it is determined upon completion of initialization and is used for 
                                  // subsequent conversions between mechanical and electrical angles. The value range is 0–4095: 0 corresponds to 0°, 1024 to 90°, 2048 to 180°, and 3072 to 270°.。

reg         [11:0] psi;           // Current electrical angle (abbreviated as ψ). The value range is 0–4095: 0 corresponds to 0°, 1024 to 90°, 2048 to 180°, and 3072 to 270°.

reg                en_iabc;       // The currents on the three phases are valid; the generation of a high-level pulse indicates that Ia, Ib, and Ic have been updated.

reg  signed [15:0] ia, ib, ic;    // Currents in the three phases. A positive value indicates current flowing from the half-bridge into the motor, while a negative value indicates current flowing from the motor into the half-bridge. 
                                  // ia is the current in phase A (abbreviated as Ia), ib is the current in phase B (abbreviated as Ib), and ic is the current in phase C (abbreviated as Ic).

wire               en_ialphabeta; // When a high-level pulse is generated, the valid current vector signal on the α/β axes (stator Cartesian coordinate system) indicates that Iα and Iβ have been updated.

wire signed [15:0] ialpha, ibeta; // Current vector in the α/β-axis (stator Cartesian coordinate system). `ialpha` is the component along the α-axis (abbreviated as Iα), and `ibeta` is the component along the β-axis (abbreviated as Iβ).

wire signed [15:0] vd, vq;        // The voltage vectors on the d/q axes (rotor Cartesian coordinate system) are the values ​​output by the PID algorithm. Vd is the voltage component on the d-axis, and Vq is the voltage component on the q-axis.

wire        [11:0] vr_rho;        // The magnitude of the voltage vector in the rotor polar coordinate system (denoted as Vrρ) is derived by transforming Vd and Vq into the polar coordinate system; specifically, Vrρ = √(Vd² + Vq²).

wire        [11:0] vr_theta;      // The angle of the voltage vector in the rotor polar coordinate system (denoted as Vrθ) is derived by converting vd and vq into polar coordinates; specifically, Vrθ = arctan(Vq/Vd). 
                                  // The value range is 0 to 4095, where 0 corresponds to 0°, 1024 to 90°, 2048 to 180°, and 3072 to 270°.

reg         [11:0] vs_rho;        // The magnitude of the voltage vector in the stator polar coordinate system (denoted as Vsρ) is obtained by applying a rotational transformation to Vrρ; 
                                  // due to the rotational invariance of the magnitude, Vsρ is effectively equal to Vrρ. Through the SVPWM module, Vsρ and Vsθ can be used to generate three-phase PWM signals.

reg         [11:0] vs_theta;      // The angle of the voltage vector in the stator polar coordinate system (denoted as Vsθ) is obtained by applying a rotation transformation to Vrθ; since the rotor polar coordinate system is derived by 
                                  // rotating the stator polar coordinate system by ψ, it follows that Vsθ = Vrθ + ψ. The SVPWM module uses Vsρ and Vsθ to generate three-phase PWM signals. 
                                  // The value range for Vsθ is 0 to 4095, where 0 corresponds to 0°, 1024 to 90°, 2048 to 180°, and 3072 to 270°.



// Description: This `always` block is responsible for calculating the electrical angle ψ from the mechanical angle φ.
// Parameter: Pole pair number N (parameter POLE_PAIR)
// Input: Mechanical angle φ
//        Initial mechanical angle Φ
// Output: Electrical angle ψ
// Calculation formula: ψ = N * (φ - Φ) (if the direction of rotation A→B→C→A is the same as the direction in which φ increases)
//         ：Or ψ = -N * (φ - Φ)    (if the direction of rotation A→B→C→A is opposite to the direction in which φ increases—i.e., the angle sensor is installed in reverse)
// Output update: Whenever φ changes, ψ changes immediately in the next cycle.
generate if(ANGLE_INV) begin                              // If the angle sensor is installed backwards
    always @ (posedge clk or negedge init_done) 
        if(~init_done)
            psi <= 0;
        else
            psi <= {4'h0, POLE_PAIR} * (init_phi - phi);  // ψ = -N * (φ - Φ)
end else begin                                            // If the angle sensor is not installed backwards
    always @ (posedge clk or negedge init_done) 
        if(~init_done)
            psi <= 0;
        else
            psi <= {4'h0, POLE_PAIR} * (phi - init_phi);  // ψ =  N * (φ - Φ)
end endgenerate



// Description: Based on Kirchhoff's Current Law (KCL), this `always` block calculates the three-phase currents (Ia, Ib, Ic) by subtracting offset values ​​from the raw ADC readings (ADCa, ADCb, ADCc).
// Input: Raw ADC values ​​ADCa, ADCb, ADCc
// Output: Phase currents Ia, Ib, Ic
// Calculation formula：Ia = ADCb + ADCc - 2*ADCa
//           Ib = ADCa + ADCc - 2*ADCb
//           Ic = ADCa + ADCb - 2*ADCc
// Output update: The output is updated after each ADC sampling completion (i.e., each time a high-level pulse is generated on `en_adc`); thus, the update frequency equals the control cycle. 
                  Following the update, `en_iabc` generates a high-level pulse lasting one clock cycle.
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



// Description : This module performs the Clark transform, calculating the α/β-axis current vectors (in the stator Cartesian coordinate system) based on three-phase currents.
// Inputs      : Phase currents Ia, Ib, Ic
// Outputs     : α/β-axis current vectors Iα, Iβ
// Formulas    : Iα = 2 * Ia - Ib - Ic
//               Iβ = √3 * (Ib - Ic)
// Output Update: Iα and Iβ are updated a few cycles after a high-level pulse is generated on en_iabc; simultaneously, a high-level pulse lasting one clock cycle is generated on en_ialphabeta (i.e., update frequency = control cycle).
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



// Description: This module performs the Park transformation, calculating d/q-axis current vectors (rotor-based Cartesian coordinates) from α/β-axis current vectors (stator-based Cartesian coordinates).
// Inputs:      Electrical angle ψ
//              α/β-axis current vectors Iα, Iβ
// Outputs:     d/q-axis current vectors Id, Iq
// Formulas:    Id = Iα * cosψ + Iβ * sinψ;
//              Iq = Iβ * cosψ - Iα * sinψ;
// Output Update: Id and Iq are updated a few cycles after a high-level pulse occurs on en_ialphabeta; 
                  simultaneously, en_idq generates a high-level pulse lasting one clock cycle (i.e., update frequency = control cycle).
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



// Description: This module performs PID control for Id (the d-axis component of the current vector). 
                Based on the target value (id_aim) and the actual value (id) of Id, it calculates the control variable Vd (the d-axis component of the voltage vector).
// Input: Actual value of the d-axis current component (id)
//        Target value of the d-axis current component (id_aim)
// Output: d-axis voltage component (vd)
// Output update: Vd is updated a certain number of cycles after each high-level pulse is generated by en_idq; that is, the update frequency equals the control cycle.
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



// Introduction:    This module performs PID control for Iq (the q-axis component of the current vector); 
                   based on the target value (iq_aim) and the actual value (iq) of Iq, it calculates the control variable Vq (the q-axis component of the voltage vector).
// Input    ：Actual value of the current vector component on the q-axis (iq)
//          Target value of the current vector component on the q-axis (iq_aim)
// Output: Component of the voltage vector on the q-axis (vq)
// Principle: PID control (in practice, there is no D; only P and I are used).
// Output update: Vq is updated a certain number of cycles after each high-level pulse is generated by en_idq; that is, the update frequency equals the control cycle.
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



// Introduction: This module is used to transform the voltage vector from the rotor Cartesian coordinate system (Vd, Vq) to the rotor polar coordinate system (Vrρ, Vrθ).
// Input: Component of the voltage vector on the d-axis of the rotor's rectangular coordinate system (Vd)
//        The component of the voltage vector on the q-axis of the rotor's rectangular coordinate system (Vq)
// Output: Magnitude of the voltage vector in the rotor polar coordinate system (Vrρ)
// Principle: The angle of the voltage vector in the rotor polar coordinate system (Vrθ)
// Output update: Vrρ and Vrθ are updated after a certain number of cycles in which Vd or Vq change; update frequency = control cycle.
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



// Introduction    ：This `always` block is used for initialization and the inverse Park transform.
//           I. Initialization: Perform initial mechanical angle calibration. First, set Vsρ to its maximum value and Vsθ to 0; the rotor will naturally rotate to the position where the electrical angle ψ is 0. 
                                Then, record the current mechanical angle φ as the initial mechanical angle Φ. Subsequently, the electrical angle ψ can be calculated using the formula ψ = N * (φ - Φ).
                                    
//           II. Inverse Park Transform: After initialization, continuously transform the voltage vector from the rotor polar coordinate system (Vrρ, Vrθ) to the stator polar coordinate system (Vsρ, Vsθ).
// Input    ：φ，Vrρ, Vrθ
// Output    ：Φ，Vsρ, Vsθ，init_done
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {vs_rho, vs_theta} <= 0;
        init_cnt <= 0;
        init_phi <= 0;
        init_done <= 1'b0;
    end else begin
        if(init_cnt<=INIT_CYCLES) begin      // If the counter variable init_cnt is less than or equal to INIT_CYCLES, then initialization is not complete.
            vs_rho <= 12'd4095;              //    Maximize Vsρ during the initialization phase.
            vs_theta <= 12'd0;               //    Set Vsθ = 0 during the initialization phase.
            init_cnt <= init_cnt + 1;
            if(init_cnt==INIT_CYCLES) begin  // If the counter variable `init_cnt` equals `INIT_CYCLES`, it indicates that initialization is nearing completion.
                init_phi <= phi;             //    Record the current mechanical angle φ as the initial mechanical angle Φ.
                init_done <= 1'b1;           //    Set init_done to 1 to indicate that initialization is complete.
            end
        end else begin                       // If the counter variable `init_cnt` is greater than `INIT_CYCLES`, initialization is complete.
            vs_rho <= vr_rho;                //    Inverse Park transformation. Due to the rotational invariance of the magnitude, Vsρ = Vrρ.
            vs_theta <= vr_theta + psi;      //    Inverse Park transformation. Since the rotor coordinate system is obtained by rotating the stator coordinate system by ψ, Vsθ = Vrθ + ψ.
        end
    end



// Introduction: This module is a 7-segment SVPWM generator used to generate 3-phase PWM signals.
// Input: Voltage vectors Vsρ and Vsθ in the stator polar coordinate system.
// Output: PWM enable signal pwm_en
//         3-phase PWM signals pwm_a, pwm_b, pwm_c
// Note: The frequency of the PWM generated by this module is the clock frequency divided by 2048. For example, if the clock frequency is 36.864 MHz, the PWM frequency is 36.864 MHz / 2048 = 18 kHz.
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



// Introduction: This module is used to control the sampling timing of the phase current detection ADC.
// Inputs: 3-phase PWM signals pwm_a, pwm_b, pwm_c
// Output: 3-phase current ADC sampling timing control signal sn_adc
// Principle: This module detects the moment when pwm_a, pwm_b, and pwm_c are all at a low level; 
              after a delay of SAMPLE_DELAY clock cycles, it generates a high-level pulse lasting one clock cycle on the sn_adc signal.
hold_detect #(
    .SAMPLE_DELAY ( SAMPLE_DELAY             )
) u_adc_sn_ctrl (
    .rstn         ( init_done                ),
    .clk          ( clk                      ),
    .in           ( ~pwm_a & ~pwm_b & ~pwm_c ),  // input : Equals 1 when pwm_a, pwm_b, and pwm_c are all low; otherwise, equals 0.
    .out          ( sn_adc                   )   // output: If the input signal is 1 and remains so for SAMPLE_DELAY cycles, a high-level pulse lasting one cycle is generated on sn_adc.
);


endmodule

