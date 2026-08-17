
//--------------------------------------------------------------------------------------------------------
// fpga_top
// Type    : synthesizable, FPGA's top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: FOC usage example; serves as the top-level module for the FPGA project. 
// It controls the motor's tangential torque, alternating between clockwise and counter-clockwise directions, while allowing the current loop control tracking curve to be monitored via UART.
// Parameters: None
// Inputs/Outputs: See comments below for details.
//--------------------------------------------------------------------------------------------------------

module fpga_top (
    input  wire clk_50m, // Connect a 50MHz crystal oscillator.
    // ------- 3-phase PWM signal (including enable signal) -----------------------------------------------------------------------------------------------------
    output wire pwm_en,  // Enable signal shared by all three phases; when pwm_en=0, all six MOSFETs are turned off.
    output wire pwm_a,   // Phase A PWM signal. When 0, the low-side switch conducts; when 1, the high-side switch conducts.
    output wire pwm_b,   // Phase B
    output wire pwm_c,   // Phase C
    // ------- current sensing values ---------------------------------------------------------------------------------------
    input wire signed [15:0] csense_a,
    input wire signed [15:0] csense_b,
    input wire signed [15:0] csense_c,
    // ------- mechanical angle of the rotor ------------------------------------------------------------------------------------
    input wire hall_a,
    input wire hall_b,
    input wire hall_c
);


wire               rstn;       // Reset signal; initially 0, set to 1 after the PLL achieves lock.
wire               clk;         // Clock signal; frequency can be in the tens of MHz range. Control frequency = clock frequency / 2048. For example, if the clock frequency is 36.864 MHz, 
                                // the control frequency is 36.864 MHz / 2048 = 18 kHz. (Control frequency = 3-phase current sampling rate = PID algorithm control frequency = SVPWM duty cycle update frequency)

wire        [11:0] phi;         // Rotor mechanical angle φ read from the AS5600 magnetic encoder; value range: 0–4095. 0 corresponds to 0°; 1024 to 90°; 2048 to 180°; 3072 to 270°.

wire               en_idq;      // A high-level pulse indicates that new values ​​for id and iq have become available; en_idq generates a high-level pulse during each control cycle.
wire signed [15:0] id;          // Actual current value of the rotor d-axis (direct axis)
wire signed [15:0] iq;          // Actual current value of the rotor q-axis (quadrature axis); can be positive or negative (e.g., positive for counter-clockwise and negative for clockwise, or vice versa)
wire signed [15:0] id_aim;      // Target current value of the rotor d-axis (direct axis); can be positive or negative
reg  signed [15:0] iq_aim;      // Target current value of the rotor q-axis (quadrature axis); can be positive or negative (e.g., positive for counter-clockwise and negative for clockwise, or vice versa)

// PLL: Generates a 36.864 MHz clock (clk) from a 50 MHz clock (clk_50m).
// Note: This module is specific to Altera Cyclone IV FPGAs. For FPGAs from other manufacturers or series, please use the corresponding IP core or primitive (e.g., Xilinx Clock Wizard) to achieve the same functionality.wire [3:0] subwire0;
altpll u_altpll ( .inclk ( {1'b0, clk_50m} ), .clk ( {subwire0, clk} ), .locked ( rstn ),  .activeclock (),  .areset (1'b0), .clkbad (),  .clkena ({6{1'b1}}),  .clkloss (), .clkswitch (1'b0), .configupdate (1'b0), .enable0 (), .enable1 (),  .extclk (),  .extclkena ({4{1'b1}}), .fbin (1'b1), .fbmimicbidir (),  .fbout (), .fref (), .icdrclk (), .pfdena (1'b1), .phasecounterselect ({4{1'b1}}), .phasedone (), .phasestep (1'b1), .phaseupdown (1'b1),  .pllena (1'b1), .scanaclr (1'b0), .scanclk (1'b0), .scanclkena (1'b1), .scandata (1'b0), .scandataout (),  .scandone (), .scanread (1'b0), .scanwrite (1'b0), .sclkout0 (), .sclkout1 (), .vcooverrange (), .vcounderrange ());
defparam u_altpll.bandwidth_type = "AUTO", u_altpll.clk0_divide_by = 99,  u_altpll.clk0_duty_cycle = 50, u_altpll.clk0_multiply_by = 73, u_altpll.clk0_phase_shift = "0", u_altpll.compensate_clock = "CLK0", u_altpll.inclk0_input_frequency = 20000, u_altpll.intended_device_family = "Cyclone IV E",  u_altpll.lpm_hint = "CBX_MODULE_PREFIX=pll", u_altpll.lpm_type = "altpll",  u_altpll.operation_mode = "NORMAL",  u_altpll.pll_type = "AUTO", u_altpll.port_activeclock = "PORT_UNUSED",  u_altpll.port_areset = "PORT_UNUSED",  u_altpll.port_clkbad0 = "PORT_UNUSED",  u_altpll.port_clkbad1 = "PORT_UNUSED", u_altpll.port_clkloss = "PORT_UNUSED", u_altpll.port_clkswitch = "PORT_UNUSED", u_altpll.port_configupdate = "PORT_UNUSED", u_altpll.port_fbin = "PORT_UNUSED", u_altpll.port_inclk0 = "PORT_USED",  u_altpll.port_inclk1 = "PORT_UNUSED", u_altpll.port_locked = "PORT_USED", u_altpll.port_pfdena = "PORT_UNUSED",  u_altpll.port_phasecounterselect = "PORT_UNUSED", u_altpll.port_phasedone = "PORT_UNUSED", u_altpll.port_phasestep = "PORT_UNUSED", u_altpll.port_phaseupdown = "PORT_UNUSED",  u_altpll.port_pllena = "PORT_UNUSED", u_altpll.port_scanaclr = "PORT_UNUSED",  u_altpll.port_scanclk = "PORT_UNUSED", u_altpll.port_scanclkena = "PORT_UNUSED", u_altpll.port_scandata = "PORT_UNUSED", u_altpll.port_scandataout = "PORT_UNUSED", u_altpll.port_scandone = "PORT_UNUSED", u_altpll.port_scanread = "PORT_UNUSED", u_altpll.port_scanwrite = "PORT_UNUSED", u_altpll.port_clk0 = "PORT_USED", u_altpll.port_clk1 = "PORT_UNUSED", u_altpll.port_clk2 = "PORT_UNUSED",  u_altpll.port_clk3 = "PORT_UNUSED", u_altpll.port_clk4 = "PORT_UNUSED", u_altpll.port_clk5 = "PORT_UNUSED", u_altpll.port_clkena0 = "PORT_UNUSED",  u_altpll.port_clkena1 = "PORT_UNUSED", u_altpll.port_clkena2 = "PORT_UNUSED", u_altpll.port_clkena3 = "PORT_UNUSED",  u_altpll.port_clkena4 = "PORT_UNUSED", u_altpll.port_clkena5 = "PORT_UNUSED", u_altpll.port_extclk0 = "PORT_UNUSED", u_altpll.port_extclk1 = "PORT_UNUSED",  u_altpll.port_extclk2 = "PORT_UNUSED",  u_altpll.port_extclk3 = "PORT_UNUSED",  u_altpll.self_reset_on_loss_lock = "OFF",  u_altpll.width_clock = 5;
//assign rstn=1'b1; assign clk=clk_50m;
    

// FOC + SVPWM Module (refer to foc_top.sv for usage and operating principles)
foc_top #(
    .INIT_CYCLES  ( 16777216       ), // In this example, the clock (clk) frequency is 36.864 MHz and INIT_CYCLES is 16,777,216; thus, the initialization time is 16,777,216 / 36,864,000 = 0.45 seconds.
    .ANGLE_INV    ( 1'b0           ), // In this example, the angle sensor is not mounted in reverse (the A->B->C->A rotation direction matches the direction of increasing φ), so this parameter is set to 0.
    .POLE_PAIR    ( 8'd7           ), // The motor used in this example has 7 pole pairs.
    .MAX_AMP      ( 9'd384         ), // 384 / 512 = 0.75; this indicates that the maximum SVPWM amplitude is 75% of the maximum amplitude limit.
    .SAMPLE_DELAY ( 9'd120         )  // Sampling delay (range: 0–511). Since the three-phase MOSFETs require time to stabilize the current after turning on, a delay is needed between the moment all three low-side switches are turned on and the ADC sampling instant. This parameter specifies the delay duration in clock cycles; once the delay elapses, the module generates a high-level pulse on the sn_adc signal to indicate to the external ADC that it is ready to sample.
) u_foc_top (
    .rstn         ( rstn           ),
    .clk          ( clk            ),
    .Kp           ( 31'd300000     ), // P parameter for the current loop PID control algorithm
    .Ki           ( 31'd30000      ), // I parameter for the current loop PID control algorithm
    .phi          ( phi            ), // input: Angle sensor input (mechanical angle, denoted as φ); range: 0–4095. 0 corresponds to 0°; 1024 to 90°; 2048 to 180°; 3072 to 270°.
    .sn_adc       ( sn_adc         ), // output: Control signal for 3-phase current ADC sampling timing; a high-level pulse lasting one clock cycle is generated on sn_adc to trigger an ADC sampling operation.
    .en_adc       ( en_adc         ), // input: Valid signal for 3-phase current ADC sampling results; after sn_adc generates a high-level pulse, the external ADC samples the 3-phase currents. Upon completion of conversion, a high-level pulse lasting one cycle should be generated on en_adc, while the ADC conversion results are presented on the adc_a, adc_b, and adc_c signals.
    .adc_a        ( csense_a    ), // input: ADC sampling result for Phase A
    .adc_b        ( csense_b    ), // Phase B
    .adc_c        ( csense_c    ), // Phase C
    .pwm_en       ( pwm_en         ),
    .pwm_a        ( pwm_a          ),
    .pwm_b        ( pwm_b          ),
    .pwm_c        ( pwm_c          ),
    .en_idq       ( en_idq         ), // output: A high-level pulse indicates that new values ​​for id and iq are available; en_idq generates a high-level pulse during every control cycle
    .id           ( id             ), // output: Actual current value of the d-axis (direct axis); can be positive or negative
    .iq           ( iq             ), // output: Actual current value of the q-axis (quadrature axis); can be positive or negative (if positive represents counter-clockwise, then negative represents clockwise, and vice versa)
    .id_aim       ( id_aim         ), // input : Target current value of the d-axis (direct axis); can be positive or negative; generally set to 0 when field-weakening control is not used
    .iq_aim       ( iq_aim         ), // input : Target current value of the q-axis (quadrature axis); can be positive or negative (if positive represents counter-clockwise, then negative represents clockwise, and vice versa)
    .init_done    (                )  // output: Initialization completion signal. 0 before initialization finishes; 1 after initialization finishes (upon entering the FOC control state)
);


reg [23:0] cnt;
always @ (posedge clk or negedge rstn)   // This always block maintains a 24-bit incrementing counter
    if(~rstn)
        cnt <= 24'd0;
    else
        cnt <= cnt + 24'd1;


assign id_aim = $signed(16'd0);          // Set id_aim to a constant 0

always @ (posedge clk or negedge rstn)   // This always block causes iq_aim to alternate between +200 and -200,
    if(~rstn) begin                      // meaning the motor's tangential torque switches between clockwise and counter-clockwise
        iq_aim <= $signed(16'd0);
    end else begin
        if(cnt[23])
            iq_aim <=  $signed(16'd200); // Set iq_aim = +200
        else
            iq_aim <= -$signed(16'd200); // Set iq_aim = -200
    end

endmodule
