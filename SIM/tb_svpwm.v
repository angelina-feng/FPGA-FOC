
//--------------------------------------------------------------------------------------------------------
// Module  : tb_swpwm
// Type    : simulation, top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: testbench for cartesian2polar.sv and swpwm.sv
//--------------------------------------------------------------------------------------------------------

module tb_swpwm();


initial $dumpvars(1, tb_swpwm);
initial $dumpvars(1, u_svpwm);
         

reg rstn = 1'b0;
reg clk  = 1'b1;
always #(13563) clk = ~clk;   // 36.864MHz
initial begin repeat(4) @(posedge clk); rstn<=1'b1; end


reg         [11:0] theta = 0;

wire signed [15:0] x, y;

wire        [11:0] rho;
wire        [11:0] phi;

wire pwm_en, pwm_a, pwm_b, pwm_c;


// Here, the sincos module is simply utilized to generate a sine wave for the cartesian2polar module, solely for simulation purposes. 
In an FOC design, the sincos module is not used to provide input data to cartesian2polar; instead, it is called by park_tr. 
sincos u_sincos (
    .rstn         ( rstn       ),
    .clk          ( clk        ),
    .i_en         ( 1'b1       ),
    .i_theta      ( theta      ),   // input: θ, an increasing angle value
    .o_en         (            ),
    .o_sin        ( y          ),   // output: y, sine wave with amplitude ±16384
    .o_cos        ( x          )    // output: x, cosine wave with amplitude ±16384
);

cartesian2polar u_cartesian2polar (
    .rstn         ( rstn       ),
    .clk          ( clk        ),
    .i_en         ( 1'b1       ),
    .i_x          ( x / 16'sd5 ),  // input: cosine wave with amplitude ±3277
    .i_y          ( y / 16'sd5 ),  // input: sine wave with amplitude ±3277
    .o_en         (            ),
    .o_rho        ( rho        ),  // output: ρ, should be constantly equal to or approximately 3277
    .o_theta      ( phi        )   // output: φ, should be an angle value close to θ
);

svpwm u_svpwm (
    .rstn         ( rstn       ),
    .clk          ( clk        ),
    .v_amp        ( 9'd384     ),
    .v_rho        ( rho        ),  // input : ρ
    .v_theta      ( phi        ),  // input : φ
    .pwm_en       ( pwm_en     ),  // output
    .pwm_a        ( pwm_a      ),  // output
    .pwm_b        ( pwm_b      ),  // output
    .pwm_c        ( pwm_c      )   // output
);


integer i;

initial begin
    while(~rstn) @ (posedge clk);
    for(i=0; i<200; i=i+1) begin
        theta <= 25 * i;               // Let θ increase.
        repeat(2048) @ (posedge clk);
        $display("%d/200", i);
    end
    $finish;
end

endmodule
