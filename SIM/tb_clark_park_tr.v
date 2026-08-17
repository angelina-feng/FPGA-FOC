//--------------------------------------------------------------------------------------------------------
// Module  : tb_clark_park_tr
// Type    : simulation, top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: testbench for sincos.sv, clark_tr.sv, park_tr.sv
//--------------------------------------------------------------------------------------------------------

module tb_clark_park_tr();


initial $dumpvars(1, tb_clark_park_tr);
         

reg rstn = 1'b0;
reg clk  = 1'b1;
always #(13563) clk = ~clk;   // 36.864MHz
initial begin repeat(4) @(posedge clk); rstn<=1'b1; end

reg                en_theta = 0;
reg         [11:0] theta = 0;       // Current electrical angle (denoted as ψ). Range: 0–4095. 0 corresponds to 0°; 1024 to 90°; 2048 to 180°; 3072 to 270°.

localparam  [11:0] PI_M2_D3 = (2*4096/3);     // (2/3)*π
localparam  [11:0] PI_D3    = (  4096/3);     // (1/3)*π

wire               en_iabc;
wire signed [15:0] ia, ib, ic;

wire               en_ialphabeta;
wire signed [15:0] ialpha, ibeta;

wire               en_idq;
wire signed [15:0] id;
wire signed [15:0] iq;

// Here, the sincos module is used merely to generate sine waves for clark_tr for simulation purposes. 
// In an FOC design, the sincos module is not used to provide input data to clark_tr; instead, it is called by park_tr.
sincos u1_sincos (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_theta                 ),
    .i_theta      ( theta + PI_M2_D3         ),   // Input: θ + (2/3)π
    .o_en         ( en_iabc                  ),
    .o_sin        ( ia                       ),   // Output: Ia (sine wave with amplitude ±16384 and initial phase (4/3)π)
    .o_cos        (                          )
);

sincos u2_sincos (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_theta                 ),
    .i_theta      ( theta + PI_D3            ),   // input: θ + (1/3)π
    .o_en         (                          ),
    .o_sin        ( ib                       ),   // output: Ib, sine wave with amplitude ±16384 and initial phase (2/3)π
    .o_cos        (                          )
);

sincos u3_sincos (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_theta                 ),
    .i_theta      ( theta                    ),   // input: θ
    .o_en         (                          ),
    .o_sin        ( ic                       ),   // output: Ic, sine wave with amplitude ±16384 and initial phase 0
    .o_cos        (                          )
);
// Clarke transform
clark_tr u_clark_tr (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_iabc                  ),
    .i_ia         ( ia / 16'sd2              ),  // input: Sine wave with amplitude ±8192 and initial phase (4/3)*π
    .i_ib         ( ib / 16'sd2              ),  // input: Sine wave with amplitude ±8192 and initial phase (2/3)*π
    .i_ic         ( ic / 16'sd2              ),  // input: Sine wave with amplitude ±8192 and initial phase 0
    .o_en         ( en_ialphabeta            ),
    .o_ialpha     ( ialpha                   ),  // output: Iα; should be a sine wave with initial phase (4/3)*π
    .o_ibeta      ( ibeta                    )   // output: Iβ; phase should lag Iα by (1/2)*π (i.e., orthogonal to Iα)
);

// Park transform
park_tr u_park_tr (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .psi          ( theta + 12'd512          ),  // input: θ + (1/4)*π
    .i_en         ( en_ialphabeta            ),
    .i_ialpha     ( ialpha                   ),  // input: Iα
    .i_ibeta      ( ibeta                    ),  // input: Iβ
    .o_en         ( en_idq                   ),
    .o_id         ( id                       ),  // output: Id; should become a constant value (Park transform converts rotor frame back to stator frame)
    .o_iq         ( iq                       )   // output: Iq; should become a constant value (Park transform converts rotor frame back to stator frame)
);

integer i;

initial begin
    while(~rstn) @ (posedge clk);
    for (i=0; i<1000; i=i+1) @ (posedge clk) begin
        en_theta <= 1'b1;
        theta <= theta + 12'd10;
        @ (posedge clk);
        en_theta <= 1'b0;
        repeat (9) @ (posedge clk);
    end
    $finish;
end

endmodule
