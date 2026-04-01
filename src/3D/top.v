`timescale 1ns / 1ps
// =============================================================================
// PHASE 2 - FOV sweep + player rotation
// -----------------------------------------------------------------------------
// Requires three hex files in your Vivado project source directory:
//   sin_table.hex   - 1024 lines, 4-hex-digit signed Q8.8
//                     sin_table[i] = round(sin(2*pi*i/1024) * 256)
//   col_angle.hex   - 640 lines, 3-hex-digit signed 10-bit
//                     col_angle[c] = angle offset for screen column c (0..639)
//   rcp_table.hex   - 256 lines, 3-hex-digit
//                     rcp_table[i] = strip height in pixels for Q4.4 distance i
//
// =============================================================================
module top (
    input  wire        clk,
    input  wire        BTNL,
    input  wire        BTNR,
    input  wire        BTNU,
    input  wire        BTND,
    output reg         Hsync,
    output reg         Vsync,
    output reg  [3:0]  Red,
    output reg  [3:0]  Green,
    output reg  [3:0]  Blue
);

// ── Power-on reset ────────────────────────────────────────────────────────────
reg [3:0] rst_sr = 4'b1111;
wire rst = rst_sr[0];
always @(posedge clk) rst_sr <= {1'b0, rst_sr[3:1]};

// ── 25 MHz pixel tick ─────────────────────────────────────────────────────────
reg [1:0] clk_div = 0;
wire pixel_tick = (clk_div == 2'd0);
always @(posedge clk) clk_div <= clk_div + 1;

// ── VGA counters ──────────────────────────────────────────────────────────────
reg [9:0] hcnt = 0, vcnt = 0;
always @(posedge clk) begin
    if (pixel_tick) begin
        if (hcnt == 799) begin
            hcnt <= 0;
            vcnt <= (vcnt == 524) ? 0 : vcnt + 1;
        end else
            hcnt <= hcnt + 1;
    end
end

wire vsync_active = (vcnt >= 490) && (vcnt < 492);
reg  vsync_r = 0;
always @(posedge clk) vsync_r <= vsync_active;

wire video_on = (hcnt < 640) && (vcnt < 480);
always @(posedge clk) begin
    Hsync <= ~((hcnt >= 656) && (hcnt < 752));
    Vsync <= ~vsync_active;
end

// ── Button debouncers ─────────────────────────────────────────────────────────
reg [1:0] sync_l=0, sync_r=0, sync_u=0, sync_d=0;
always @(posedge clk) begin
    sync_l <= {sync_l[0], BTNL};
    sync_r <= {sync_r[0], BTNR};
    sync_u <= {sync_u[0], BTNU};
    sync_d <= {sync_d[0], BTND};
end
reg [19:0] db_cnt_l=0, db_cnt_r=0, db_cnt_u=0, db_cnt_d=0;
reg btn_l=0, btn_r=0, btn_u=0, btn_d=0;
always @(posedge clk) begin
    if (sync_l[1]==btn_l) db_cnt_l<=0;
    else begin db_cnt_l<=db_cnt_l+1; if(db_cnt_l==20'hFFFFF) btn_l<=sync_l[1]; end
    if (sync_r[1]==btn_r) db_cnt_r<=0;
    else begin db_cnt_r<=db_cnt_r+1; if(db_cnt_r==20'hFFFFF) btn_r<=sync_r[1]; end
    if (sync_u[1]==btn_u) db_cnt_u<=0;
    else begin db_cnt_u<=db_cnt_u+1; if(db_cnt_u==20'hFFFFF) btn_u<=sync_u[1]; end
    if (sync_d[1]==btn_d) db_cnt_d<=0;
    else begin db_cnt_d<=db_cnt_d+1; if(db_cnt_d==20'hFFFFF) btn_d<=sync_d[1]; end
end

// ── Palette ───────────────────────────────────────────────────────────────────
reg [11:0] palette [0:15];
integer pi;
initial begin
    for (pi=0; pi<16; pi=pi+1) palette[pi]=12'h000;
    palette[0] = 12'h000;
    palette[1] = 12'h248;  // ceiling
    palette[2] = 12'h433;  // floor
    palette[3] = 12'hA62;  // front/back wall
    palette[4] = 12'h742;  // side wall
end

// ── Player angle ──────────────────────────────────────────────────────────────
reg [9:0] player_angle = 0;

// ── ROMs from hex files ───────────────────────────────────────────────────────
reg signed [15:0] sin_table  [0:1023];
reg signed [9:0]  col_angle  [0:639];
reg        [8:0]  rcp_table  [0:255];

initial begin
    $readmemh("sin_table.hex", sin_table);
    $readmemh("col_angle.hex", col_angle);
    $readmemh("rcp_table.hex", rcp_table);
end

// ── Distance reciprocal LUT ───────────────────────────────────────────────────
// rcp_cos_table[i] = min(32767, floor(8*256*256 / i)), i=1..255
reg [14:0] rcp_cos_table [0:255];
integer rci;
initial begin
    rcp_cos_table[0] = 15'd32767;
    for (rci = 1; rci < 256; rci = rci + 1)
        rcp_cos_table[rci] = (524288 / rci > 32767) ? 15'd32767 : (524288 / rci);
end

// ── Column buffers ────────────────────────────────────────────────────────────
reg [8:0] wall_top  [0:639];
reg [8:0] wall_bot  [0:639];
reg       wall_side [0:639];

// ── Ray engine FSM ────────────────────────────────────────────────────────────
localparam RAY_IDLE  = 3'd0;
localparam RAY_FETCH = 3'd1;
localparam RAY_CALC1 = 3'd2;
localparam RAY_CALC2 = 3'd3;
localparam RAY_WRITE = 3'd4;
localparam RAY_DONE  = 3'd5;

reg [2:0]  ray_state = RAY_IDLE;
reg [9:0]  ray_col   = 0;

reg signed [15:0] r_cos_val;
reg signed [15:0] r_sin_val;
reg        [14:0] r_dist_x;
reg        [14:0] r_dist_y;
reg        [14:0] r_actual;
reg               r_is_side;
// RAY_WRITE scratch regs (must be module-level in Verilog-2001)
reg [7:0]  r_perp_idx;
reg [8:0]  r_strip;
reg [8:0]  r_half_strip;
reg [8:0]  r_wt;
reg [8:0]  r_wb;

wire [9:0] total_angle = (player_angle + col_angle[ray_col][9:0]) & 10'h3FF;
wire [9:0] cos_idx     = (total_angle + 10'd256) & 10'h3FF;

wire [8:0] abs_cos = r_cos_val[15] ? ({1'b0, (~r_cos_val[7:0] + 1'b1)}) : {1'b0, r_cos_val[7:0]};
wire [8:0] abs_sin = r_sin_val[15] ? ({1'b0, (~r_sin_val[7:0] + 1'b1)}) : {1'b0, r_sin_val[7:0]};

always @(posedge clk) begin
    if (rst) begin
        ray_state <= RAY_IDLE;
        ray_col   <= 0;
    end else begin
        case (ray_state)

            RAY_IDLE: begin
                if (vsync_active) begin
                    ray_col   <= 0;
                    ray_state <= RAY_FETCH;
                end
            end

            RAY_FETCH: begin
                r_cos_val <= sin_table[cos_idx];
                r_sin_val <= sin_table[total_angle];
                ray_state <= RAY_CALC1;
            end

            RAY_CALC1: begin
                r_dist_x  <= rcp_cos_table[(abs_cos == 9'd256) ? 8'd255 : abs_cos[7:0]];
                r_dist_y  <= rcp_cos_table[(abs_sin == 9'd256) ? 8'd255 : abs_sin[7:0]];
                ray_state <= RAY_CALC2;
            end

            RAY_CALC2: begin
                if (r_dist_x <= r_dist_y) begin
                    r_actual  <= r_dist_x;
                    r_is_side <= 1'b0;
                end else begin
                    r_actual  <= r_dist_y;
                    r_is_side <= 1'b1;
                end
                ray_state <= RAY_WRITE;
            end

            RAY_WRITE: begin
    r_perp_idx   = (r_actual[14:7] > 8'd255) ? 8'd255 : r_actual[14:7];
    r_strip      = rcp_table[r_perp_idx];
    r_half_strip = {1'b0, r_strip[8:1]};

    r_wt = (9'd240 > r_half_strip) ? (9'd240 - r_half_strip) : 9'd0;
    r_wb = (9'd240 + r_half_strip < 9'd479) ? (9'd240 + r_half_strip) : 9'd479;

    wall_top [ray_col] <= r_wt;
    wall_bot [ray_col] <= r_wb;
    wall_side[ray_col] <= r_is_side;

    if (ray_col == 10'd639)
        ray_state <= RAY_DONE;
    else begin
        ray_col   <= ray_col + 1'b1;
        ray_state <= RAY_FETCH;
    end
end

            RAY_DONE: begin
                if (!vsync_active)
                    ray_state <= RAY_IDLE;
            end

            default: ray_state <= RAY_IDLE;
        endcase
    end
end

// ── Pixel output ──────────────────────────────────────────────────────────────
always @(posedge clk) begin
    if (video_on) begin
        if      (vcnt < wall_top[hcnt])
            {Red, Green, Blue} <= palette[1];
        else if (vcnt <= wall_bot[hcnt])
            {Red, Green, Blue} <= wall_side[hcnt] ? palette[4] : palette[3];
        else
            {Red, Green, Blue} <= palette[2];
    end else
        {Red, Green, Blue} <= 12'h000;
end

// ── Program ROM ───────────────────────────────────────────────────────────────
reg [15:0] program_rom [0:255];
integer fi;
initial begin
    for (fi=0; fi<256; fi=fi+1)
        program_rom[fi] = 16'b1111_0000_0000_0000; // HALT

    program_rom[ 0] = 16'b1100_0001_0000_0000; // SETX  R1,0
    program_rom[ 1] = 16'b1101_1101_0000_0001; // ADDI  R13,R0,1
    program_rom[ 2] = 16'b1100_0111_1111_1111; // SETX  R7,255
    program_rom[ 3] = 16'b0110_0111_0111_0010; // LSHIFT R7,R7,2
    program_rom[ 4] = 16'b1101_0111_0111_0011; // ADDI  R7,R7,3

    program_rom[ 5] = 16'b1110_1100_0000_0000; // LOAD  R12
    program_rom[ 6] = 16'b1101_1011_1011_0000; // NOP
    program_rom[ 7] = 16'b1010_0000_1100_1101; // BLT   R12,R13,-3

    program_rom[ 8] = 16'b1110_1100_0000_0000; // LOAD  R12
    program_rom[ 9] = 16'b1101_1011_1011_0000; // NOP
    program_rom[10] = 16'b1101_1011_1011_0000; // NOP
    program_rom[11] = 16'b1010_0000_0000_1100; // BLT   R0,R12,-4

    program_rom[12] = 16'b1110_0011_0001_0000; // LOAD  R3 BTNR
    program_rom[13] = 16'b1110_0100_0010_0000; // LOAD  R4 BTNL
    program_rom[14] = 16'b0001_0001_0001_0011; // ADD   R1,R1,R3
    program_rom[15] = 16'b0010_0001_0001_0100; // SUB   R1,R1,R4
    program_rom[16] = 16'b0011_0001_0001_0111; // AND   R1,R1,R7
    program_rom[17] = 16'b0000_0101_0001_0000; // WANGLE R1
    program_rom[18] = 16'b1000_0000_0000_0101; // JMP 5
end

// ── CPU core ──────────────────────────────────────────────────────────────────
reg [15:0] registers [0:15];
reg [7:0]  pc;
integer ri;
initial begin
    pc = 0;
    for (ri=0; ri<16; ri=ri+1) registers[ri] = 0;
end

wire [15:0] instr      = program_rom[pc];
wire [3:0]  opcode     = instr[15:12];
wire [3:0]  rd         = instr[11:8];
wire [3:0]  rs1        = instr[7:4];
wire [3:0]  rs2_or_imm = instr[3:0];
wire [7:0]  imm8       = instr[7:0];
wire [11:0] addr12     = instr[11:0];
wire signed [7:0]  br_offs   = {{4{rs2_or_imm[3]}}, rs2_or_imm};
wire signed [15:0] s_rs1_val = registers[rs1];

reg [1:0] cpu_cnt = 0;
wire cpu_tick = (cpu_cnt == 2'd3);
always @(posedge clk)
    cpu_cnt <= (rst || cpu_tick) ? 0 : cpu_cnt + 1'b1;

always @(posedge clk) begin
    if (rst) begin
        pc           <= 0;
        player_angle <= 0;
        for (ri=0; ri<16; ri=ri+1) registers[ri] <= 0;
    end else if (cpu_tick) begin
        case (opcode)
            4'b0000: begin
                case (rd)
                    4'b0101: player_angle <= registers[rs1][9:0];
                    default: ;
                endcase
                pc <= pc + 1'b1;
            end
            4'b0001: begin registers[rd] <= registers[rs1] +  registers[rs2_or_imm]; pc <= pc+1'b1; end
            4'b0010: begin registers[rd] <= registers[rs1] -  registers[rs2_or_imm]; pc <= pc+1'b1; end
            4'b0011: begin registers[rd] <= registers[rs1] &  registers[rs2_or_imm]; pc <= pc+1'b1; end
            4'b0100: begin registers[rd] <= registers[rs1] |  registers[rs2_or_imm]; pc <= pc+1'b1; end
            4'b0101: begin registers[rd] <= ~registers[rs1];                          pc <= pc+1'b1; end
            4'b0110: begin registers[rd] <= registers[rs1] << rs2_or_imm;             pc <= pc+1'b1; end
            4'b0111: begin registers[rd] <= registers[rs1] >> rs2_or_imm;             pc <= pc+1'b1; end
            4'b1000: pc <= addr12[7:0];
            4'b1001: begin
                if (registers[rd]==registers[rs2_or_imm]) pc <= pc+1'b1+br_offs;
                else pc <= pc+1'b1;
            end
            4'b1010: begin
                if (s_rs1_val < $signed(registers[rs2_or_imm])) pc <= pc+1'b1+br_offs;
                else pc <= pc+1'b1;
            end
            4'b1011: pc <= pc+1'b1;
            4'b1100: begin registers[rd] <= {8'h00, imm8};                                       pc <= pc+1'b1; end
            4'b1101: begin registers[rd] <= registers[rs1] + {{12{rs2_or_imm[3]}}, rs2_or_imm}; pc <= pc+1'b1; end
            4'b1110: begin
                case (rs1)
                    4'b0001: registers[rd] <= {15'b0, btn_r};
                    4'b0010: registers[rd] <= {15'b0, btn_l};
                    4'b0011: registers[rd] <= {15'b0, btn_d};
                    4'b0100: registers[rd] <= {15'b0, btn_u};
                    default: registers[rd] <= {15'b0, vsync_r};
                endcase
                pc <= pc+1'b1;
            end
            4'b1111: pc <= pc;
            default: pc <= pc+1'b1;
        endcase
    end
end

endmodule
