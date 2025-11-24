`ifndef LSU
`define LSU
`include "data_memory.sv"
`include "load_unit.sv"
`include "mux3_1.sv"
/*------------------------------------------------------------*/


/*------------------------------------------------------------*/

module lsu (
    input logic i_clk, 
    input logic i_reset, 
    input logic i_lsu_wren,
    input logic [31:0] i_lsu_addr,
    input logic [31:0] i_st_data,  
    output logic [31:0] o_ld_data, 
    input logic [2:0] slt_sl,

    output logic o_lsu_ready,
    /* case slt_st
       SW = 3'b010, SB = 3'b000, SH = 3'b001;
       LW = 3'b101, LB = 3'b011, LH = 3'b100;
       LBU = 3'b110, LHU = 3'b111;
    */
    output logic [31:0] o_io_ledr, //
    output logic [31:0] o_io_ledg,  
    output logic [6:0] o_io_hex0, 
    output logic [6:0] o_io_hex1, 
    output logic [6:0] o_io_hex2,   
    output logic [6:0] o_io_hex3,  // output buffer
    output logic [6:0] o_io_hex4, 
    output logic [6:0] o_io_hex5, 
    output logic [6:0] o_io_hex6,   
    output logic [6:0] o_io_hex7, 
    output logic [31:0] o_io_lcd, //
    
    input logic [31:0] i_io_sw // switch -> input buffer
  );

  localparam RESERVED_1  = 32'h1001_1000;
  localparam SWITCH      = 32'h1001_0000;
  localparam RESERVED_2  = 32'h1000_5000;
  localparam LCD_CONTROL = 32'h1000_4FFF;
  localparam SEG7LEDS_74 = 32'h1000_3FFF;
  localparam SEG7LEDS_30 = 32'h1000_2FFF;
  localparam GREEN_LEDS  = 32'h1000_1FFF;
  localparam RED_LEDS    = 32'h1000_0FFF;
  localparam RESERVED_3  = 32'h0FFF_FFFF;
  localparam MEM_2KB     = 32'h0000_07FF;
  
  /*---------------------------------*/

    logic [31:0] data_out_1, data_out_2, data_out_3; // IO_switchs - peripheral registers - Data (SRAM)
    logic en_datamem;
    logic en_op_buf;
    /*-------- SRAM --------*/
    logic en_sram;
    logic lsu_ready;
  //logic ACK;
    /*-------- Input buffer --------*/
    logic [31:0] INPUT;
    always_ff @(posedge i_clk) begin // ghi đồng bộ
        INPUT <= i_io_sw; 
    end
    assign data_out_1 = i_reset ? 32'd0: INPUT; // đọc bất đồng bộ
    
    assign o_lsu_ready = lsu_ready;
    /*-------- Data mem --------*/


    sram_controller sram (
        .i_clk      (i_clk),
        .i_reset    (i_reset),

        .i_wren     (i_lsu_wren & en_sram),
        .i_addr     (i_lsu_addr),
        .i_wdata    (i_st_data),
        .i_size     (slt_sl),

        .o_rdata    (data_out_3),
        .o_ready    (lsu_ready),

        // SRAM pins
        .SRAM_ADDR  (SRAM_ADDR),
        .SRAM_DQ    (SRAM_DQ),
        .SRAM_WE_N  (SRAM_WE_N),
        .SRAM_OE_N  (SRAM_OE_N),
        .SRAM_UB_N  (SRAM_UB_N),
        .SRAM_LB_N  (SRAM_LB_N),
        .SRAM_CE_N  (SRAM_CE_N)
    );

/*-------- DEMUX --------*/
    demux_sel_mem demux_1 (
        .i_lsu_addr(i_lsu_addr[31:0]),
        .en_datamem(en_sram),   // đổi tên
        .en_op_buf(en_op_buf)  
    );

/*-------- Output buffer --------*/
    output_buffer  outputperiph (
        .slt_sl (slt_sl),
        .st_data_2_i   (i_st_data), 
        .addr_2_i      (i_lsu_addr[31:0]),
        .en_bf         (en_op_buf), 
        .st_en_2_i     (i_lsu_wren),
        .i_clk         (i_clk), 
        .i_reset       (i_reset),
        .data_out_2_o  (data_out_2), 
        .io_lcd_o      (o_io_lcd), 
        .io_ledg_o     (o_io_ledg), 
        .io_ledr_o     (o_io_ledr), 
        .io_hex0_o     (o_io_hex0), 
        .io_hex1_o     (o_io_hex1), 
        .io_hex2_o     (o_io_hex2), 
        .io_hex3_o     (o_io_hex3), 
        .io_hex4_o     (o_io_hex4), 
        .io_hex5_o     (o_io_hex5), 
        .io_hex6_o     (o_io_hex6), 
        .io_hex7_o     (o_io_hex7)
        );
    /*-------- MUX --------*/
    mux_3_1_lsu mux31  (
        .in_data_3_i(data_out_3), 
        .in_data_2_i(data_out_2), 
        .in_data_1_i(data_out_1), 
        .i_lsu_addr(i_lsu_addr[31:0]),
        .o_ld_data(o_ld_data)
        );	

endmodule

module datamem (
  input logic i_clk,
  input logic i_reset,
  input logic i_wren,
  input logic i_enb,
  input logic [2:0] slt_sl,
  input logic [10:0] i_addr,
  input logic [31:0] i_data,
  output logic [31:0] o_data
);
  logic [31:0] data_mem [2047:0];
  logic [31:0] data_bs,data_tmp;

  data_trsf trsf_st (
    .slt_sl(slt_sl),
    .addr_sp(i_addr[1:0]),
    .wr_en(i_wren),
    .data_bf(i_data),
    .data_bs(data_bs),
    .data_af(data_tmp)
  );

  assign o_data = (i_reset == 1'b1) ? 32'h0: data_tmp;

  always @(*) begin
    data_bs = data_mem [i_addr[10:2]];
  end

  always_ff @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
      for (int i=0;i<2048; i++) begin
        data_mem[i] <= 0;
      end
    end
    else if (i_enb) begin
      if (i_wren) begin
        data_mem[i_addr[10:2]] <= data_tmp;
      end
      else 
        data_mem[i_addr[10:2]] <= data_mem[i_addr[10:2]];
    end
  end
endmodule

module demux_sel_mem (
    input logic [31:0] i_lsu_addr,
    output logic en_datamem,
    output logic en_op_buf  
  );
  parameter START_DATAMEM = 32'h0000_0000;
  parameter END_DATAMEM = 32'h0000_0800;
  parameter START_OP_BF = 32'h1000_0000;
  parameter END_OP_BF = 32'h1000_5000;

  always @(*) begin
    if ((i_lsu_addr >= START_DATAMEM) && (i_lsu_addr < END_DATAMEM)) begin
      en_datamem = 1'b1;
      en_op_buf = 1'b0;
    end
    else if ((i_lsu_addr >= START_OP_BF) && (i_lsu_addr < END_OP_BF)) begin
      en_datamem = 1'b0;
      en_op_buf = 1'b1;
    end
    else begin
      en_datamem = 1'b0;
      en_op_buf = 1'b0;
    end
  end
endmodule

module output_buffer(
  input logic [2:0] slt_sl, // chọn store/load kiểu gì (W H B HU BU)
    /* 
       SW = 3'b010, SB = 3'b000, SH = 3'b001;
       LW = 3'b101, LB = 3'b011, LH = 3'b100;
       LBU = 3'b110, LHU = 3'b111;
    */
  input logic [31:0] st_data_2_i, // data
	input logic [31:0] addr_2_i, // address
  input logic en_bf, // en
	input logic st_en_2_i, //write enable
	input logic i_clk,
  input logic i_reset,

	output logic [31:0] data_out_2_o, // data out -> mux
	output logic [31:0] io_lcd_o,
	output logic [31:0] io_ledg_o,
	output logic [31:0] io_ledr_o,
	output logic [6:0] io_hex0_o,
	output logic [6:0] io_hex1_o,
	output logic [6:0] io_hex2_o,
	output logic [6:0] io_hex3_o,
	output logic [6:0] io_hex4_o,
	output logic [6:0] io_hex5_o,
	output logic [6:0] io_hex6_o,
	output logic [6:0] io_hex7_o
	 );

  logic [31:0] data_bs,data_tmp;
  logic [31:0] MEMBF [0:4];

  assign data_out_2_o = (i_reset == 1'b1) ? 32'h0: data_tmp;
  assign io_ledr_o =  MEMBF[0];
  assign io_ledg_o =  MEMBF[1];
  assign io_hex0_o =  MEMBF[2][6:0];  
  assign io_hex1_o =  MEMBF[2][14:8];
  assign io_hex2_o =  MEMBF[2][22:16]; 
  assign io_hex3_o =  MEMBF[2][30:24];
  assign io_hex4_o =  MEMBF[3][6:0];
  assign io_hex5_o =  MEMBF[3][14:8]; 
  assign io_hex6_o =  MEMBF[3][22:16];
  assign io_hex7_o =  MEMBF[3][30:24];    
  assign io_lcd_o  =  MEMBF[4];  

  data_trsf trsf_st (
    .slt_sl(slt_sl),
    .addr_sp(addr_2_i[1:0]),
    .wr_en(st_en_2_i),
    .data_bf(st_data_2_i),
    .data_bs(data_bs),
    .data_af(data_tmp)
  );

  always @(*) begin
    case(addr_2_i[15:12])
      4'h0: data_bs = MEMBF[0]; // LED Red
      4'h1: data_bs = MEMBF[1]; // led green
      4'h2: data_bs = MEMBF[2];// HEX 0-3
      4'h3: data_bs = MEMBF[3];// HEX 4-7
      4'h4: data_bs = MEMBF[4]; //lcd
      default data_bs = 32'h0;
    endcase
  end

  always_ff @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
      MEMBF[0] <= '0;
      MEMBF[1] <= '0;
      MEMBF[2] <= '0;
      MEMBF[3] <= '0;
      MEMBF[4] <= '0;
    end
    else if (en_bf) begin
      if (st_en_2_i) begin
        case(addr_2_i[15:12])
          4'h0: MEMBF[0] <= data_tmp; // LED Red
          4'h1: MEMBF[1] <= data_tmp; // led green
          4'h2: MEMBF[2] <= data_tmp; // HEX 0-3
          4'h3: MEMBF[3] <= data_tmp; //HEX 4-7
          4'h4: MEMBF[4] <= data_tmp; //lcd
          default: begin
            MEMBF[0] <= MEMBF[0];
            MEMBF[1] <= MEMBF[1];
            MEMBF[2] <= MEMBF[2];
            MEMBF[3] <= MEMBF[3];
            MEMBF[4] <= MEMBF[4];
          end
        endcase
      end      
    end
  end
endmodule

module  data_trsf
  (
    input logic [2:0] slt_sl,
    input logic [1:0] addr_sp,
    input logic wr_en,
    input logic [31:0] data_bf,
    input logic [31:0] data_bs,
    output logic [31:0] data_af
  );
  // Định nghĩa giá trị SW, SB, SH, LW, LB, LH, LBU, LHU
  localparam SW = 3'b010, SB = 3'b000, SH = 3'b001;
  localparam LW = 3'b101, LB = 3'b011, LH = 3'b100;
  localparam LBU = 3'b110, LHU = 3'b111;

  logic [31:0] memb_tmp,memh_tmp;

  always @(*) begin
    case (addr_sp) 
      2'b00: begin
      memb_tmp = (data_bs & 32'h000000ff);
      memh_tmp = (data_bs & 32'h0000ffff);
      end
      2'b01: begin
      memb_tmp = (data_bs & 32'h0000ff00);
      memh_tmp = (data_bs & 32'h0000ffff);
      end
      2'b10: begin
      memb_tmp = (data_bs & 32'h00ff0000);
      memh_tmp = (data_bs & 32'hffff0000);
      end
      2'b11: begin
      memb_tmp = (data_bs & 32'hff000000);
      memh_tmp = (data_bs & 32'hffff0000);
      end
    endcase
  end

  always @(*) begin
    if(wr_en) begin
      case (slt_sl) 
        SW: data_af = data_bf;
        SB: begin
          case(addr_sp)
            2'b00: data_af = {data_bs[31:8],data_bf[7:0]};               //(data_bf & 32'h000000ff) | (data_bs & 32'hffffff00);
            2'b01: data_af = {data_bs[31:16],data_bf[7:0],data_bs[7:0]}; //((data_bf & 32'h000000ff) << 8) | (data_bs & 32'hffff00ff);
            2'b10: data_af = {data_bs[31:24],data_bf[7:0],data_bs[15:0]};//((data_bf & 32'h000000ff) << 16)| (data_bs & 32'hff00ffff);
            2'b11: data_af = {data_bf[7:0],data_bs[23:0]};               //((data_bf & 32'h000000ff) << 24)| (data_bs & 32'h00ffffff);
          endcase
        end
        SH: begin
          case (addr_sp)
            2'b00: data_af = {data_bs[31:16],data_bf[15:0]};  //(data_bf & 32'h0000ffff) | (data_bs & 32'hffff0000); 
            2'b01: data_af = {data_bs[31:16],data_bf[15:0]};  //((data_bf & 32'h0000ffff)) | (data_bs & 32'hffff0000); 
            2'b10: data_af = {data_bf[15:0], data_bs[15:0]};  //((data_bf & 32'h0000ffff) << 16) | (data_bs & 32'h0000ffff);
            2'b11: data_af = {data_bf[15:0], data_bs[15:0]};  //((data_bf & 32'h0000ffff) << 16)  | (data_bs & 32'h0000ffff); 
          endcase 
        end
        default: data_af = data_bf;
      endcase
    end
    else begin
      case (slt_sl) 
      LW: data_af = data_bs;
      LBU: begin
        case (addr_sp) 
          2'b00: data_af = memb_tmp;
          2'b01: data_af = {8'b0, memb_tmp[31:8]};  //(memb_tmp >>8);
          2'b10: data_af = {16'b0,memb_tmp[31:16]}; //(memb_tmp >>16);
          2'b11: data_af = {23'b0,memb_tmp[31:24]}; //(memb_tmp >>24);
        endcase
      end
      LHU: begin
        case(addr_sp)
          2'b00: data_af = memh_tmp;
          2'b01: data_af = memh_tmp;
          2'b10: data_af = {16'b0,memh_tmp[31:16]}; //memh_tmp >> 16;
          2'b11: data_af = {16'b0,memh_tmp[31:16]}; //memh_tmp >> 16;
        endcase
      end
      LB: begin
        case (addr_sp)
          2'b00: data_af = (memb_tmp[7])  ? {24'hffffff, memb_tmp[7:0]}   : {24'h000000, memb_tmp[7:0]};
          2'b01: data_af = (memb_tmp[15]) ? {24'hffffff, memb_tmp[15:8]}  : {24'h000000, memb_tmp[15:8]};
          2'b10: data_af = (memb_tmp[23]) ? {24'hffffff, memb_tmp[23:16]} : {24'h000000, memb_tmp[23:16]};
          2'b11: data_af = (memb_tmp[31]) ? {24'hffffff, memb_tmp[31:24]} : {24'h000000, memb_tmp[31:24]};
        endcase
        /*case(addr_sp)
          2'b00: data_af = (memb_tmp[7] == 1) ? (memb_tmp | 32'hffffff00): memb_tmp;
          2'b01: data_af = (memb_tmp[15] == 1)? ((memb_tmp >> 8) | 32'hffffff00) : (memb_tmp >>8);
          2'b10: data_af = (memb_tmp[23] == 1)? ((memb_tmp >> 16) | 32'hffffff00) : (memb_tmp >>16);
          2'b11: data_af = (memb_tmp[31] == 1)? ((memb_tmp >> 24) | 32'hffffff00) : (memb_tmp >>24);
        endcase*/
      end
      
      LH: begin
        case (addr_sp)
          2'b00: data_af = (memh_tmp[15]) ? {16'hffff, memh_tmp[15:0]}   : {16'h0000, memh_tmp[15:0]};
          2'b01: data_af = (memh_tmp[15]) ? {16'hffff, memh_tmp[15:0]}   : {16'h0000, memh_tmp[15:0]};
          2'b10: data_af = (memh_tmp[31]) ? {16'hffff, memh_tmp[31:16]}  : {16'h0000, memh_tmp[31:16]};
          2'b11: data_af = (memh_tmp[31]) ? {16'hffff, memh_tmp[31:16]}  : {16'h0000, memh_tmp[31:16]};
        endcase
        /*case(addr_sp)
          2'b00: data_af = (memh_tmp[15] == 1)? (memh_tmp | 32'hffff0000): memh_tmp;
          2'b01: data_af = (memh_tmp[15] == 1)? (memh_tmp | 32'hffff0000): memh_tmp;
          2'b10: data_af = (memh_tmp[31] == 1)? ((memh_tmp >> 16) | 32'hffff0000) : (memh_tmp >>16);
          2'b11: data_af = (memh_tmp[31] == 1)? ((memh_tmp >> 16) | 32'hffff0000) : (memh_tmp >>16);
        endcase*/
      end
      default: data_af = 32'h00000000;
      endcase
    end
  end
endmodule

module mux_3_1_lsu(
  input logic [31:0] in_data_1_i,
  input logic [31:0] in_data_2_i,
  input logic [31:0] in_data_3_i,
  input logic [31:0] i_lsu_addr,
  output logic [31:0] o_ld_data
);
  logic [1:0] addr_sel ;
 
  always @(*) begin
    case (i_lsu_addr[31:12])
      20'h1001_0:  addr_sel  =  2'b00; // SW
      20'h1000_4:  addr_sel  =  2'b01; //LCD
      20'h1000_3:  addr_sel  =  2'b01; //7-seg
      20'h1000_2:  addr_sel  =  2'b01; //7-seg
      20'h1000_1:  addr_sel  =  2'b01; //GRL
      20'h1000_0:  addr_sel  =  2'b01; //RL
      20'h0000_0:  addr_sel  =  2'b10; //MEM
      default addr_sel = 2'b11; 
    endcase
  end
  always @(*) begin
    case(addr_sel)
    2'b00:o_ld_data = in_data_1_i; // input_bf
    2'b01:o_ld_data = in_data_2_i; // output_bf
    2'b10:o_ld_data = in_data_3_i; // mem
    default: o_ld_data = 32'd0;    
    endcase
  end
endmodule

/*------------------------------------------------------------*/
`endif



module sram_controller (
    input  logic        i_clk,
    input  logic        i_reset,

    // LSU interface
    input  logic        i_wren,          // 1: store, 0: load
    input  logic [31:0] i_addr,
    input  logic [31:0] i_wdata,
    input  logic [2:0]  i_size,          // SB/SH/SW/LB/LH/LW...

    output logic [31:0] o_rdata,
    output logic        o_ready,

    // SRAM interface
    output logic [17:0] SRAM_ADDR,
    inout  logic [15:0] SRAM_DQ,
    output logic        SRAM_WE_N,
    output logic        SRAM_OE_N,
    output logic        SRAM_UB_N,
    output logic        SRAM_LB_N,
    output logic        SRAM_CE_N
);

    // FSM state
    typedef enum logic [1:0] {IDLE, SETUP, ACCESS, DONE} state_t;
    state_t state, next_state;

    // internal
    logic [15:0] write_data_16;
    logic [15:0] read_data_16;
    logic [1:0]  byte_offset;

    assign byte_offset = i_addr[1:0];   // 0~3

    // ----------------------------------------
    // SRAM DQ bus tri-state control
    // ----------------------------------------
    assign SRAM_DQ = (state == ACCESS && i_wren) ? write_data_16 : 16'hZZZZ;

    // ----------------------------------------
    // FSM
    // ----------------------------------------
    always_ff @(posedge i_clk or posedge i_reset) begin
        if (i_reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE:   if (i_size != 3'b000)  next_state = SETUP;   // có yêu cầu
            SETUP:                     next_state = ACCESS;
            ACCESS:                    next_state = DONE;
            DONE:                      next_state = IDLE;
        endcase
    end

    // ----------------------------------------
    // Control logic
    // ----------------------------------------
    always_ff @(posedge i_clk) begin
        case (state)

            IDLE: begin
                o_ready    <= 1;
                SRAM_CE_N  <= 1;      // disable SRAM
                SRAM_WE_N  <= 1;
                SRAM_OE_N  <= 1;
            end

            SETUP: begin
                o_ready <= 0;
                SRAM_CE_N <= 0;       // enable chip

                SRAM_ADDR <= i_addr[17:0];

                // Setup UB/LB for byte/halfword/word
                unique case (i_size)
                    3'b000: begin // SB
                        SRAM_UB_N <= (byte_offset[1] == 1);
                        SRAM_LB_N <= (byte_offset[1] == 0);
                    end
                    3'b001: begin // SH
                        SRAM_UB_N <= 0;
                        SRAM_LB_N <= 0;
                    end
                    default: begin // SW, LW
                        SRAM_UB_N <= 0;
                        SRAM_LB_N <= 0;
                    end
                endcase

                // Write enable or read enable
                if (i_wren)
                    SRAM_WE_N <= 0;   // write
                else
                    SRAM_OE_N <= 0;   // read
            end

            ACCESS: begin
                // Collect read data
                if (!i_wren)
                    read_data_16 <= SRAM_DQ;

                // Prepare write data
                if (i_wren) begin
                    unique case (i_size)
                        3'b000: write_data_16 <= {i_wdata[7:0], i_wdata[7:0]};   // byte
                        3'b001: write_data_16 <= i_wdata[15:0];                  // halfword
                        default: write_data_16 <= i_wdata[15:0];                 // SW lower
                    endcase
                end
            end

            DONE: begin
                // Output 32-bit to LSU
                if (!i_wren) begin
                    unique case (i_size)
                        3'b011: o_rdata <= {{24{read_data_16[7]}}, read_data_16[7:0]};   // LB
                        3'b110: o_rdata <= {24'h0, read_data_16[7:0]};                  // LBU
                        3'b100: o_rdata <= {{16{read_data_16[15]}}, read_data_16};       // LH
                        3'b111: o_rdata <= {16'h0, read_data_16};                       // LHU
                        default: o_rdata <= {16'h0, read_data_16};                     // LW (16-bit SRAM)
                    endcase
                end

                SRAM_WE_N <= 1;
                SRAM_OE_N <= 1;
                SRAM_CE_N <= 1;

                o_ready <= 1;   // CPU resume
            end

        endcase
    end

endmodule
