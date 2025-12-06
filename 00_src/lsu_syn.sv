// Author: Nhu Bui
//`ifndef LSU
//`define LSU
////`include "data_memory.sv"
////`include "load_unit.sv"
////`include "mux3_1.sv"

/*------------------------------------------------------------*/

module lsu_syn (
    input logic i_clk, 
    input logic i_reset, 
    input logic i_lsu_wren,
    input logic [31:0] i_lsu_addr,
    input logic [31:0] i_st_data,  
    output logic [31:0] o_ld_data, 
    input logic [2:0] slt_sl,
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
  logic [31:0] ld_data_tmp;

  logic [31:0] input_bf_tmp;
  logic [31:0] output_bf_tmp;
  logic [31:0] data_mem_tmp;
  logic [2:0]  slt_sl_tmp;
/*-------- Input buffer --------*/
  logic [31:0] INPUT;
  always_ff @(posedge i_clk) begin // ghi đồng bộ
    INPUT <= i_io_sw;
  end
  assign input_bf_tmp = i_reset ? 32'd0: INPUT; // đọc bất đồng bộ

//---------Syn Read------------
 always_ff @ (posedge i_clk) begin
    data_out_1 <= input_bf_tmp;
    //data_out_2 <= output_bf_tmp;
    slt_sl_tmp <= slt_sl; 
 end

/*-------- Data mem --------*/
  datamem_syn mem (
    .i_clk  (i_clk),
    .i_reset(i_reset),
    .i_wren (i_lsu_wren),
    .slt_sl (slt_sl),
    .i_enb  (en_datamem),
    .i_addr (i_lsu_addr[15:0]), // XXXXXXXXx
    .i_data (i_st_data),
    .o_data (/*data_mem_tmp*/ data_out_3)
  );

/*-------- DEMUX --------*/
  demux_sel_mem demux_1 (
    .i_lsu_addr(i_lsu_addr[31:0]),
    .en_datamem(en_datamem),
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
    .data_out_2_o  (/*output_bf_tmp*/ data_out_2), 
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
    .i_clk      (i_clk),
    .in_data_3_i(data_out_3), 
    .in_data_2_i(data_out_2), 
    .in_data_1_i(data_out_1), 
    .i_lsu_addr(i_lsu_addr[31:0]),
    .o_ld_data(o_ld_data)
    );	
endmodule

module datamem_syn (
  input logic i_clk,
  input logic i_reset,
  input logic i_wren,
  input logic i_enb,
  input logic [2:0] slt_sl,
  input logic [15:0] i_addr,
  input logic [31:0] i_data,
  output logic [31:0] o_data
);
  logic [31:0] data_mem [16383:0];
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
    data_bs = data_mem [i_addr[15:2]];
  end

  always_ff @(posedge i_clk ) begin
    if (i_reset) begin
      for (int i=0;i<16384; i++) begin
        data_mem[i] <= 0;
      end
    end
    else if (i_enb) begin
      if (i_wren) begin
        data_mem[i_addr[15:2]] <= data_tmp;
      end
      else 
        data_mem[i_addr[15:2]] <= data_mem[i_addr[15:2]];
    end
  end
endmodule

module mask_st_create
  (
    input logic [2:0] i_slt_sl,
    input logic [1:0] i_addr_sp,
    output logic[3:0] o_byte_mask
  );
  localparam SW = 3'b010, SB = 3'b000, SH = 3'b001;
  localparam LW = 3'b101, LB = 3'b011, LH = 3'b100;
  localparam LBU = 3'b110, LHU = 3'b111;

  always_comb begin
    case (i_slt_sl)
      SW:   o_byte_mask = 4'b1111;
      SB:   begin
          case (i_addr_sp)
            2'b00:   o_byte_mask = 4'b0001;
            2'b01:   o_byte_mask = 4'b0010;
            2'b10:   o_byte_mask = 4'b0100;
            2'b11:   o_byte_mask = 4'b1000;
            default: o_byte_mask = 4'b0001;
          endcase
        end
      SH:   begin
          case (i_addr_sp)
            2'b00:   o_byte_mask = 4'b0011;
            2'b01:   o_byte_mask = 4'b0011;
            2'b10:   o_byte_mask = 4'b1100;
            2'b11:   o_byte_mask = 4'b1100;
            default: o_byte_mask = 4'b0011;
          endcase
        end
      default: o_byte_mask = 4'b1111;
    endcase
  end
endmodule

module mask_ld_create
  (
    input logic [2:0] i_slt_sl,
    input logic [1:0] i_addr_sp,
    output logic[3:0] o_byte_mask
  );
  localparam SW = 3'b010, SB = 3'b000, SH = 3'b001;
  localparam LW = 3'b101, LB = 3'b011, LH = 3'b100;
  localparam LBU = 3'b110, LHU = 3'b111;

  always_comb begin
    case (i_slt_sl)
      LW:   o_byte_mask = 4'b1111;
      LB:   begin
          case (i_addr_sp)
            2'b00:   o_byte_mask = 4'b0001;
            2'b01:   o_byte_mask = 4'b0010;
            2'b10:   o_byte_mask = 4'b0100;
            2'b11:   o_byte_mask = 4'b1000;
            default: o_byte_mask = 4'b0001;
          endcase
        end
      LH:   begin
          case (i_addr_sp)
            2'b00:   o_byte_mask = 4'b0011;
            2'b01:   o_byte_mask = 4'b0011;
            2'b10:   o_byte_mask = 4'b1100;
            2'b11:   o_byte_mask = 4'b1100;
            default: o_byte_mask = 4'b0011;
          endcase
        end
      LBU:  begin
          case (i_addr_sp)
            2'b00:   o_byte_mask = 4'b0001;
            2'b01:   o_byte_mask = 4'b0010;
            2'b10:   o_byte_mask = 4'b0100;
            2'b11:   o_byte_mask = 4'b1000;
            default: o_byte_mask = 4'b0001;
          endcase
        end
      LHU:  begin
          case (i_addr_sp)
            2'b00:   o_byte_mask = 4'b0011;
            2'b01:   o_byte_mask = 4'b0011;
            2'b10:   o_byte_mask = 4'b1100;
            2'b11:   o_byte_mask = 4'b1100;
            default: o_byte_mask = 4'b0011;
          endcase
        end
    endcase
  end
endmodule

module datamem_syn (
    input  logic        i_clk,
    input  logic        i_reset,
    input  logic        i_wren,
    input  logic        i_enb,
    input  logic [2:0]  slt_sl,
    input  logic [15:0] i_addr, 
    input  logic [31:0] i_data,
    output logic [31:0] o_data
    );
    logic [31:0] data_mem [16383:0];
    logic [3:0] bmask_st, bmask_ld;
    logic [31:0] data_ld_bf;

    mask_st_create mask_st (
        .i_slt_sl    (slt_sl),
        .i_addr_sp   (i_addr[1:0]),
        .o_byte_mask (bmask_st)
    );
    mask_ld_create mask_ld (
        .i_slt_sl    (slt_sl),
        .i_addr_sp   (i_addr[1:0]),
        .o_byte_mask (bmask_ld)
    );

    always_ff @(posedge i_clk) begin
        if (i_reset) begin
            for (int i = 0; i < 16384; i++)
                data_mem[i] <= 32'h0;
        end
        else if (i_enb) begin
            if (i_wren) begin
                if (bmask_st[0]) data_mem[{i_addr[15:2],2'b00}] <= i_data[7:0];

                if (bmask_st[1]) data_mem[{i_addr[15:2],2'b01}] <= i_data[15:8];

                if (bmask_st[2]) data_mem[{i_addr[15:2],2'b10}] <= i_data[23:16];

                if (bmask_st[3]) data_mem[{i_addr[15:2],2'b11}] <= i_data[31:24];
            end
            else 
            data_ld_bf <= data_mem[i_addr[15:2]];  // --- synchronous read ---
        end
    end


    always_comb begin
        if (i_reset)
            o_data = 32'b0;
        else
            begin
                o_data  [7:0]   =   data_ld_bf  [7:0]   & bmask_ld [0];
                o_data  [15:8]  =   data_ld_bf  [15:8]  & bmask_ld [1];
                o_data  [23:16] =   data_ld_bf  [23:16] & bmask_ld [2];
                o_data  [31:24] =   data_ld_bf  [31:24] & bmask_ld [3];
            end
    end

endmodule

`ifndef DEMUX_SEL_MEM
`define DEMUX_SEL_MEM

module demux_sel_mem (
    input logic [31:0]  i_lsu_addr,
    output logic        en_datamem,
    output logic        en_op_buf  
  );
  always @(*) begin
    en_datamem  = 1'b0;
    en_op_buf   = 1'b0;
    if (i_lsu_addr[31:28]==4'b0) begin //0xxx_xxxx
      en_datamem  = 1'b1;
      en_op_buf   = 1'b0;
    end
    else if (i_lsu_addr[19:16]==4'b0) begin //1xx0_xxxx
      en_datamem  = 1'b0;
      en_op_buf   = 1'b1;
    end
    else begin //1xx1_xxxx
      en_datamem  = 1'b0;
      en_op_buf   = 1'b0;
    end
  end
endmodule
`endif

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

  //logic [31:0] data_bs,data_tmp;
    logic [31:0] MEMBF [0:4];
    logic [3:0] bmask_st, bmask_ld;
    logic [31:0] data_ld_bf;

    mask_st_create mask_st_output_bf (
        .i_slt_sl    (slt_sl),
        .i_addr_sp   (addr_2_i[1:0]),
        .o_byte_mask (bmask_st)
    );
    mask_ld_create mask_ld_output_bf (
        .i_slt_sl    (slt_sl),
        .i_addr_sp   (addr_2_i[1:0]),
        .o_byte_mask (bmask_ld)
    );

   always_ff @(posedge i_clk) begin
        if (i_reset) begin
            MEMBF[0] <= '0;
            MEMBF[1] <= '0;
            MEMBF[2] <= '0;
            MEMBF[3] <= '0;
            MEMBF[4] <= '0;
        end
        else if (en_bf) begin
            if (st_en_2_i) begin
                if (bmask_st[0]) MEMBF[addr_2_i[15:12]][7:0] <= st_data_2_i[7:0];

                if (bmask_st[1]) MEMBF[addr_2_i[15:12]][15:8] <= st_data_2_i[15:8];

                if (bmask_st[2]) MEMBF[addr_2_i[15:12]][23:16] <= st_data_2_i[23:16];

                if (bmask_st[3]) MEMBF[addr_2_i[15:12]][31:24] <= st_data_2_i[31:24];
            end
            //else 
            data_ld_bf <= MEMBF[addr_2_i[15:12]];  // --- synchronous read ---
        end
    end

    always_comb begin
        if (i_reset)
            data_out_2_o = 32'b0;
        else
            begin
                data_out_2_o  [7:0]   =   data_ld_bf  [7:0]   & bmask_ld [0];
                data_out_2_o  [15:8]  =   data_ld_bf  [15:8]  & bmask_ld [1];
                data_out_2_o  [23:16] =   data_ld_bf  [23:16] & bmask_ld [2];
                data_out_2_o  [31:24] =   data_ld_bf  [31:24] & bmask_ld [3];
            end
    end


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
  input logic        i_clk,
  input logic [31:0] in_data_1_i,
  input logic [31:0] in_data_2_i,
  input logic [31:0] in_data_3_i,
  input logic [31:0] i_lsu_addr,
  output logic [31:0] o_ld_data
);
  logic [1:0] addr_sel ;
  logic [1:0] addr_sel_tmp;
  always_comb begin
      case (i_lsu_addr[31:16])
        16'h1001:  addr_sel  =  2'b00; // SW
        16'h1000:  addr_sel  =  2'b01; // LCD
        16'h0000:  addr_sel  =  2'b10; // RL
        default:   addr_sel = 2'b11; 
      endcase
  end

  always_ff @ (posedge i_clk) begin
    addr_sel_tmp <= addr_sel;
  end

  always_comb begin
    case(addr_sel_tmp)
      2'b00:o_ld_data = in_data_1_i; // input_bf
      2'b01:o_ld_data = in_data_2_i; // output_bf
      2'b10:o_ld_data = in_data_3_i; // mem
      default: o_ld_data = 32'd0;    
    endcase
  end
/*
  always_ff @ (posedge i_clk) begin
	o_ld_data <= load_data_tmp;
  end
  */
endmodule

/*------------------------------------------------------------*/
//`endif
