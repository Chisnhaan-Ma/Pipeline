module cache (
    input  logic         i_clk,
    input  logic         i_reset,

    input  logic         i_wr_en,        // 1 = write, 0 = read
    input  logic [31:0]  i_addr,
    input  logic [31:0]  i_wdata,
    output logic [31:0]  o_rdata,
    output logic         o_hit,
    output logic         o_stall, // stall CPU

    // ============================
    // SRAM INTERFACE
    // ============================
    output logic         o_sram_req,
    output logic [31:0]  o_sram_addr,
    output logic         o_sram_wr_en,
    output logic [31:0]  o_sram_wdata,
    input  logic [31:0]  i_sram_rdata,
    input  logic         i_sram_ready
);

    localparam NUM_SET = 4;
    localparam NUM_WAY = 16;
    logic [3:0] rep_way;
    
    // ---------------------------
    // TAG + INDEX
    // ---------------------------
    logic [1:0] index;
    assign index = i_addr[1:0];
    logic [29:0] tag;
    assign tag  = i_addr[31:2];
    
    // ---------------------------
    // CACHE STORAGE
    // ---------------------------
    logic valid[NUM_SET][NUM_WAY];
    logic dirty[NUM_SET][NUM_WAY];
    logic [29:0] tag_array[NUM_SET][NUM_WAY];
    logic [31:0] data_array[NUM_SET][NUM_WAY];
    logic [3:0] fifo_ptr[NUM_SET];

    // ---------------------------
    // HIT CHECK
    // ---------------------------
    logic [NUM_WAY-1:0] way_hit;
    always_comb begin
        for (int w=0; w<NUM_WAY; w++)
            way_hit[w] = valid[index][w] && (tag_array[index][w] == tag);
    end
    assign hit = |way_hit;

    logic [3:0] hit_way;
    always_comb begin
        hit_way = 0;
        for (int i=0; i<NUM_WAY; i++)
            if (way_hit[i]) hit_way = i;
    end

        // -----------------------------------
    // MISS HANDLING FSM
    // -----------------------------------
    typedef enum logic [1:0] {
        IDLE,
        WRITEBACK,
        REFILL
    } state_t;

    state_t state, next_state;

    // Latch thông tin khi MISS
    logic [31:0] miss_addr;
    logic [31:0] miss_wdata;
    logic miss_wr_en;
    logic [3:0]  victim_way;

    // Output mặc định
    assign o_stall = (state != IDLE);

    // SRAM output mặc định
    assign o_sram_req    = (state != IDLE);
    assign o_sram_addr   = miss_addr;
    assign o_sram_wr_en  = (state == WRITEBACK);
    assign o_sram_wdata  = (state == WRITEBACK) ? 
                            data_array[index][victim_way] : miss_wdata;

    // -------------------------------
    // FSM NEXT STATE LOGIC
    // -------------------------------
    always_comb begin
        next_state = state;

        case (state)

        IDLE: begin
            if (!hit) begin
                // Chọn victim = FIFO
                victim_way = fifo_ptr[index];

                // Lưu miss info
                miss_addr   = i_addr;
                miss_wdata  = i_wdata;
                miss_wr_en  = i_wr_en;

                if (valid[index][victim_way] && dirty[index][victim_way])
                    next_state = WRITEBACK;   // dirty → phải write-back
                else
                    next_state = REFILL;      // clean → refill ngay
            end
        end

        WRITEBACK: begin
            if (i_sram_ready)
                next_state = REFILL;
        end

        REFILL: begin
            if (i_sram_ready)
                next_state = IDLE;
        end

        endcase
    end

    // -------------------------------
    // FSM STATE UPDATE
    // -------------------------------
    always_ff @(posedge i_clk or posedge i_reset) begin
        if (i_reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    // -------------------------------
    // CACHE UPDATE (WRITEBACK & REFILL)
    // -------------------------------
    always_ff @(posedge i_clk) begin
        if (state == REFILL && i_sram_ready) begin
            // Ghi line mới vào victim way
            tag_array[index][victim_way]   <= tag;
            data_array[index][victim_way]  <= i_sram_rdata;
            valid[index][victim_way] <= 1'b1;

            // Nếu miss là write → cập nhật giá trị user ghi
            if (miss_wr_en) begin
                data_array[index][victim_way] <= miss_wdata;
                dirty[index][victim_way]      <= 1'b1;
            end else begin
                dirty[index][victim_way] <= 1'b0;
            end

            // FIFO replacement pointer
            fifo_ptr[index] <= fifo_ptr[index] + 1;
        end

        // HIT → update data ngay
        if (hit && i_wr_en) begin
            data_array[index][hit_way] <= i_wdata;
            dirty[index][hit_way]      <= 1'b1;
        end
    end

    // -------------------------------
    // READ OUTPUT
    // -------------------------------
    assign o_hit = hit;

    always_comb begin
        if (hit)
            o_rdata = data_array[index][hit_way];
        else
            o_rdata = i_sram_rdata;   // trả về khi refill xong
    end

// ============================
// STALL SIGNAL
// ============================
//assign o_stall = (state == MISS) || (state == REFILL && !i_sram_ready);
endmodule
